"""Execute exactly one safety-gated Nav2 acceptance leg and write JSON evidence.

This node has no velocity or actuator publisher.  Its only publisher is the
irreversible navigation stop latch consumed by the separately launched bridge.
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool, Empty
from tf2_ros import Buffer, TransformException, TransformListener

from .navigation_acceptance_logic import (
    CommandPoseResponseMonitor,
    LegProgressMonitor,
    graph_preflight_errors,
    load_leg_spec,
    load_resume_reference,
    source_pose_errors,
)
from .route_points_logic import loaded_map_binding
from .scene_pose_recorder_logic import quaternion_to_yaw


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def atomic_json(path, document):
    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + '.tmp')
    temporary.write_text(
        json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + '\n',
        encoding='utf-8',
    )
    temporary.replace(destination)


class NavigationAcceptanceLeg(Node):
    def __init__(self, spec, source_reference, options, evidence):
        super().__init__('navigation_acceptance_leg')
        self.spec = spec
        self.source_reference = source_reference
        self.options = options
        self.evidence = evidence
        self.pose = None
        self.live_map_binding = None
        self.map_received_at = None
        self.manual_suppressed = False
        self.manual_suppressed_at = None
        self.navigation_active = False
        self.navigation_active_at = None
        self.goal_handle = None
        self.monitor = None
        self.goal_submitted = False
        self.goal_submitted_at = None
        self.abort_reason = None
        self.abort_deadline = None
        self.last_stop_publish_at = None
        self.done = False
        self.exit_code = 2
        self.feedback_rows = []
        self.motion_rows = []
        self.motion_monitor = CommandPoseResponseMonitor()
        self.latest_controller_command = None
        self.latest_map_pose = None
        self.last_map_transform_stamp_ns = None
        self.last_motion_sample_at = None
        self.last_checkpoint_at = None

        transient = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, transient)
        self.create_subscription(
            Twist, '/controller/cmd_vel', self._on_controller_command, 20)
        self.create_subscription(
            Bool, '/raicom/manual_suppressed', self._on_manual_suppressed, 10)
        self.create_subscription(
            Bool, '/raicom/navigation_active', self._on_navigation_active, transient)
        self.stop_publisher = self.create_publisher(Empty, '/raicom/nav_stop', 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.create_timer(0.1, self._tick)

    def _checkpoint(self, force=False):
        now = time.monotonic()
        if (not force and self.last_checkpoint_at is not None and
                now - self.last_checkpoint_at < 1.0):
            return
        try:
            atomic_json(self.options.evidence_file, self.evidence)
            self.last_checkpoint_at = now
        except Exception as exc:  # evidence loss must stop an active leg
            self.get_logger().error(f'evidence checkpoint failed: {exc}')
            self.evidence['evidence_write_error'] = str(exc)
            if self.goal_submitted and self.abort_reason is None:
                self.abort_reason = f'evidence_checkpoint_failed:{exc}'
                self.evidence['abort_reason'] = self.abort_reason
                self.abort_deadline = now + 3.0
                self._publish_stop()
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()

    @staticmethod
    def _owners(infos):
        return sorted(info.node_name for info in infos)

    def graph_snapshot(self):
        return {
            'controller_publishers': self._owners(
                self.get_publishers_info_by_topic('/controller/cmd_vel')),
            'staging_publishers': self._owners(
                self.get_publishers_info_by_topic('/raicom/nav2_staging/cmd_vel')),
            'staging_subscribers': self._owners(
                self.get_subscriptions_info_by_topic('/raicom/nav2_staging/cmd_vel')),
            'localization_publishers': self._owners(
                self.get_publishers_info_by_topic('/amcl_pose')),
            'map_publishers': self._owners(self.get_publishers_info_by_topic('/map')),
        }

    def _on_map(self, message):
        try:
            self.live_map_binding = loaded_map_binding(message)
        except ValueError as exc:
            self.evidence['live_map_error'] = str(exc)
            return
        self.map_received_at = time.monotonic()

    def _on_manual_suppressed(self, message):
        self.manual_suppressed = bool(message.data)
        self.manual_suppressed_at = time.monotonic()

    def _on_navigation_active(self, message):
        self.navigation_active = bool(message.data)
        self.navigation_active_at = time.monotonic()

    def _on_controller_command(self, message):
        now = time.monotonic()
        self.latest_controller_command = {
            'received_at': now,
            'linear_x_mps': float(message.linear.x),
            'linear_y_mps': float(message.linear.y),
            'angular_z_radps': float(message.angular.z),
        }
        reason = self.motion_monitor.observe_command(
            now, message.linear.x, message.linear.y, message.angular.z)
        self._update_motion_evidence(now)
        if reason is not None and self.abort_reason is None:
            self.request_abort(reason)

    def _sample_map_pose(self, now):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time())
        except TransformException:
            return
        stamp_ns = (int(transform.header.stamp.sec) * 1_000_000_000
                    + int(transform.header.stamp.nanosec))
        if stamp_ns == self.last_map_transform_stamp_ns:
            return
        self.last_map_transform_stamp_ns = stamp_ns
        q = transform.transform.rotation
        yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.latest_map_pose = {
            'received_at': now,
            'stamp_ns': stamp_ns,
            'x': float(transform.transform.translation.x),
            'y': float(transform.transform.translation.y),
            'yaw_rad': yaw,
        }
        reason = self.motion_monitor.observe_pose(
            now, self.latest_map_pose['x'], self.latest_map_pose['y'], yaw)
        self._update_motion_evidence(now)
        if reason is not None and self.abort_reason is None:
            self.request_abort(reason)

    def _update_motion_evidence(self, now=None, force=False):
        now = time.monotonic() if now is None else float(now)
        summary = self.motion_monitor.snapshot(now)
        motion_evidence = self.evidence.setdefault('motion_response', {})
        motion_evidence['summary'] = summary
        motion_evidence['samples'] = self.motion_rows
        if (not force and self.last_motion_sample_at is not None and
                now - self.last_motion_sample_at < 0.2):
            return

        def current(values, keys):
            if values is None:
                return None
            row = {key: values[key] for key in keys}
            row['age_sec'] = max(0.0, now - values['received_at'])
            return row

        self.motion_rows.append({
            'elapsed_sec': (
                None if self.motion_monitor.armed_at is None
                else max(0.0, now - self.motion_monitor.armed_at)),
            'controller_command': current(
                self.latest_controller_command,
                ('linear_x_mps', 'linear_y_mps', 'angular_z_radps')),
            'map_pose': current(
                self.latest_map_pose, ('stamp_ns', 'x', 'y', 'yaw_rad')),
            'command_effort_equivalent_m': summary['command_effort_equivalent_m'],
            'map_pose_response_equivalent_m': (
                summary['map_pose_response_equivalent_m']),
            'response_ratio': summary['response_ratio'],
        })
        if len(self.motion_rows) > 600:
            del self.motion_rows[:100]
        self.last_motion_sample_at = now

    @staticmethod
    def _fresh(received_at, now, max_age):
        return received_at is not None and 0.0 <= now - received_at <= max_age

    def preflight_errors(self):
        now = time.monotonic()
        errors = graph_preflight_errors(self.graph_snapshot())
        if not self.nav_client.server_is_ready():
            errors.append('navigate_to_pose action server is unavailable')
        if self.live_map_binding is None:
            errors.append('live transient-local /map has not been received')
        elif self.live_map_binding != self.spec.loaded_map_binding:
            errors.append('live /map binding differs from the route journal')
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', Time())
        except TransformException as exc:
            errors.append(f'live map->base_footprint TF is unavailable: {exc}')
        else:
            q = transform.transform.rotation
            stamp_ns = (int(transform.header.stamp.sec) * 1_000_000_000
                        + int(transform.header.stamp.nanosec))
            transform_age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
            self.pose = {
                'frame': transform.header.frame_id,
                'child_frame': transform.child_frame_id,
                'x': float(transform.transform.translation.x),
                'y': float(transform.transform.translation.y),
                'yaw': quaternion_to_yaw(q.x, q.y, q.z, q.w),
                'age_sec': transform_age,
            }
            if not 0.0 <= transform_age <= self.options.message_max_age:
                errors.append(
                    f'map->base_footprint TF age {transform_age:.3f} s exceeds limit')
            position_error, heading_error = source_pose_errors(
                self.pose['x'], self.pose['y'], self.pose['yaw'], self.source_reference)
            self.evidence['source_pose_check'] = {
                'current': dict(self.pose),
                'reference': dict(self.source_reference),
                'position_error_m': position_error,
                'heading_error_rad': heading_error,
                'position_limit_m': self.options.source_position_tolerance,
                'heading_limit_rad': self.options.source_heading_tolerance,
            }
            if position_error > self.options.source_position_tolerance:
                errors.append(f'source position error {position_error:.3f} m exceeds limit')
            if heading_error > self.options.source_heading_tolerance:
                errors.append(f'source heading error {heading_error:.3f} rad exceeds limit')
        if not (self.navigation_active and self._fresh(
                self.navigation_active_at, now, self.options.message_max_age)):
            errors.append('navigation bridge active acknowledgement is false or stale')
        if not (self.manual_suppressed and self._fresh(
                self.manual_suppressed_at, now, self.options.suppression_max_age)):
            errors.append('manual input suppression acknowledgement is false or stale')
        return errors

    def submit(self):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self.spec.target['x'])
        goal.pose.pose.position.y = float(self.spec.target['y'])
        yaw = float(self.spec.target['yaw_rad'])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_submitted = True
        self.goal_submitted_at = time.monotonic()
        self.evidence['phase'] = 'goal_submitted'
        self.evidence['goal_submitted_at_utc'] = utc_now()
        self._checkpoint(force=True)
        if self.abort_reason is not None:
            return
        future = self.nav_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:  # pragma: no cover - DDS transport path
            self.request_abort(f'goal_transport_error:{exc}')
            return
        if not handle.accepted:
            self.request_abort('goal_rejected')
            return
        self.goal_handle = handle
        if self.abort_reason is not None:
            self.evidence['late_goal_accepted_after_abort'] = True
            handle.cancel_goal_async()
            handle.get_result_async().add_done_callback(self._on_result)
            self._checkpoint(force=True)
            return
        started = time.monotonic()
        self.monitor = LegProgressMonitor(
            started_at=started,
            max_recoveries=self.options.max_recoveries,
            feedback_timeout_sec=self.options.feedback_timeout,
            progress_timeout_sec=self.options.progress_timeout,
            leg_timeout_sec=self.options.leg_timeout,
            min_progress_m=self.options.min_progress,
            progress_exempt_distance_m=self.options.progress_exempt_distance,
        )
        initial_map_pose = None
        if self.latest_map_pose is not None:
            initial_map_pose = (
                self.latest_map_pose['x'], self.latest_map_pose['y'],
                self.latest_map_pose['yaw_rad'])
        self.motion_monitor.arm(started, initial_map_pose)
        self._update_motion_evidence(started, force=True)
        self.evidence['phase'] = 'goal_accepted'
        self.evidence['goal_accepted_at_utc'] = utc_now()
        handle.get_result_async().add_done_callback(self._on_result)
        self._checkpoint(force=True)

    def _on_feedback(self, message):
        feedback = message.feedback
        now = time.monotonic()
        row = {
            'elapsed_sec': None if self.monitor is None else now - self.monitor.started_at,
            'distance_remaining_m': float(feedback.distance_remaining),
            'number_of_recoveries': int(feedback.number_of_recoveries),
            'pose': {
                'x': float(feedback.current_pose.pose.position.x),
                'y': float(feedback.current_pose.pose.position.y),
                'z': float(feedback.current_pose.pose.orientation.z),
                'w': float(feedback.current_pose.pose.orientation.w),
            },
        }
        self.feedback_rows.append(row)
        if len(self.feedback_rows) > 600:
            del self.feedback_rows[:100]
        self.evidence['feedback'] = self.feedback_rows
        if self.monitor is not None and self.abort_reason is None:
            reason = self.monitor.observe(
                now, feedback.distance_remaining, feedback.number_of_recoveries)
            if reason:
                self.request_abort(reason)
        self._checkpoint()

    def _publish_stop(self):
        for _ in range(3):
            self.stop_publisher.publish(Empty())
        self.last_stop_publish_at = time.monotonic()
        self.evidence['stop_latch_published'] = True

    def request_abort(self, reason):
        if self.abort_reason is None:
            self.abort_reason = str(reason)
            self._update_motion_evidence(force=True)
            self.evidence['abort_reason'] = self.abort_reason
            self.evidence['phase'] = 'cancelling'
            self.evidence['abort_requested_at_utc'] = utc_now()
            self.abort_deadline = time.monotonic() + 3.0
            self._publish_stop()
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self._checkpoint(force=True)

    def _on_result(self, future):
        try:
            wrapped = future.result()
            status = int(wrapped.status)
        except Exception as exc:  # pragma: no cover - DDS transport path
            self.request_abort(f'result_transport_error:{exc}')
            return
        self.evidence['nav2_status'] = status
        self.evidence['result_at_utc'] = utc_now()
        self._update_motion_evidence(force=True)
        if status == GoalStatus.STATUS_SUCCEEDED and self.abort_reason is None:
            self.evidence['outcome'] = 'SUCCEEDED'
            self.evidence['phase'] = 'terminal'
            self.exit_code = 0
            self._publish_stop()
            self.done = True
        else:
            if self.abort_reason is None:
                self.abort_reason = f'nav2_non_success_status:{status}'
                self.evidence['abort_reason'] = self.abort_reason
                self._publish_stop()
            self.evidence['outcome'] = 'NOT_SUCCEEDED'
            self.evidence['phase'] = 'terminal'
            self.exit_code = 2
            self.done = True
        self._checkpoint(force=True)

    def _tick(self):
        now = time.monotonic()
        if (self.goal_submitted and self.goal_handle is None and
                self.abort_reason is None and self.goal_submitted_at is not None and
                now - self.goal_submitted_at > self.options.goal_response_timeout):
            self.request_abort(
                f'goal_response_timeout:{now - self.goal_submitted_at:.3f}s')
        if self.monitor is not None and self.abort_reason is None and not self.done:
            self._sample_map_pose(now)
        if self.monitor is not None and self.abort_reason is None and not self.done:
            reason = self.motion_monitor.check(now)
            self._update_motion_evidence(now)
            if reason:
                self.request_abort(reason)
        if self.monitor is not None and self.abort_reason is None and not self.done:
            reason = self.monitor.check(now)
            if reason:
                self.request_abort(reason)
        if self.abort_reason is not None and not self.done:
            if self.last_stop_publish_at is None or now - self.last_stop_publish_at >= 0.5:
                self._publish_stop()
            if self.abort_deadline is not None and now >= self.abort_deadline:
                self.evidence['outcome'] = 'ABORTED_WITHOUT_TERMINAL_RESULT'
                self.evidence['phase'] = 'terminal'
                self.exit_code = 2
                self.done = True


def parse_args(args=None):
    parser = argparse.ArgumentParser(description='One authorized Nav2 acceptance leg')
    parser.add_argument('--route-file', required=True)
    parser.add_argument('--map-file', required=True)
    parser.add_argument('--approach-file', required=True)
    parser.add_argument('--source', required=True)
    parser.add_argument('--target', required=True)
    parser.add_argument('--evidence-file', required=True)
    parser.add_argument('--resume-evidence-file')
    parser.add_argument('--operator-authorized', action='store_true', default=False)
    parser.add_argument('--preflight-only', action='store_true', default=False)
    parser.add_argument('--preflight-timeout', type=float, default=8.0)
    parser.add_argument('--message-max-age', type=float, default=1.0)
    parser.add_argument('--suppression-max-age', type=float, default=0.25)
    parser.add_argument('--source-position-tolerance', type=float, default=0.10)
    parser.add_argument('--source-heading-tolerance', type=float, default=math.radians(20.0))
    parser.add_argument('--max-recoveries', type=int, default=2)
    parser.add_argument('--goal-response-timeout', type=float, default=2.0)
    parser.add_argument('--feedback-timeout', type=float, default=2.0)
    parser.add_argument('--progress-timeout', type=float, default=8.0)
    parser.add_argument('--leg-timeout', type=float, default=60.0)
    parser.add_argument('--min-progress', type=float, default=0.02)
    parser.add_argument('--progress-exempt-distance', type=float, default=0.08)
    return parser.parse_known_args(args)


def main(args=None):
    options, ros_args = parse_args(args)
    evidence = {
        'schema': 'raicom_navigation_acceptance/v1',
        'created_at_utc': utc_now(),
        'operator_authorized': bool(options.operator_authorized),
        'phase': 'configuration',
        'stop_latch_published': False,
    }
    try:
        spec = load_leg_spec(
            options.route_file, options.map_file, options.source, options.target,
            options.approach_file)
        source_reference = spec.source
        if options.resume_evidence_file:
            source_reference = load_resume_reference(options.resume_evidence_file, spec)
        evidence['leg'] = {
            'source': spec.source_name,
            'target': spec.target_name,
            'target_pose': spec.target,
            'work_target_pose': spec.work_target,
            'route_sha256': spec.route_sha256,
            'approach_sha256': spec.approach_sha256,
            'static_approach_checks': spec.static_checks,
            'terminal_rotation_checks': spec.rotation_checks,
            'source_reference_kind': spec.source_reference_kind,
        }
        evidence['source_reference'] = source_reference
        if not options.operator_authorized and not options.preflight_only:
            raise ValueError('Explicit --operator-authorized flag is required')
    except Exception as exc:
        evidence['outcome'] = 'CONFIGURATION_REJECTED'
        evidence['error'] = str(exc)
        evidence['phase'] = 'terminal'
        atomic_json(options.evidence_file, evidence)
        raise SystemExit(2)

    stop_requested = False

    def on_signal(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = NavigationAcceptanceLeg(spec, source_reference, options, evidence)
    try:
        deadline = time.monotonic() + options.preflight_timeout
        # A newly joined TF listener can receive a sizeable reliable-history
        # backlog.  Running the graph/parameter checks after every single
        # callback prevents the executor from ever catching up to the newest
        # transform on a loaded Pi.  Drain callbacks continuously and perform
        # the comparatively expensive preflight checks at a bounded cadence.
        next_preflight_check = time.monotonic()
        spin_iterations = 0
        preflight_checks = 0
        errors = ['preflight has not run']
        while rclpy.ok() and time.monotonic() < deadline and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.02)
            spin_iterations += 1
            now = time.monotonic()
            if now < next_preflight_check:
                continue
            errors = node.preflight_errors()
            preflight_checks += 1
            if not errors:
                break
            next_preflight_check = now + 0.5
        evidence['preflight'] = {
            'checked_at_utc': utc_now(),
            'graph': node.graph_snapshot(),
            'errors': errors,
            'passed': not errors,
            'spin_iterations': spin_iterations,
            'checks': preflight_checks,
        }
        if errors or stop_requested:
            if not options.preflight_only:
                node.request_abort('preflight_failed:' + '; '.join(errors) if errors
                                   else 'signal_before_goal')
            evidence['outcome'] = 'PREFLIGHT_REJECTED'
            evidence['phase'] = 'terminal'
            node.done = True
        elif options.preflight_only:
            evidence['outcome'] = 'PREFLIGHT_PASSED_NO_GOAL'
            evidence['phase'] = 'terminal'
            node.exit_code = 0
            node.done = True
        else:
            evidence['phase'] = 'preflight_passed'
            atomic_json(options.evidence_file, evidence)
            node.submit()
            while rclpy.ok() and not node.done:
                rclpy.spin_once(node, timeout_sec=0.1)
                if stop_requested and node.abort_reason is None:
                    node.request_abort('signal_interrupted')
        # Keep the DDS writer alive briefly so the bridge can receive the
        # three terminal stop-latch samples before this one-shot node exits.
        stop_delivery_deadline = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < stop_delivery_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        evidence['finished_at_utc'] = utc_now()
        atomic_json(options.evidence_file, evidence)
        raise SystemExit(node.exit_code)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
