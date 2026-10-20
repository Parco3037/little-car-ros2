"""ROS-independent queue orchestration; Nav2 remains the motion planner."""

from dataclasses import dataclass
import math
import time
from .field_tool_recovery import DEFAULT_MAX_RECOVERIES, recovery_limit
from .field_tool_nav_heartbeat import NavigationHeartbeat

# The operator's common acceptance policy overrides legacy point metadata.
POSITION_TOLERANCE_M = 0.02
YAW_TOLERANCE_DEG = 10.0
YAW_TOLERANCE_RAD = math.radians(YAW_TOLERANCE_DEG)


def arrival_errors(pose, target):
    position = math.hypot(pose['x'] - target.x, pose['y'] - target.y)
    yaw = abs(math.remainder(pose['yaw_rad'] - target.yaw, 2.0 * math.pi))
    if not all(math.isfinite(v) for v in (position, yaw)):
        raise ValueError('arrival pose is not finite')
    return position, math.degrees(yaw)


def repeat_target_order(points, first, second, rounds, current_pose):
    """Build a transient round trip from the endpoint occupied by the robot."""
    if first == second or first not in points or second not in points:
        raise ValueError('请选择两个不同的已保存点位')
    if isinstance(rounds, bool) or not isinstance(rounds, int) or not 1 <= rounds <= 20:
        raise ValueError('往返轮数必须为1到20')
    candidates = []
    for name in (first, second):
        p = points[name]['approach']
        target = NavigationTarget(name, name, p['x'], p['y'], p['yaw_rad'],
                                  POSITION_TOLERANCE_M, YAW_TOLERANCE_DEG)
        target.validate()
        distance, yaw = arrival_errors(current_pose, target)
        if distance <= POSITION_TOLERANCE_M and yaw <= YAW_TOLERANCE_DEG:
            candidates.append((distance, name))
    if not candidates:
        raise ValueError('请先停在其中一个端点并保持该点朝向，再开始重复测试')
    source = min(candidates)[1]
    other = second if source == first else first
    return [other, source] * rounds


@dataclass(frozen=True)
class NavigationTarget:
    name: str
    label: str
    x: float
    y: float
    yaw: float
    position_tolerance_m: float
    yaw_tolerance_deg: float
    timeout_s: float = 120.0

    def validate(self):
        if not self.name or not all(math.isfinite(float(v)) for v in (self.x, self.y, self.yaw)):
            raise ValueError('navigation target pose is invalid')
        if not 0.01 <= self.position_tolerance_m <= 0.20:
            raise ValueError('navigation position tolerance is invalid')
        if not 1.0 <= self.yaw_tolerance_deg <= 30.0:
            raise ValueError('navigation yaw tolerance is invalid')
        if not 5.0 <= self.timeout_s <= 600.0:
            raise ValueError('navigation timeout is invalid')


class NavigationController:
    ACTION_NAME = 'NavigateToPose'
    IDLE = 'idle'
    WAITING_ACCEPTANCE = 'waiting_acceptance'
    ACTIVE = 'active'
    VERIFYING_STOP = 'verifying_stop'
    ZERO_HOLD = 'zero_hold'
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'
    STOPPED = 'stopped'
    TERMINAL = {IDLE, SUCCEEDED, FAILED, STOPPED}

    def __init__(self, node):
        self.node = node
        self.heartbeat = NavigationHeartbeat(node)
        self.state = self.IDLE
        self.targets = []
        self.index = 0
        self.mission_id = self.goal_id = self.last_error = ''
        self.last_failure_kind = ''
        self.send_future = self.result_future = self.goal_handle = None
        self.parameter_future = None
        self.cancel_requested = False
        self.started_at = self.deadline = self.hold_until = 0.0
        self.feedback = {'distance_remaining_m': None, 'recoveries': 0}
        self.arrivals = []
        self.last_nav_status = None
        self.rejection_retries = 0
        self.arrival_retries = 0
        self.arrival_retry_limit = 2  # Both controller modes can correct small stopped drift.
        self.max_recoveries = self.active_max_recoveries = DEFAULT_MAX_RECOVERIES
        self.dispatch_receipts = []

    @property
    def cleanup_pending(self):
        return self.send_future is not None or self.parameter_future is not None

    def start(self, targets, mission_id, *, resume_index=0):
        self.heartbeat.touch_ui()
        if self.state not in self.TERMINAL:
            return False
        self._drain_stopped_goal()
        if self.cleanup_pending:
            self.last_error = 'waiting for previous goal cancellation'
            return False
        if not targets or not mission_id:
            raise ValueError('navigation queue or mission is empty')
        for target in targets:
            target.validate()
        if not isinstance(resume_index, int) or not 0 <= resume_index < len(targets):
            raise ValueError('navigation resume index is invalid')
        self.active_max_recoveries = recovery_limit(self.max_recoveries)
        self.targets, self.index = list(targets), resume_index
        self.mission_id, self.goal_id = mission_id, ''
        self.last_error = ''
        self.last_failure_kind = ''
        self.arrivals = []
        self.last_nav_status = None
        self.rejection_retries = self.arrival_retries = 0
        self.dispatch_receipts = []
        self.deadline = time.monotonic() + 15.0
        self.state = self.WAITING_ACCEPTANCE
        self.node.set_motion_mission(mission_id)
        return True

    def current_target(self):
        return self.targets[self.index] if self.index < len(self.targets) else None

    def poll(self):
        self.heartbeat.touch_ui()
        try:
            if getattr(self.node, 'external_stop_requested', False) is True:
                self.node.external_stop_requested = False
                if self.state not in self.TERMINAL:
                    self.stop()
            if self.state in self.TERMINAL:
                self._drain_stopped_goal()
                return self.state
            fault = self.heartbeat.fault_for(self.goal_id)
            if fault:
                self._fail(fault[1], kind=fault[0])
                return self.state
            now = time.monotonic()
            # Once dispatched, Nav2's own TF/costmap checks govern velocity.
            # Keep the same action through delays in this GUI's sensor cache.
            executing = self.send_future is not None and self.state in {
                self.WAITING_ACCEPTANCE, self.ACTIVE}
            reason = ''
            if not executing:
                check = getattr(self.node, 'navigation_input_error', None)
                reason = check() if check else None
                if not isinstance(reason, str):
                    reason = '' if self.node.fixed_tf_ok(.5) and self.node.sensor_ready(.5) else 'fixed-map TF or scan is stale or unavailable'
            if reason:
                if (self.state == self.VERIFYING_STOP and now <= self.deadline
                        or self.state == self.ZERO_HOLD and now <= self.hold_until + 5.):
                    self.last_error = '等待停车后的定位复核：' + reason
                    return self.state
                # Before dispatch there is no moving goal to abort. Keep the
                # same target pending until the existing bounded deadline.
                if self.send_future is None and self.state == self.WAITING_ACCEPTANCE and now <= self.deadline:
                    self.last_error = reason
                    self.node.publish_lease(False)
                    return self.state
                record = getattr(self.node, 'record_navigation_fault', None)
                if record:
                    record(reason, self.current_target().name if self.current_target() else '')
                self._fail(reason, kind='sensor_unavailable')
                return self.state
            self.last_error = ''
            if self.state == self.ZERO_HOLD:
                if now >= self.hold_until and self.node.zero_velocity_hold_ok():
                    self.state = self.WAITING_ACCEPTANCE
                    self.deadline = now + 5.0
                elif now > self.hold_until + 5.0:
                    self._fail('zero-velocity hold timed out between points')
                return self.state

            if self.state == self.VERIFYING_STOP:
                self._verify_stopped_arrival(now)
                return self.state

            target = self.current_target()
            if target is None:
                self._finish_success()
                return self.state
            if self.send_future is None:
                self._prepare_goal(target, now)
            if self.send_future is not None:
                self._poll_active(target, now)
            if self.state == self.WAITING_ACCEPTANCE and self.send_future is not None:
                self.heartbeat.authorize(self.goal_id, self.mission_id,
                                         accepted=False, deadline=self.deadline)
            if self.state == self.ACTIVE and self.goal_handle is not None:
                self.heartbeat.authorize(self.goal_id, self.mission_id,
                    accepted=True, deadline=self.started_at + target.timeout_s,
                    result_future=self.result_future)
            fault = self.heartbeat.fault_for(self.goal_id)
            if fault:
                self._fail(fault[1], kind=fault[0])
        except Exception as exc:
            self._fail(f'navigation request failed: {exc}')
        return self.state

    def _prepare_goal(self, target, now):
        if now > self.deadline:
            self._fail('controller parameter service timed out')
            return
        if not self.node.navigation_ready():
            return
        if self.parameter_future is None:
            self.parameter_future = self.node.set_controller_tolerances(
                target.position_tolerance_m, target.yaw_tolerance_deg)
        if self.parameter_future is None or not self.parameter_future.done():
            return
        response = self.parameter_future.result()
        self.parameter_future = None
        if response is None or len(response.results) != 4 or not all(r.successful for r in response.results):
            self._fail('controller rejected point tolerances')
            return
        self.goal_id = f'{self.mission_id}:{self.index}:{time.monotonic_ns()}'
        self.feedback = {'distance_remaining_m': None, 'recoveries': 0}
        identity = self.goal_id
        self.deadline = now + 5.0
        if not self.heartbeat.authorize(identity, self.mission_id,
                                         accepted=False, deadline=self.deadline):
            return
        self.send_future = self.node.navigate_client.send_goal_async(
            self.node.make_navigation_goal(target),
            feedback_callback=lambda msg: self._on_feedback(msg, identity))
        self.deadline = now + 5.0

    def _poll_active(self, target, now):
        if not self.send_future.done():
            if now > self.deadline:
                self._fail(f'{self.ACTION_NAME} goal acceptance timed out')
            return
        if self.result_future is None:
            self.goal_handle = self.send_future.result()
            if self.goal_handle is None or not self.goal_handle.accepted:
                self.dispatch_receipts.append({'goal_id': self.goal_id, 'accepted': False,
                                               'target': target.name, 'at': now})
                self._revoke()
                self._clear_goal()
                if self.rejection_retries == 0:
                    self.rejection_retries += 1
                    self.node.reset_navigation_readiness()
                    self.state, self.hold_until = self.ZERO_HOLD, now + 1.0
                else:
                    self._fail(f'{self.ACTION_NAME} goal was rejected twice')
                return
            self.dispatch_receipts.append({'goal_id': self.goal_id, 'accepted': True,
                                           'target': target.name, 'at': now})
            self.result_future = self.goal_handle.get_result_async()
            self.started_at, self.state = now, self.ACTIVE
            return
        if not self.result_future.done():
            if now - self.started_at > target.timeout_s:
                self._fail(f'{self.ACTION_NAME} goal timed out')
            elif self.feedback['recoveries'] > self.active_max_recoveries:
                self._fail(f"导航恢复次数已超过上限：{self.feedback['recoveries']} > {self.active_max_recoveries} 次，已停止")
            return
        status = int(self.result_future.result().status)
        self.last_nav_status = status
        self._revoke()
        self._clear_goal()
        if status != 4:  # action_msgs/GoalStatus.STATUS_SUCCEEDED
            self._fail(f'{self.ACTION_NAME} ended with status {status}')
            return
        # The action may finish before the smoother/guard has stopped. Verify
        # a fresh map pose after the original zero hold before advancing.
        self.state = self.VERIFYING_STOP
        self.deadline = now + 5.0

    def _verify_stopped_arrival(self, now):
        if now > self.deadline:
            self._fail('arrival zero-velocity hold timed out')
            return
        if not self.node.zero_velocity_hold_ok():
            return
        target = self.current_target()
        try:
            pose = self.node.capture_current_pose()
        except Exception as exc:
            # A fresh check can race the next TF sample; keep the successful
            # action's bounded verification window instead of issuing E-stop.
            self.last_error = '等待停车后的定位复核：' + str(exc)
            return
        position, yaw_deg = arrival_errors(pose, target)
        accepted = (position <= target.position_tolerance_m
                    and yaw_deg <= target.yaw_tolerance_deg)
        report = {'name': target.name, 'nav_status': self.last_nav_status,
                  'mission_id': self.mission_id, 'goal_id': self.goal_id,
                  'goal_index': self.index, 'recorded_at_unix': time.time(),
                  'elapsed_s': now - self.started_at,
                  'position_error_m': position, 'yaw_error_deg': yaw_deg,
                  'position_tolerance_m': target.position_tolerance_m,
                  'yaw_tolerance_deg': target.yaw_tolerance_deg,
                  'pose': pose, 'stopped': True, 'accepted': accepted}
        self.arrivals.append(report)
        self.node.record_navigation_arrival(report)
        if not accepted:
            if (self.arrival_retries < self.arrival_retry_limit and position <= .10):
                self.arrival_retries += 1
                self.state, self.hold_until = self.ZERO_HOLD, now + .5
                return  # Same coordinates, fresh Nav2 goal, no queue advance.
            self._fail(f'arrival outside tolerance after stop: {position:.6f} m, {yaw_deg:.3f} deg')
            return
        self.index += 1
        self.rejection_retries = self.arrival_retries = 0
        if self.index == len(self.targets):
            self._finish_success()
        else:
            self.state, self.hold_until = self.ZERO_HOLD, now + 0.5

    def _on_feedback(self, message, identity):
        if identity != self.goal_id or self.state in self.TERMINAL:
            return
        item = message.feedback
        self.feedback['distance_remaining_m'] = float(item.distance_remaining)
        self.feedback['recoveries'] = max(self.feedback['recoveries'], int(item.number_of_recoveries))

    def _clear_goal(self):
        self.send_future = self.result_future = self.goal_handle = None
        self.cancel_requested = False

    def _drain_stopped_goal(self):
        if self.parameter_future is not None and self.parameter_future.done():
            self.parameter_future = None
        if self.send_future is None or not self.send_future.done():
            return
        if self.goal_handle is None:
            self.goal_handle = self.send_future.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            self._clear_goal()
            return
        if self.result_future is None:
            self.result_future = self.goal_handle.get_result_async()
        if self.result_future.done():
            self._clear_goal()
            return
        if not self.cancel_requested:
            self.goal_handle.cancel_goal_async()
            self.cancel_requested = True

    def _revoke(self):
        self.heartbeat.revoke()

    def halt(self, reason='controlled stop'):
        self.state, self.last_error = self.STOPPED, reason
        self.last_failure_kind = ''
        self._revoke()
        try:
            # Keep a pending acceptance handle: cancel it if it arrives late.
            self._drain_stopped_goal()
        except Exception as exc:
            self.last_error = f'{reason}; cancellation pending: {exc}'

    def stop(self):
        self.halt('operator stop')
        self.node.publish_nav_stop()

    def pause_sensor_input(self):
        """Revoke motion and cancel this goal, without issuing an operator E-stop."""
        self.halt('waiting for sensor recovery')

    def _finish_success(self):
        self._revoke()
        self.state = self.SUCCEEDED

    def _fail(self, reason, *, kind=''):
        if kind in {'sensor_unavailable', 'heartbeat_unavailable'}:
            self.pause_sensor_input()
        else:
            self.halt(reason)
        self.state, self.last_error = self.FAILED, reason
        self.last_failure_kind = kind
