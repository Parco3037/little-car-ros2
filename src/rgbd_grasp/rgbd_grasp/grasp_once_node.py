#!/usr/bin/env python3
"""Fail-closed one-shot side grasp for a single blue 5 cm cube."""

from collections import deque
import json
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import Trigger

from .grasp_core import (
    Rejected,
    bounded_search_positions,
    compensate_grasp_id4,
    desired_sag_compensation,
    select_candidate_uvz,
    smooth_joint_waypoints,
    stable_target,
    validate_arm_pulses,
    validate_commanded_fk,
    validate_joint_step,
)
from .grasp_hardware import GraspHardware


from .grasp_clearance import raised_side_path, joint_ramp, validate_ramp_height, checked_height


class GraspOnceNode(Node):
    """Expose explicit precheck, grasp, stop and release services."""

    def __init__(self):
        super().__init__('grasp_once_node')
        self.callback_group = ReentrantCallbackGroup()
        self.cancel_event = threading.Event()
        self.execution_lock = threading.Lock()
        self.holding = False
        self.prepared = False
        self.auto_prepare_timer = None
        self.auto_prepare_thread = None
        self.targets = deque(maxlen=30)
        self.last_candidate_uvz = None
        self.centered_id1 = None
        self.observation_after_s = 0.0

        defaults = {
            'motion_enabled': False,
            'auto_prepare': False,
            'auto_grasp': False,
            'auto_prepare_delay_s': 3.0,
            'auto_target_wait_s': 8.0,
            'target_topic': '/rgbd_grasp/target_point_ik',
            'target_frame': 'arm_ik_world',
            'target_max_age_s': 3.0,
            'candidate_topic': '/rgbd_grasp/blocks_uvz',
            'candidate_max_age_s': 3.0,
            'visual_center_enabled': True,
            # Aurora930 publishes a 640 px-wide RGB image. Match the vendor
            # tracker and align to the optical image midpoint, not an
            # empirical offset that made a front-centred cube rotate ID1.
            'image_center_u': 326.0,
            'center_tolerance_px': 6.0,
            'center_pixels_per_pulse': 1.7,
            'center_max_step_pulse': 20,
            'center_max_steps': 4,
            'center_observation_s': 15.0,
            'require_single_candidate': False,
            'target_selection': 'nearest',
            'stable_target_samples': 3,
            'stable_target_spread_m': 0.005,
            'workspace': [0.00, 0.35, -0.30, 0.20, 0.08, 0.55],
            'grasp_offset': [0.0, 0.0, 0.0],
            'grasp_height_offset_m': 0.015,
            'grasp_id4_offset_pulse': 60,
            'sag_compensation_m': 0.020,
            'pregrasp_clearance_m': 0.015,
            'approach_distance_m': 0.05,
            'lift_distance_m': 0.03,
            'cartesian_step_m': 0.005,
            'pitch_deg': 30.0,
            'pitch_range_deg': [-180.0, 180.0],
            'fixed_id5': 860,
            'id5_tolerance': 25,
            'id1_range': [350, 550],
            'initial_pose': [450, 800, 50, 250, 860, 390],
            'carry_pose': [450, 500, 250, 400, 860],
            'carry_pose_duration_s': 0.8,
            'initial_pose_duration_s': 2.0,
            'search_step': 20,
            'search_sweeps': 2,
            'search_observation_s': 4.0,
            'servo_limits_flat': [
                350, 550,
                125, 875,
                0, 1000,
                42, 958,
                0, 1000,
            ],
            'motion_duration_s': 0.45,
            'settle_timeout_s': 3.0,
            'initial_settle_tolerance': 30,
            'settle_tolerance': 30,
            'stable_joint_samples': 2,
            'service_timeout_s': 1.5,
            'max_first_joint_jump': 450,
            'max_path_joint_jump': 80,
            'max_carry_joint_jump': 700,
            'joint_ramp_step': 60,
            'gripper_open': 390,
            'gripper_close': 50,
        }
        self.parameters = {}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            self.parameters[name] = self.get_parameter(name).value

        flat_limits = self.parameters.pop('servo_limits_flat')
        if len(flat_limits) != 10:
            raise RuntimeError('servo_limits_flat must contain 10 values')
        self.parameters['servo_limits'] = [
            flat_limits[index:index + 2]
            for index in range(0, len(flat_limits), 2)
        ]

        self.status_pub = self.create_publisher(String, '~/status', 10)
        self.last_candidate_count = None
        self.last_candidate_stamp_s = 0.0
        self.target_sub = self.create_subscription(
            PointStamped,
            self.parameters['target_topic'],
            self.target_callback,
            10,
            callback_group=self.callback_group,
        )
        self.candidate_sub = self.create_subscription(
            Float32MultiArray,
            self.parameters['candidate_topic'],
            self.candidate_callback,
            10,
            callback_group=self.callback_group,
        )
        self.hardware = GraspHardware(
            self, lambda: self.cancel_event.is_set())

        self.create_service(
            Trigger,
            '~/precheck',
            self.precheck_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '~/prepare',
            self.prepare_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '~/grasp_once',
            self.grasp_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '~/stop',
            self.stop_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '~/release',
            self.release_callback,
            callback_group=self.callback_group,
        )

        if not self.parameters['motion_enabled']:
            self.publish_status('idle', 'startup: motion is disabled')
            self.get_logger().warning(
                'Motion is disabled. Only precheck can run.')
        elif self.parameters['auto_prepare']:
            delay = max(
                0.1, float(self.parameters['auto_prepare_delay_s']))
            self.auto_prepare_timer = self.create_timer(
                delay,
                self.auto_prepare_callback,
                callback_group=self.callback_group,
            )
            self.publish_status(
                'waiting_for_prepare',
                f'automatic initial pose scheduled in {delay:.1f}s',
            )
        else:
            self.publish_status(
                'idle', 'startup: waiting for explicit prepare')

    def auto_prepare_callback(self):
        """Run the launch-time prepare exactly once without blocking ROS."""
        timer = self.auto_prepare_timer
        if timer is None:
            return
        timer.cancel()
        self.auto_prepare_timer = None
        self.auto_prepare_thread = threading.Thread(
            target=self.auto_prepare_worker,
            name='rgbd_grasp_auto_prepare',
            daemon=True,
        )
        self.auto_prepare_thread.start()

    def auto_prepare_worker(self):
        if self.cancel_event.is_set():
            self.publish_status('cancelled', 'startup cancelled before prepare')
            return
        if self.holding:
            self.publish_status(
                'rejected', 'automatic prepare forbidden while holding')
            return
        if not self.execution_lock.acquire(blocking=False):
            self.publish_status(
                'rejected', 'automatic prepare skipped: operation active')
            return
        try:
            measured = self.move_to_initial_pose()
            self.publish_status(
                'prepared',
                'automatic initial pose complete',
                measured={str(key): int(value)
                          for key, value in measured.items()},
            )
            if self.parameters['auto_grasp']:
                self.publish_status(
                    'auto_grasp',
                    'waiting for one stable target after automatic prepare',
                )
                if not self.wait_for_target(
                        float(self.parameters['auto_target_wait_s'])):
                    self.acquire_target_with_search()
                self.center_target_with_id1()
                plan = self.run_precheck()
                self.execute(plan)
                self.publish_status(
                    'auto_grasp_complete',
                    'one-shot complete; staying at hold pose with gripper command preserved',
                    hold_pose_reached=True,
                    gripper_command_preserved=True,
                    physical_holding_confirmed=False,
                )
        except Rejected as error:
            self.prepared = False
            self.hardware.stop()
            stage = (
                'cancelled' if self.cancel_event.is_set() else 'rejected')
            self.publish_status(stage, str(error))
        except Exception as error:
            self.prepared = False
            self.hardware.stop()
            self.get_logger().error(
                f'unexpected automatic prepare failure: {error!r}')
            self.publish_status('failed', str(error))
        finally:
            self.execution_lock.release()

    def publish_status(self, stage, detail, **extra):
        payload = {'stage': stage, 'detail': detail, **extra}
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(message)
        self.get_logger().info(message.data)

    @staticmethod
    def finish(response, success, message):
        response.success = bool(success)
        response.message = str(message)
        return response

    def target_callback(self, message):
        stamp_s = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1e-9
        )
        if stamp_s <= self.observation_after_s:
            return
        self.targets.append({
            'stamp_s': stamp_s,
            'frame': message.header.frame_id,
            'point': [
                message.point.x,
                message.point.y,
                message.point.z,
            ],
        })

    def candidate_callback(self, message):
        self.last_candidate_count = 0
        self.last_candidate_uvz = None
        try:
            selected, count = select_candidate_uvz(
                message.data, self.parameters['target_selection'])
            self.last_candidate_uvz = selected
            self.last_candidate_count = count
        except Rejected:
            pass
        self.last_candidate_stamp_s = (
            self.get_clock().now().nanoseconds * 1e-9)

    def current_target(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        return stable_target(
            self.targets,
            now_s,
            self.parameters['target_frame'],
            int(self.parameters['stable_target_samples']),
            float(self.parameters['target_max_age_s']),
            float(self.parameters['stable_target_spread_m']),
            self.parameters['workspace'],
        )

    def clear_perception(self):
        self.observation_after_s = self.get_clock().now().nanoseconds * 1e-9
        self.targets.clear()
        self.last_candidate_count = None
        self.last_candidate_uvz = None
        self.last_candidate_stamp_s = 0.0
        self.publish_status('observing', 'waiting for fresh data after motion settled')

    def perception_ready(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        age = now_s - self.last_candidate_stamp_s
        if (not -0.05 <= age <= self.parameters['candidate_max_age_s']
                or not self.last_candidate_count or self.last_candidate_uvz is None):
            return False
        if self.parameters['require_single_candidate']:
            if self.last_candidate_count != 1:
                raise Rejected(
                    'expected exactly one blue cube candidate')
        try:
            self.current_target()
        except Rejected as error:
            if str(error) in (
                    'not enough target samples',
                    'target is stale',
                    'target is not stable'):
                return False
            raise
        return True

    def wait_for_target(self, timeout_s=None):
        if timeout_s is None:
            timeout_s = float(self.parameters['search_observation_s'])
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            self.hardware.guard()
            if self.perception_ready():
                return True
            time.sleep(0.05)
        return False

    def current_candidate_u(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        age = now_s - self.last_candidate_stamp_s
        if age > float(self.parameters['candidate_max_age_s']):
            raise Rejected('candidate pixels are stale')
        if not self.last_candidate_count or self.last_candidate_uvz is None:
            raise Rejected('no fresh selected blue cube candidate')
        if self.parameters['require_single_candidate'] and self.last_candidate_count != 1:
            raise Rejected('expected exactly one fresh blue cube candidate')
        return float(self.last_candidate_uvz[0])

    def center_target_with_id1(self):
        self.centered_id1 = None
        if not self.parameters['visual_center_enabled']:
            measured = self.hardware.read((1, 5))
            self.hardware.assert_fixed_id5(measured)
            self.centered_id1 = int(measured[1])
            return self.centered_id1
        center_u = float(self.parameters['image_center_u'])
        tolerance = float(self.parameters['center_tolerance_px'])
        pixels_per_pulse = float(self.parameters['center_pixels_per_pulse'])
        max_step = int(self.parameters['center_max_step_pulse'])
        max_steps = int(self.parameters['center_max_steps'])
        if tolerance <= 0.0 or pixels_per_pulse <= 0.0:
            raise Rejected('invalid visual centering calibration')
        lower, upper = map(int, self.parameters['id1_range'])
        for step_index in range(max_steps + 1):
            self.hardware.guard()
            u = self.current_candidate_u()
            error_px = u - center_u
            measured = self.hardware.read((1, 5))
            self.hardware.assert_fixed_id5(measured)
            current = int(measured[1])
            if abs(error_px) <= tolerance:
                self.centered_id1 = current
                self.publish_status(
                    'centered',
                    'blue cube aligned with the gripper centerline',
                    u=round(u, 1),
                    error_px=round(error_px, 1),
                    id1=current,
                    candidate_count=self.last_candidate_count,
                    selected_v=float(self.last_candidate_uvz[1]),
                    target_selection=self.parameters['target_selection'],
                )
                return current
            if step_index >= max_steps:
                raise Rejected(
                    f'visual centering did not converge: error={error_px:.1f}px')
            delta = int(round(-error_px / pixels_per_pulse))
            delta = max(-max_step, min(max_step, delta))
            if delta == 0:
                delta = -1 if error_px > 0.0 else 1
            goal = max(lower, min(upper, current + delta))
            if goal == current:
                raise Rejected('visual centering reached the ID1 boundary')
            self.publish_status(
                'centering',
                'moving ID1 toward the calibrated image center',
                u=round(u, 1),
                error_px=round(error_px, 1),
                id1_start=current,
                id1_goal=goal,
            )
            self.hardware.command({1: goal}, duration=max(0.25, abs(goal-current)/60.0))
            self.hardware.wait((1,), [goal])
            self.clear_perception()
            if not self.wait_for_target(
                    float(self.parameters['center_observation_s'])):
                raise Rejected('blue cube lost during visual centering')

    def move_to_initial_pose(self):
        self.prepared = False
        self.centered_id1 = None
        self.hardware.guard()
        self.assert_single_command_source()
        pose = [int(value) for value in self.parameters['initial_pose']]
        if len(pose) != 6 or any(value < 0 or value > 1000
                                 for value in pose):
            raise Rejected('invalid initial pose')
        lower, upper = map(int, self.parameters['id1_range'])
        if not lower <= pose[0] <= upper:
            raise Rejected('initial ID1 is outside bounded range')
        if pose[4] != int(self.parameters['fixed_id5']):
            raise Rejected('initial ID5 must match fixed_id5')
        if pose[5] != int(self.parameters['gripper_open']):
            raise Rejected('initial ID10 must match gripper_open')
        measured = self.hardware.read()
        self.hardware.assert_fixed_id5(measured)
        if not lower <= measured[1] <= upper:
            raise Rejected('actual ID1 is outside bounded range')
        self.publish_status(
            'prepare',
            'moving to configured observation pose',
            start={str(key): int(value)
                   for key, value in measured.items()},
            goal=pose,
        )
        ids = (1, 2, 3, 4, 10)
        goals = pose[:4] + [pose[5]]
        self.hardware.command(
            dict(zip(ids, goals)),
            duration=float(self.parameters['initial_pose_duration_s']),
        )
        measured = self.hardware.wait(
            ids,
            goals,
            tolerance=float(
                self.parameters['initial_settle_tolerance']),
        )
        self.hardware.assert_fixed_id5(measured)
        self.prepared = True
        self.clear_perception()
        self.publish_status(
            'observing',
            'initial pose reached; waiting for fresh camera data',
            measured={str(key): int(value)
                      for key, value in measured.items()},
        )
        return measured

    def prepared_pose_is_current(self):
        """Avoid re-commanding the observation pose on every grasp call."""
        if not self.prepared:
            return False
        measured = self.hardware.read()
        pose = [int(value) for value in self.parameters['initial_pose']]
        ids = (1, 2, 3, 4, 5, 10)
        tolerance = int(self.parameters['initial_settle_tolerance'])
        if any(abs(measured[servo_id] - goal) > tolerance
               for servo_id, goal in zip(ids, pose)):
            self.prepared = False
            return False
        self.hardware.assert_fixed_id5(measured)
        return True

    def acquire_target_with_search(self):
        if self.wait_for_target():
            return
        measured = self.hardware.read()
        self.hardware.assert_fixed_id5(measured)
        start = int(measured[1])
        positions = bounded_search_positions(
            start,
            self.parameters['id1_range'],
            self.parameters['search_step'],
            self.parameters['search_sweeps'],
        )
        for position in positions:
            self.publish_status(
                'search', 'moving ID1 only', id1=int(position))
            self.hardware.command({1: int(position)})
            self.hardware.wait((1,), [int(position)])
            self.clear_perception()
            if self.wait_for_target():
                return
        self.publish_status(
            'search', 'target not found; restoring ID1 start', id1=start)
        self.hardware.command({1: start})
        self.hardware.wait((1,), [start])
        self.clear_perception()
        raise Rejected('blue cube not found in bounded ID1 search')

    def assert_single_command_source(self):
        for topic in ('/servo_controller', '/ros_robot_controller/bus_servo/set_state'):
            infos = self.get_publishers_info_by_topic(topic)
            own = [i for i in infos if i.node_name == self.get_name()
                   and i.node_namespace == self.get_namespace()]
            foreign = [i.node_namespace.rstrip('/') + '/' + i.node_name
                       for i in infos if i not in own]
            if foreign or len(own) > 1:
                raise Rejected('another servo command publisher is active on '
                               + topic + ': ' + ', '.join(foreign or ['duplicate grasp node']))

    def plan(self):
        self.hardware.guard()
        self.assert_single_command_source()
        self.current_candidate_u()
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if self.parameters['require_single_candidate']:
            if (now_s - self.last_candidate_stamp_s
                    > self.parameters['candidate_max_age_s']):
                raise Rejected('candidate count is stale')
            if self.last_candidate_count != 1:
                raise Rejected(
                    'expected exactly one blue cube candidate')

        target = self.current_target()
        measured = self.hardware.read()
        self.hardware.assert_fixed_id5(measured)
        locked_id1 = None
        if self.parameters['visual_center_enabled']:
            if self.centered_id1 is None:
                raise Rejected('visual centering must run before planning')
            locked_id1 = int(self.centered_id1)
            if abs(int(measured[1]) - locked_id1) > int(
                    self.parameters['settle_tolerance']):
                raise Rejected('ID1 moved after visual centering')

        height_offset = float(self.parameters['grasp_height_offset_m'])
        if not np.isfinite(height_offset) or not 0.0 <= height_offset <= 0.020:
            raise Rejected('grasp_height_offset_m must be between 0 and 0.020 m')
        grasp_offset = np.asarray(self.parameters['grasp_offset'], dtype=float).copy()
        if grasp_offset.shape != (3,):
            raise Rejected('invalid grasp_offset')
        grasp_offset[2] += height_offset
        base_sag = float(self.parameters['sag_compensation_m'])
        radius = float(np.linalg.norm((target + grasp_offset)[:2]))
        requested_sag = desired_sag_compensation(base_sag, radius)
        # The requested allowance may exceed the vendor IK envelope at full
        # extension. Find the greatest feasible allowance before planning.
        probe = np.asarray(target + grasp_offset, dtype=float)
        sag = requested_sag
        try:
            self.hardware.ik(probe + [0.0, 0.0, sag])
        except Rejected:
            self.hardware.ik(probe)  # No safe path if even zero allowance fails.
            low, high = 0.0, sag
            for _ in range(7):
                middle = (low + high) / 2.0
                try:
                    self.hardware.ik(probe + [0.0, 0.0, middle])
                    low = middle
                except Rejected:
                    high = middle
            sag = max(0.0, low - 0.001)
        clearance = float(self.parameters['pregrasp_clearance_m'])
        pregrasp, approach, compensated_target = raised_side_path(
            target, grasp_offset, float(self.parameters['approach_distance_m']),
            float(self.parameters['cartesian_step_m']), sag, clearance,
        )

        cartesian = [pregrasp] + list(approach)

        pulses = []
        raw_ik_id1 = []
        nominal_pulses = []
        commanded_fk = []
        id4_offset = self.parameters['grasp_id4_offset_pulse']

        for index, point in enumerate(cartesian):
            try:
                nominal = self.hardware.ik(point)  # retains the 5mm nominal FK check
                nominal_pulses.append(nominal)
                pulse = compensate_grasp_id4(
                    nominal, id4_offset, self.parameters['servo_limits'][3])
                commanded_fk.append(validate_commanded_fk(
                    self.hardware.fk(pulse), self.parameters['workspace']))
                raw_ik_id1.append(int(nominal[0]))
                pulses.append(pulse)

            except Rejected as error:
                point_text = [
                    round(float(value), 4)
                    for value in point
                ]

                raise Rejected(
                    f'IK failed at path point {index}/{len(cartesian) - 1}, '
                    f'XYZ={point_text}: {error}'
                )

        pulses, smoothed_waypoints = smooth_joint_waypoints(
            pulses, commanded_fk, self.hardware.fk, self.parameters['workspace'])

        current = np.asarray(
            [measured[servo_id] for servo_id in (1, 2, 3, 4, 5)],
            dtype=float,
        )
        validate_joint_step(
            current,
            pulses[0],
            float(self.parameters['max_first_joint_jump']),
        )
        ramp_min_z = validate_ramp_height(
            current, pulses[0], self.hardware.fk, self.parameters['workspace'],
            self.parameters['fixed_id5'])
        previous = pulses[0]
        for solution in pulses[1:]:
            validate_joint_step(
                previous,
                solution,
                float(self.parameters['max_path_joint_jump']),
            )
            previous = solution

        carry = validate_arm_pulses(
            self.parameters['carry_pose'],
            self.parameters['servo_limits'],
            self.parameters['id1_range'],
            self.parameters['fixed_id5'],
        )
        validate_joint_step(
            previous,
            carry,
            float(self.parameters['max_carry_joint_jump']),
        )

        return {
            'target': target,
            'grasp_target': compensated_target,
            'sag_compensation_m': sag,
            'requested_sag_compensation_m': requested_sag,
            'sag_reach_limited': sag < requested_sag - 1e-6,
            'pregrasp_clearance_m': clearance,
            'ramp_min_model_z_m': ramp_min_z,
            'grasp_height_offset_m': height_offset,
            'grasp_id4_offset_pulse': int(id4_offset),
            'nominal_final_pulses': [int(v) for v in nominal_pulses[-1]],
            'commanded_final_pulses': [int(v) for v in pulses[-1]],
            'commanded_fk_xyz': [float(v) for v in commanded_fk[-1]],
            'candidate_count': self.last_candidate_count,
            'measured': measured,
            'pregrasp': pulses[0],
            'approach': pulses[1:1 + len(approach)],
            'carry': carry,
            'point_count': len(cartesian) + 1,
            'smoothed_waypoints': smoothed_waypoints,
            'locked_id1': locked_id1,
            'raw_ik_id1_range': [min(raw_ik_id1), max(raw_ik_id1)],
        }

    def run_precheck(self):
        self.publish_status('precheck', 'reading target and hardware')
        plan = self.plan()
        target = [round(float(value), 4) for value in plan['target']]
        measured = {
            str(key): int(value)
            for key, value in plan['measured'].items()
        }
        self.publish_status(
            'ready',
            'all IK points and safety bounds passed',
            target=target,
            grasp_target=[round(float(v), 4) for v in plan['grasp_target']],
            grasp_height_offset_m=plan['grasp_height_offset_m'],
            sag_compensation_m=plan['sag_compensation_m'],
            requested_sag_compensation_m=plan['requested_sag_compensation_m'],
            sag_reach_limited=plan['sag_reach_limited'],
            pregrasp_clearance_m=plan['pregrasp_clearance_m'],
            ramp_min_model_z_m=plan['ramp_min_model_z_m'],
            grasp_id4_offset_pulse=plan['grasp_id4_offset_pulse'],
            nominal_final_pulses=plan['nominal_final_pulses'],
            commanded_final_pulses=plan['commanded_final_pulses'],
            commanded_fk_xyz=[round(v, 4) for v in plan['commanded_fk_xyz']],
            candidate_count=plan['candidate_count'],
            target_selection=self.parameters['target_selection'],
            measured=measured,
            point_count=plan['point_count'],
            smoothed_waypoints=plan['smoothed_waypoints'],
            locked_id1=plan['locked_id1'],
            raw_ik_id1_range=plan['raw_ik_id1_range'],
        )
        return plan

    def precheck_callback(self, _request, response):
        if not self.execution_lock.acquire(blocking=False):
            return self.finish(response, False, 'another operation is active')
        self.cancel_event.clear()
        try:
            plan = self.run_precheck()
            return self.finish(
                response,
                True,
                'precheck passed for '
                + str(plan['point_count'])
                + ' path points',
            )
        except Rejected as error:
            self.publish_status('rejected', str(error))
            return self.finish(response, False, str(error))
        except Exception as error:
            self.get_logger().error(f'unexpected precheck failure: {error!r}')
            self.publish_status('failed', str(error))
            return self.finish(response, False, str(error))
        finally:
            self.execution_lock.release()

    def prepare_callback(self, _request, response):
        if not self.parameters['motion_enabled']:
            return self.finish(response, False, 'motion disabled')
        if self.holding:
            return self.finish(
                response, False, 'already holding; prepare is forbidden')
        if not self.execution_lock.acquire(blocking=False):
            return self.finish(response, False, 'another operation is active')
        self.cancel_event.clear()
        try:
            measured = self.move_to_initial_pose()
            return self.finish(
                response,
                True,
                'initial pose reached: '
                + str({key: int(value)
                       for key, value in measured.items()}),
            )
        except Rejected as error:
            self.hardware.stop()
            stage = (
                'cancelled' if self.cancel_event.is_set() else 'rejected')
            self.publish_status(stage, str(error))
            return self.finish(response, False, str(error))
        except Exception as error:
            self.hardware.stop()
            self.get_logger().error(f'unexpected prepare failure: {error!r}')
            self.publish_status('failed', str(error))
            return self.finish(response, False, str(error))
        finally:
            self.execution_lock.release()

    def command_and_wait_arm(self, pulses, stage):
        self.publish_status(
            stage,
            'moving arm',
            id1=int(pulses[0]),
        )
        self.hardware.command_arm(pulses)
        measured = self.hardware.wait(
            (1, 2, 3, 4),
            [int(value) for value in pulses[:4]],
        )
        actual = [int(measured[sid]) for sid in (1, 2, 3, 4, 5)]
        commanded_xyz = self.hardware.fk(pulses)
        measured_xyz = self.hardware.fk(actual)
        # Always log the real feedback, including the sample that causes a stop.
        self.publish_status(stage, 'physical joint and height readback',
            commanded=[int(v) for v in pulses], measured={str(k):int(v) for k,v in measured.items()},
            commanded_fk_xyz=[float(v) for v in commanded_xyz],
            measured_fk_xyz=[float(v) for v in measured_xyz],
            model_height_drop_m=float(commanded_xyz[2]-measured_xyz[2]),
            sag_compensation_m=float(self.active_sag_compensation_m))

    def stream_arm_path(self, points, stage, overlap):
        """Send validated waypoints without settling at every intermediate point."""
        if not points:
            raise Rejected('empty arm path')
        interval = float(self.parameters['motion_duration_s']) * overlap
        for index, pulses in enumerate(points[:-1]):
            # At long reach, periodically let the loaded joints catch up.
            if stage == 'approach' and index >= len(points) // 2 and index % 2 == 0:
                self.command_and_wait_arm(pulses, stage)
                continue
            self.publish_status(stage, 'moving arm', id1=int(pulses[0]))
            self.hardware.command_arm(pulses)
            deadline = time.monotonic() + interval
            while time.monotonic() < deadline:
                self.hardware.guard()
                time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))
        self.command_and_wait_arm(points[-1], stage)

    def ramp_to_pregrasp(self, measured, destination):
        start = [measured[sid] for sid in (1, 2, 3, 4, 5)]
        points = joint_ramp(start, destination, float(self.parameters['joint_ramp_step']),
                            self.parameters['fixed_id5'])
        self.stream_arm_path(points, 'move_to_pregrasp', overlap=1.0)

    def execute(self, plan):
        self.active_sag_compensation_m = plan['sag_compensation_m']
        self.publish_status('open_gripper', f"command ID10={self.parameters['gripper_open']}")
        self.hardware.command({10: int(self.parameters['gripper_open'])})
        self.hardware.wait(
            (10,),
            [int(self.parameters['gripper_open'])],
        )

        self.ramp_to_pregrasp(
            plan['measured'],
            plan['pregrasp'],
        )
        self.stream_arm_path(plan['approach'], 'approach', overlap=0.4 / 0.45)

        self.publish_status('close_gripper', f"command ID10={self.parameters['gripper_close']}")
        self.hardware.command({10: int(self.parameters['gripper_close'])})
        grip = self.hardware.wait((10,), goals=None)
        if grip[10] <= int(self.parameters['gripper_close']) + 10:
            raise Rejected('gripper reached empty-close position; grip not confirmed')

        # Keep the grasp latched even if the hold-pose move fails or is stopped.
        # A subsequent prepare/grasp request must not open the occupied claw.
        self.holding = True
        carry = plan['carry']
        self.publish_status(
            'move_to_hold_pose',
            'moving to configured post-grasp hold pose',
            goal=[int(value) for value in carry],
            gripper_command_preserved=True,
        )
        self.hardware.command(
            {servo_id: int(value) for servo_id, value in
             zip((1, 2, 3, 4, 5), carry)},
            duration=float(self.parameters['carry_pose_duration_s']),
            post_grasp_hold=True,
        )
        held_pose = self.hardware.wait(
            (1, 2, 3, 4, 5),
            [int(value) for value in carry],
        )

        self.publish_status(
            'holding_unconfirmed',
            'configured hold pose reached; staying here with gripper command preserved',
            hold_pose_reached=True,
            goal=[int(value) for value in carry],
            measured={str(sid): int(held_pose[sid]) for sid in (1, 2, 3, 4, 5)},
            gripper_command_preserved=True,
            gripper_readback=grip[10], physical_holding_confirmed=False,
        )

    def grasp_callback(self, _request, response):
        if not self.parameters['motion_enabled']:
            return self.finish(
                response,
                False,
                'motion disabled; relaunch with motion_enabled:=true',
            )
        if self.holding:
            return self.finish(
                response,
                False,
                'already holding; release explicitly before another grasp',
            )
        if not self.execution_lock.acquire(blocking=False):
            return self.finish(response, False, 'another operation is active')
        self.cancel_event.clear()
        try:
            if not self.prepared_pose_is_current():
                self.move_to_initial_pose()
            else:
                self.clear_perception()
                self.publish_status(
                    'observing',
                    'initial pose already reached; waiting for fresh camera data',
                )
            self.acquire_target_with_search()
            self.center_target_with_id1()
            plan = self.run_precheck()
            self.execute(plan)
            return self.finish(
                response,
                True,
                'grasp motion complete; physical holding unconfirmed',
            )
        except Rejected as error:
            self.hardware.stop()
            stage = (
                'cancelled'
                if self.cancel_event.is_set()
                else 'rejected'
            )
            self.publish_status(stage, str(error))
            return self.finish(response, False, str(error))
        except Exception as error:
            self.hardware.stop()
            self.get_logger().error(f'unexpected grasp failure: {error!r}')
            self.publish_status('failed', str(error))
            return self.finish(response, False, str(error))
        finally:
            self.execution_lock.release()

    def stop_callback(self, _request, response):
        self.cancel_event.set()
        if self.auto_prepare_timer is not None:
            self.auto_prepare_timer.cancel()
            self.auto_prepare_timer = None
        self.hardware.stop()
        self.publish_status(
            'stopped',
            'future stages cancelled; gripper was not released',
        )
        return self.finish(
            response,
            True,
            'stop sent; gripper state preserved',
        )

    def release_callback(self, _request, response):
        if not self.parameters['motion_enabled']:
            return self.finish(response, False, 'motion disabled')
        if not self.execution_lock.acquire(blocking=False):
            return self.finish(response, False, 'another operation is active')
        self.cancel_event.clear()
        try:
            self.publish_status(
                'release',
                'opening ID10 only; arm will not move',
            )
            self.hardware.command({
                10: int(self.parameters['gripper_open']),
            })
            self.hardware.wait(
                (10,),
                [int(self.parameters['gripper_open'])],
            )
            self.holding = False
            self.publish_status('idle', 'release complete')
            return self.finish(response, True, 'gripper opened')
        except Rejected as error:
            self.hardware.stop()
            self.publish_status('rejected', str(error))
            return self.finish(response, False, str(error))
        finally:
            self.execution_lock.release()


def main(args=None):
    rclpy.init(args=args)
    node = GraspOnceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_event.set()
        if node.auto_prepare_timer is not None:
            node.auto_prepare_timer.cancel()
        if rclpy.ok():
            node.hardware.stop()
        if node.auto_prepare_thread is not None:
            node.auto_prepare_thread.join(timeout=3.0)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
