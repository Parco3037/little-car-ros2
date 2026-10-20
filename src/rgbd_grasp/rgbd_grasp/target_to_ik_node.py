#!/usr/bin/env python3
"""Transform an RGB-D target into the vendor arm IK coordinates."""

import math
import time
import json

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped, Pose
from kinematics_msgs.srv import SetJointValue
from ros_robot_controller_msgs.msg import GetBusServoCmd
from ros_robot_controller_msgs.srv import GetBusServoState
from rclpy.node import Node
from std_msgs.msg import String


def pose_matrix(pose: Pose) -> np.ndarray:
    """Convert a ROS pose to a homogeneous transform."""
    quaternion = np.asarray([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ], dtype=float)
    if not np.all(np.isfinite(quaternion)):
        raise ValueError('non-finite endpoint quaternion')
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-9:
        raise ValueError('zero endpoint quaternion')
    x, y, z, w = quaternion / norm
    rotation = np.asarray([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=float)
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = rotation
    transform[:3, 3] = [
        pose.position.x,
        pose.position.y,
        pose.position.z,
    ]
    if not np.all(np.isfinite(transform)):
        raise ValueError('non-finite endpoint pose')
    return transform


class TargetToIkNode(Node):
    """Combine vendor FK, hand-eye transform, and the camera target."""

    def __init__(self):
        super().__init__('target_to_ik_node')
        defaults = {
            'input_topic': '/rgbd_grasp/target_point',
            'output_topic': '/rgbd_grasp/target_point_ik',
            'target_frame': 'arm_ik_world',
            'pose_service': '/kinematics/set_joint_value_target',
            # Camera bracket is before the wrist roll. This is a mathematical
            # reference only; this node never publishes an actuator command.
            'camera_roll_reference': 500,
            'observation_max_age_s': 0.75,
            'surface_to_center_m': 0.025,
            'reject_uninitialized_pose': True,
            'end_to_camera_flat': [
                0.0, 0.0, 1.0, -0.101,
                -1.0, 0.0, 0.0, 0.0,
                0.0, -1.0, 0.0, 0.037,
                0.0, 0.0, 0.0, 1.0,
            ],
        }
        self.parameters = {}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
            self.parameters[name] = self.get_parameter(name).value

        flat = np.asarray(self.parameters['end_to_camera_flat'], dtype=float)
        if flat.shape != (16,) or not np.all(np.isfinite(flat)):
            raise RuntimeError('end_to_camera_flat must contain 16 numbers')
        self.end_to_camera = flat.reshape(4, 4)
        if not np.allclose(self.end_to_camera[3], [0.0, 0.0, 0.0, 1.0]):
            raise RuntimeError('invalid homogeneous hand-eye matrix')

        self.publisher = self.create_publisher(
            PointStamped, self.parameters['output_topic'], 10)
        self.subscription = self.create_subscription(
            PointStamped,
            self.parameters['input_topic'],
            self.target_callback,
            1,
        )
        self.pose_client = self.create_client(
            SetJointValue, self.parameters['pose_service'])
        self.read_client = self.create_client(
            GetBusServoState, '/ros_robot_controller/bus_servo/get_state')
        self.pending = None
        self.pending_message = None
        self.latest_message = None
        self.publish_count = 0
        self.measured = []
        self.pending_started = 0.0
        self.observation_enabled = True
        self.observation_after_s = 0.0
        self.last_requested_key = None
        self.create_subscription(String, '/grasp_once_node/status', self.stage_callback, 10)
        self.create_timer(0.25, self.observation_tick)
        self.get_logger().info(
            'TargetToIkNode started: camera target -> arm_ik_world')

    @staticmethod
    def message_key(message):
        """Return the source image timestamp as a comparable tuple."""
        return (message.header.stamp.sec, message.header.stamp.nanosec)

    def target_callback(self, message):
        """Retain the newest valid camera target and request vendor FK."""
        point = np.asarray([
            message.point.x,
            message.point.y,
            message.point.z,
        ], dtype=float)
        if not np.all(np.isfinite(point)) or point[2] <= 0.05:
            self.get_logger().warning('Rejected invalid camera target')
            return
        self.latest_message = message

    def stage_callback(self, message):
        """Leave the servo bus to motion feedback while any joint is moving."""
        try:
            stage = json.loads(message.data)['stage']
        except (ValueError, KeyError, TypeError):
            return
        allowed = stage in ('idle', 'observing', 'prepared', 'auto_grasp',
                            'centered', 'search_observing', 'centering_observing')
        if allowed and not self.observation_enabled:
            self.observation_after_s = self.get_clock().now().nanoseconds * 1e-9
            self.latest_message = None
        self.observation_enabled = allowed
        if not allowed:
            if self.pending is not None:
                future, self.pending = self.pending, None
                future.cancel()
            self.pending_message = None
            self.latest_message = None

    def observation_tick(self):
        self.check_timeout()
        if self.observation_enabled and self.pending is None:
            self.request_pose()

    def request_pose(self):
        """Read real joints twice, then request FK; never use command cache."""
        if not self.observation_enabled or self.latest_message is None:
            return
        key = self.message_key(self.latest_message)
        stamp = key[0] + key[1] * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if (key == self.last_requested_key or stamp <= self.observation_after_s
                or not -0.05 <= age <= float(self.parameters['observation_max_age_s'])):
            return
        if not (self.pose_client.service_is_ready()
                and self.read_client.service_is_ready()):
            self.get_logger().warning(
                'Waiting for actual joint readback and FK services')
            return
        self.pending_message = self.latest_message
        self.last_requested_key = key
        self.measured = []
        self.pending_started = time.monotonic()
        self.read_next_joint()

    def read_next_joint(self):
        request = GetBusServoState.Request()
        command = GetBusServoCmd()
        command.id = len(self.measured) % 4 + 1
        command.get_position = 1
        request.cmd.append(command)
        self.pending = self.read_client.call_async(request)
        self.pending.add_done_callback(self.joint_done)

    def joint_done(self, future):
        if future is not self.pending:
            return
        try:
            result = future.result()
            if (result is None or not result.success or len(result.state) != 1
                    or len(result.state[0].position) != 1):
                raise RuntimeError('actual joint position unavailable')
            pulse = int(result.state[0].position[0])
            if not 0 <= pulse <= 1000:
                raise RuntimeError('invalid actual joint position')
            self.measured.append(pulse)
            if len(self.measured) < 8:
                self.read_next_joint()
                return
            if np.max(np.abs(np.array(self.measured[:4])
                             - np.array(self.measured[4:]))) > 2:
                raise RuntimeError('arm is moving during camera observation')
            request = SetJointValue.Request()
            request.joint_value = [float(p) for p in self.measured[4:]] + [
                float(self.parameters['camera_roll_reference'])]
            self.pending = self.pose_client.call_async(request)
            self.pending.add_done_callback(self.pose_done)
        except Exception as error:
            self.get_logger().warning(f'Actual-pose conversion rejected: {error}')
            self.finish_request()

    def check_timeout(self):
        if self.pending is not None and time.monotonic() - self.pending_started > 1.0:
            future, self.pending = self.pending, None
            future.cancel()
            self.get_logger().warning('Actual-pose conversion timed out')
            self.finish_request()

    def finish_request(self):
        self.pending = None
        self.pending_message = None

    def pose_done(self, future):
        """Publish a target expressed in the vendor IK world frame."""
        if future is not self.pending:
            return
        message = self.pending_message
        try:
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            if not message.header.frame_id or not -0.05 <= age <= float(
                    self.parameters['observation_max_age_s']):
                raise RuntimeError('stale camera observation or missing frame')
            response = future.result()
            if response is None or not response.success or not response.solution:
                raise RuntimeError('vendor FK did not return an endpoint pose')
            endpoint = pose_matrix(response.pose)
            endpoint_xyz = endpoint[:3, 3]
            if (self.parameters['reject_uninitialized_pose']
                    and np.linalg.norm(endpoint_xyz[:2]) < 0.01
                    and endpoint_xyz[2] > 0.40):
                raise RuntimeError(
                    'kinematics pose cache is uninitialized; call prepare first')

            camera = np.asarray([
                message.point.x,
                message.point.y,
                message.point.z,
                1.0,
            ], dtype=float)
            offset = float(self.parameters['surface_to_center_m'])
            if not 0.0 <= offset <= 0.05:
                raise RuntimeError('surface_to_center_m outside [0, 0.05]')
            scale = (camera[2] + offset) / camera[2]
            camera[:3] *= scale

            target = endpoint @ self.end_to_camera @ camera
            if not np.all(np.isfinite(target)) or not math.isclose(
                    float(target[3]), 1.0, abs_tol=1e-6):
                raise RuntimeError('invalid IK target transform result')

            output = PointStamped()
            output.header.stamp = message.header.stamp
            output.header.frame_id = self.parameters['target_frame']
            output.point.x = float(target[0])
            output.point.y = float(target[1])
            output.point.z = float(target[2])
            self.publisher.publish(output)
            self.publish_count += 1
            if self.publish_count % 15 == 1:
                self.get_logger().info(
                    'Target in arm_ik_world: '
                    f'X={target[0]:.3f}, Y={target[1]:.3f}, '
                    f'Z={target[2]:.3f} m')
        except Exception as error:
            self.get_logger().warning(f'IK-frame conversion rejected: {error}')

        self.finish_request()


def main(args=None):
    """Run the ROS node."""
    rclpy.init(args=args)
    node = TargetToIkNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
