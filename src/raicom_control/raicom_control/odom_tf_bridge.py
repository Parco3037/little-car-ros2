"""Use one deterministic odometry source for both Nav2 messages and TF."""

from copy import deepcopy

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .odom_bridge_logic import pose_sample_is_valid, validate_bridge_configuration


class OdomTfBridge(Node):
    """Republish raw odometry and broadcast the matching transform unchanged."""

    def __init__(self) -> None:
        super().__init__('odom_tf_bridge')
        self.declare_parameter('input_topic', '/odom_raw')
        self.declare_parameter('output_topic', '/odom')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_footprint')

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.parent_frame = str(self.get_parameter('parent_frame').value)
        self.child_frame = str(self.get_parameter('child_frame').value)
        validate_bridge_configuration(
            input_topic,
            output_topic,
            self.parent_frame,
            self.child_frame,
        )

        self.publisher = self.create_publisher(Odometry, output_topic, 10)
        self.broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            input_topic,
            self._on_odometry,
            10,
        )
        self.invalid_samples = 0
        self.get_logger().info(
            f'Raw odometry route active: {input_topic} -> {output_topic}; '
            f'TF {self.parent_frame} -> {self.child_frame}'
        )

    def _on_odometry(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not pose_sample_is_valid(values):
            self.invalid_samples += 1
            if self.invalid_samples == 1 or self.invalid_samples % 50 == 0:
                self.get_logger().error(
                    f'rejected invalid odometry pose sample; count={self.invalid_samples}'
                )
            return

        output = deepcopy(msg)
        output.header.frame_id = self.parent_frame
        output.child_frame_id = self.child_frame
        self.publisher.publish(output)

        transform = TransformStamped()
        transform.header.stamp = output.header.stamp
        transform.header.frame_id = self.parent_frame
        transform.child_frame_id = self.child_frame
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        self.broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTfBridge()
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
