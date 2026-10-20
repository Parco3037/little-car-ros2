#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
target_to_base_node.py

作用：

订阅：
    /rgbd_grasp/target_point

输入是蓝色方块在 Aurora930 相机坐标系中的三维位置。

例如：

    frame_id = rgb_camera_link

    x = -0.013
    y = -0.029
    z =  0.194

利用 ROS2 TF，把这个点转换到：

    base_link

最后发布：

    /rgbd_grasp/target_point_base


为什么单独写一个节点？

因为视觉检测和坐标变换属于两个不同功能。

blue_block_3d_node：
    负责“看见物体”。

target_to_base_node：
    负责“告诉机器人这个物体在哪里”。

以后无论我们换 YOLO、换相机、换检测算法，
机械臂这一层都不需要跟着大改。
"""


import rclpy

from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PointStamped

from tf2_ros import (
    Buffer,
    TransformListener,
    TransformException
)

from tf2_geometry_msgs import do_transform_point


class TargetToBaseNode(Node):

    def __init__(self):

        super().__init__('target_to_base_node')

        # ============================================================
        # ROS 参数
        # ============================================================

        # 输入：
        # 蓝色方块在相机坐标系中的位置
        self.declare_parameter(
            'input_topic',
            '/rgbd_grasp/target_point'
        )

        # 输出：
        # 蓝色方块在机器人 base_link 中的位置
        self.declare_parameter(
            'output_topic',
            '/rgbd_grasp/target_point_base'
        )

        # 目标坐标系
        self.declare_parameter(
            'target_frame',
            'base_link'
        )

        self.input_topic = (
            self.get_parameter('input_topic')
            .get_parameter_value()
            .string_value
        )

        self.output_topic = (
            self.get_parameter('output_topic')
            .get_parameter_value()
            .string_value
        )

        self.target_frame = (
            self.get_parameter('target_frame')
            .get_parameter_value()
            .string_value
        )

        # ============================================================
        # TF Buffer
        # ============================================================
        #
        # Buffer：
        #     保存 ROS 中最近一段时间的 TF 数据。
        #
        # TransformListener：
        #     自动监听 /tf 和 /tf_static，
        #     把接收到的数据放入 Buffer。
        #

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ============================================================
        # Subscriber
        # ============================================================

        self.target_sub = self.create_subscription(
            PointStamped,
            self.input_topic,
            self.target_callback,
            10
        )

        # ============================================================
        # Publisher
        # ============================================================

        self.target_base_pub = self.create_publisher(
            PointStamped,
            self.output_topic,
            10
        )

        self.count = 0

        self.get_logger().info(
            'TargetToBaseNode started.'
        )

        self.get_logger().info(
            f'Input topic: {self.input_topic}'
        )

        self.get_logger().info(
            f'Target frame: {self.target_frame}'
        )


    def target_callback(self, msg):

        """
        输入 msg：

            header.frame_id = rgb_camera_link

            point.x
            point.y
            point.z

        输出：

            header.frame_id = base_link

            point.x
            point.y
            point.z
        """

        source_frame = msg.header.frame_id

        if source_frame == '':

            self.get_logger().warning(
                'Target PointStamped has empty frame_id.'
            )

            return

        # ============================================================
        # 查询 TF
        # ============================================================
        #
        # 这里第一版使用“最新 TF”。
        #
        # 原因是：
        # 当前阶段我们检测物体的时候，
        # 机械臂基本保持静止。
        #
        # 这样做比严格按图像时间戳查询更容易调试。
        #
        # 后面机械臂与视觉同时高速运动以后，
        # 我们再改成严格的时间同步 TF。
        #

        try:

            transform = (
                self.tf_buffer.lookup_transform(
                    self.target_frame,
                    source_frame,
                    Time(),
                    timeout=Duration(
                        seconds=0.2
                    )
                )
            )

        except TransformException as e:

            self.get_logger().warning(
                f'TF failed: '
                f'{source_frame} -> '
                f'{self.target_frame}: '
                f'{e}'
            )

            return

        # ============================================================
        # 真正做坐标变换
        # ============================================================
        #
        # do_transform_point 会自动处理：
        #
        # 平移
        # 旋转
        #
        # 所以我们不需要自己写 4x4 矩阵。
        #

        try:

            point_base = do_transform_point(
                msg,
                transform
            )

        except Exception as e:

            self.get_logger().error(
                f'Point transform error: {e}'
            )

            return

        # 确保 frame_id 正确
        point_base.header.frame_id = (
            self.target_frame
        )

        self.target_base_pub.publish(
            point_base
        )

        # ============================================================
        # 调试输出
        # ============================================================

        self.count += 1

        # 不每帧刷终端。
        if self.count % 15 == 0:

            self.get_logger().info(
                'Target in base_link: '
                f'X={point_base.point.x:.3f}, '
                f'Y={point_base.point.y:.3f}, '
                f'Z={point_base.point.z:.3f} m'
            )


def main(args=None):

    rclpy.init(args=args)

    node = TargetToBaseNode()

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
