#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
blue_block_3d_node.py

功能：
1. 订阅 Aurora930 RGB 图像
2. 订阅 Aurora930 Depth 图像
3. 订阅 RGB CameraInfo
4. 检测一个或多个蓝色方块
5. 读取每个蓝色方块中心附近 5x5 像素区域的深度
6. 根据相机内参计算目标在相机坐标系中的 X、Y、Z
7. 自动选择一个目标
8. 发布选中目标的 PointStamped
9. 发布所有蓝色块的 [u,v,Z] 数据
10. 发布调试图像

本阶段仍然不控制机械臂。
"""

import json
import os
from pathlib import Path
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray, String

from cv_bridge import CvBridge

from message_filters import (
    Subscriber,
    ApproximateTimeSynchronizer
)


class BlueBlock3DNode(Node):

    def __init__(self):

        super().__init__('blue_block_3d_node')

        # ============================================================
        # ROS 参数
        # ============================================================

        # 根据你已经验证成功的真实 Aurora930 Topic，
        # 这里直接使用 /aurora/... 作为默认值。

        self.declare_parameter(
            'rgb_topic',
            '/aurora/rgb/image_raw'
        )

        self.declare_parameter(
            'depth_topic',
            '/aurora/depth/image_raw'
        )

        self.declare_parameter(
            'camera_info_topic',
            '/aurora/rgb/camera_info'
        )

        # 太小的蓝色区域认为是噪声。
        self.declare_parameter(
            'min_contour_area',
            300.0
        )

        # 对单个正方形允许一定透视变形。
        #
        # 理想正方形：
        #
        # 长边 / 短边 = 1
        #
        # 这里允许最大到 1.7。
        self.declare_parameter(
            'max_square_aspect_ratio',
            1.7
        )

        # 临时选择目标的方法。
        #
        # min_v：
        #     选择图像中 v 最小的方块
        #     通常就是画面中更靠上的方块
        #
        # max_v：
        #     选择 v 最大的方块
        #
        # 如果你的摄像头安装方向与正常图像相反，
        # 可以运行时改成 max_v。
        self.declare_parameter(
            'target_selection',
            'nearest'
        )

        self.rgb_topic = (
            self.get_parameter('rgb_topic')
            .get_parameter_value()
            .string_value
        )

        self.depth_topic = (
            self.get_parameter('depth_topic')
            .get_parameter_value()
            .string_value
        )

        self.camera_info_topic = (
            self.get_parameter('camera_info_topic')
            .get_parameter_value()
            .string_value
        )

        self.min_contour_area = (
            self.get_parameter('min_contour_area')
            .get_parameter_value()
            .double_value
        )

        self.max_square_aspect_ratio = (
            self.get_parameter('max_square_aspect_ratio')
            .get_parameter_value()
            .double_value
        )

        self.target_selection = (
            self.get_parameter('target_selection')
            .get_parameter_value()
            .string_value
        )

        # ============================================================
        # CvBridge
        # ============================================================

        self.bridge = CvBridge()

        # ============================================================
        # 保存相机内参
        # ============================================================
        #
        # CameraInfo 中最重要的是矩阵 K：
        #
        # [ fx  0  cx ]
        # [  0 fy  cy ]
        # [  0  0   1 ]
        #
        # 我们真正需要：
        #
        # fx
        # fy
        # cx
        # cy

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_frame = None

        # ============================================================
        # 蓝色 HSV 范围
        # ============================================================
        #
        # 这还是第一版经验值。
        #
        # 后面如果发现光照变化导致识别不稳，
        # 我们再专门做 HSV 标定工具。

        self.lower_blue = np.array(
            [95, 80, 50],
            dtype=np.uint8
        )

        self.upper_blue = np.array(
            [135, 255, 255],
            dtype=np.uint8
        )

        # ============================================================
        # CameraInfo
        # ============================================================
        #
        # CameraInfo 没必要和每一帧图像强行同步。
        #
        # 相机内参启动以后基本不会变化，
        # 所以收到一次以后保存下来即可。

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        # ============================================================
        # RGB + Depth 同步订阅
        # ============================================================

        self.rgb_sub = Subscriber(
            self,
            Image,
            self.rgb_topic,
            qos_profile=qos_profile_sensor_data
        )

        self.depth_sub = Subscriber(
            self,
            Image,
            self.depth_topic,
            qos_profile=qos_profile_sensor_data
        )

        self.sync = ApproximateTimeSynchronizer(
            [
                self.rgb_sub,
                self.depth_sub
            ],
            queue_size=10,
            slop=0.08
        )

        self.sync.registerCallback(
            self.image_callback
        )

        # ============================================================
        # Publisher
        # ============================================================

        # RViz 调试画面
        self.debug_pub = self.create_publisher(
            Image,
            '/rgbd_grasp/debug_image_3d',
            10
        )

        # 所有检测到的蓝块信息
        #
        # 格式：
        #
        # [u1, v1, depth1,
        #  u2, v2, depth2,
        #  ...]
        #
        self.blocks_pub = self.create_publisher(
            Float32MultiArray,
            '/rgbd_grasp/blocks_uvz',
            10
        )

        # 当前选中的抓取目标。
        #
        # 这里已经是真正的三维点：
        #
        # X
        # Y
        # Z
        #
        # 单位全部是米。
        self.target_point_pub = self.create_publisher(
            PointStamped,
            '/rgbd_grasp/target_point',
            10
        )

        self.frame_count = 0
        # One passive selection record per attempt; never commands the arm.
        self.declare_parameter(
            'selection_record_dir',
            str(Path.home() / 'raicom_runtime' / 'rgbd_grasp' / 'selection')
        )
        self.selection_record_dir = self.get_parameter('selection_record_dir').value
        self.latest_selection_frame = None
        self.selection_recorded = False
        self.status_sub = self.create_subscription(
            String, '/grasp_once_node/status', self.grasp_status_callback, 10
        )

        self.get_logger().info(
            'BlueBlock3DNode started.'
        )

        self.get_logger().info(
            f'RGB: {self.rgb_topic}'
        )

        self.get_logger().info(
            f'Depth: {self.depth_topic}'
        )

        self.get_logger().info(
            f'CameraInfo: {self.camera_info_topic}'
        )


    def grasp_status_callback(self, msg):
        """Record the latest camera frame when the grasp plan becomes ready."""
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if not isinstance(status, dict):
            return
        if status.get('stage') == 'prepare':
            self.selection_recorded = False
        if (status.get('stage') != 'ready' or self.selection_recorded
                or not self.selection_record_dir):
            return
        self.selection_recorded = True
        if self.latest_selection_frame is None:
            self.get_logger().warning('Selection record unavailable: no camera frame')
            return
        rgb, depth, mask, debug, metadata, received_at = self.latest_selection_frame
        try:
            root = Path(self.selection_record_dir) / (
                'domain_' + str(self.context.get_domain_id())
            )
            directory = root / f'{time.time_ns()}_{os.getpid()}'
            directory.mkdir(parents=True, exist_ok=False)
            for name, frame in [('rgb', rgb), ('mask', mask), ('debug', debug)]:
                if not cv2.imwrite(str(directory / f'{name}.png'), frame):
                    raise OSError(f'could not save {name}.png')
            # Preserve the original integer/float depth representation exactly.
            np.save(str(directory / 'depth.npy'), depth, allow_pickle=False)
            record = dict(metadata, ready_status=status,
                          frame_age_at_ready_s=time.monotonic() - received_at,
                          note='Latest camera frame at ready; not an exact association '
                               'with the temporally filtered grasp target.')
            (directory / 'selection.json').write_text(
                json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8'
            )
            self.get_logger().info(f'Selection record: {directory}')
        except Exception as error:
            # A diagnostic I/O failure must not interrupt sensing or grasping.
            self.get_logger().warning(f'Selection record failed: {error}')

    # ================================================================
    # CameraInfo 回调
    # ================================================================

    def camera_info_callback(self, msg):

        # ROS CameraInfo.k 一共有 9 个数。
        #
        # 排列：
        #
        # k[0] k[1] k[2]
        # k[3] k[4] k[5]
        # k[6] k[7] k[8]

        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        self.camera_frame = msg.header.frame_id

        # 第一次收到 CameraInfo 时打印。
        #
        # 后面不再重复刷屏。

        if not hasattr(
            self,
            'camera_info_received'
        ):

            self.camera_info_received = True

            self.get_logger().info(
                'Camera intrinsics received: '
                f'fx={self.fx:.2f}, '
                f'fy={self.fy:.2f}, '
                f'cx={self.cx:.2f}, '
                f'cy={self.cy:.2f}'
            )

            self.get_logger().info(
                f'Camera frame: '
                f'{self.camera_frame}'
            )


    # ================================================================
    # 检测蓝色方块
    # ================================================================

    def detect_blue_blocks(
        self,
        bgr_image
    ):

        """
        返回一个 detections 列表。

        每个 detection 是一个字典：

        {
            'u': 中心 u,
            'v': 中心 v,
            'area': 面积,
            'box': 外接矩形角点,
            'type': 'single' 或 'split'
        }

        single：
            真正检测到了一个独立正方形轮廓

        split：
            检测到一个大约 1:2 的整体蓝色区域，
            根据“两个相同方块上下堆叠”的先验信息
            人为拆成两个目标
        """

        # ------------------------------------------------------------
        # 图像预处理
        # ------------------------------------------------------------

        blurred = cv2.GaussianBlur(
            bgr_image,
            (5, 5),
            0
        )

        hsv = cv2.cvtColor(
            blurred,
            cv2.COLOR_BGR2HSV
        )

        mask = cv2.inRange(
            hsv,
            self.lower_blue,
            self.upper_blue
        )

        # ------------------------------------------------------------
        # 形态学处理
        # ------------------------------------------------------------

        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (5, 5)
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel
        )

        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            kernel
        )

        # ------------------------------------------------------------
        # 找轮廓
        # ------------------------------------------------------------

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        square_candidates = []

        # 如果两个蓝色方块粘在一起，
        # 这里保存可能的“大矩形”。
        stacked_candidates = []

        for contour in contours:

            area = cv2.contourArea(
                contour
            )

            if area < self.min_contour_area:
                continue

            rect = cv2.minAreaRect(
                contour
            )

            rect_width = rect[1][0]
            rect_height = rect[1][1]

            if (
                rect_width <= 1.0
                or rect_height <= 1.0
            ):
                continue

            short_side = min(
                rect_width,
                rect_height
            )

            long_side = max(
                rect_width,
                rect_height
            )

            aspect_ratio = (
                long_side / short_side
            )

            rect_area = (
                rect_width
                * rect_height
            )

            fill_ratio = (
                area / rect_area
            )

            # ========================================================
            # 情况 1：
            # 一个独立的正方形
            # ========================================================

            if (
                aspect_ratio
                <= self.max_square_aspect_ratio
                and fill_ratio > 0.55
                and not (1.55 <= cv2.boundingRect(contour)[3]
                         / float(cv2.boundingRect(contour)[2]) <= 2.8)
            ):

                center_x = rect[0][0]
                center_y = rect[0][1]

                box = cv2.boxPoints(
                    rect
                )

                box = np.int32(
                    box
                )

                square_candidates.append(
                    {
                        'u': int(
                            round(center_x)
                        ),

                        'v': int(
                            round(center_y)
                        ),

                        'area': area,

                        'box': box,

                        'type': 'single'
                    }
                )

                continue

            # ========================================================
            # 情况 2：
            # 两个相同蓝块上下贴在一起，
            # HSV 把它们识别成一个整体
            # ========================================================
            #
            # 对两个 5x5 正方形上下拼接：
            #
            # 整体大约是：
            #
            # 5 cm x 10 cm
            #
            # 所以长宽比大约是 2。
            #
            # 这里允许：
            #
            # 1.55 ~ 2.8
            #
            # 给透视和相机角度留一定余量。

            x, y, w, h = cv2.boundingRect(
                contour
            )

            vertical_ratio = (
                h / float(w)
            )

            if (
                1.55
                <= vertical_ratio
                <= 2.8
                and fill_ratio > 0.50
            ):

                stacked_candidates.append(
                    {
                        'x': x,
                        'y': y,
                        'w': w,
                        'h': h,
                        'area': area
                    }
                )

        # ============================================================
        # 独立方块与贴合方块同时保留，交给深度选择最近的一组。
        detections = list(square_candidates)
        for stack in stacked_candidates:
            x, y, w, h = (stack[key] for key in ('x', 'y', 'w', 'h'))
            center_u = int(x + w / 2)
            for fraction in (0.25, 0.75):
                detections.append({
                    'u': center_u,
                    'v': int(y + h * fraction),
                    'area': stack['area'] / 2.0,
                    'box': None,
                    'type': 'split',
                })

        # ------------------------------------------------------------
        # 按 v 从小到大排序
        # ------------------------------------------------------------
        #
        # 因此：
        #
        # detections[0]
        #
        # 通常是图像里更上面的那个。

        detections.sort(
            key=lambda d: d['v']
        )

        return detections, mask


    # ================================================================
    # 获取 5x5 Depth ROI
    # ================================================================

    def get_depth_at_target(
        self,
        depth_image,
        depth_encoding,
        u,
        v
    ):

        height, width = (
            depth_image.shape[:2]
        )

        if (
            u < 0
            or u >= width
            or v < 0
            or v >= height
        ):
            return None

        # 中心周围 5x5 像素
        x_min = max(
            0,
            u - 2
        )

        x_max = min(
            width,
            u + 3
        )

        y_min = max(
            0,
            v - 2
        )

        y_max = min(
            height,
            v + 3
        )

        roi = depth_image[
            y_min:y_max,
            x_min:x_max
        ]

        values = (
            roi
            .astype(np.float32)
            .reshape(-1)
        )

        values = values[
            np.isfinite(values)
        ]

        if values.size == 0:
            return None

        # Aurora930 当前使用 uint16 Depth，
        # 单位按照你已经验证成功的程序处理为 mm。

        if (
            depth_encoding == '16UC1'
            or depth_encoding == 'mono16'
            or np.issubdtype(
                depth_image.dtype,
                np.integer
            )
        ):

            values = values[
                (values > 100.0)
                & (values < 4000.0)
            ]

            if values.size == 0:
                return None

            depth_m = (
                float(
                    np.median(values)
                )
                / 1000.0
            )

        else:

            values = values[
                (values > 0.10)
                & (values < 4.0)
            ]

            if values.size == 0:
                return None

            depth_m = float(
                np.median(values)
            )

        return depth_m


    # ================================================================
    # 像素 + Depth -> 相机三维坐标
    # ================================================================

    def pixel_to_camera(
        self,
        u,
        v,
        depth
    ):

        """
        输入：

        u, v:
            RGB 图像像素坐标

        depth:
            相机到目标距离 Z，单位 m

        返回：

        X, Y, Z

        注意：

        这里得到的是 Camera Optical Frame 中的坐标，
        还不是机械臂 base_link 坐标。
        """

        if (
            self.fx is None
            or self.fy is None
            or self.cx is None
            or self.cy is None
        ):
            return None

        z = depth

        x = (
            (u - self.cx)
            * z
            / self.fx
        )

        y = (
            (v - self.cy)
            * z
            / self.fy
        )

        return (
            float(x),
            float(y),
            float(z)
        )


    # ================================================================
    # RGB + Depth 回调
    # ================================================================

    def image_callback(
        self,
        rgb_msg,
        depth_msg
    ):

        try:

            bgr_image = (
                self.bridge.imgmsg_to_cv2(
                    rgb_msg,
                    desired_encoding='bgr8'
                )
            )

            depth_image = (
                self.bridge.imgmsg_to_cv2(
                    depth_msg,
                    desired_encoding='passthrough'
                )
            )

        except Exception as e:

            self.get_logger().error(
                f'CvBridge error: {e}'
            )

            return

        depth_image = np.squeeze(
            depth_image
        )

        if depth_image.ndim != 2:

            self.get_logger().error(
                'Depth image is not 2D.'
            )

            return

        rgb_h, rgb_w = (
            bgr_image.shape[:2]
        )

        depth_h, depth_w = (
            depth_image.shape[:2]
        )

        if (
            rgb_w != depth_w
            or rgb_h != depth_h
        ):

            self.get_logger().warning(
                'RGB and Depth resolutions '
                'are different.'
            )

            return

        # ------------------------------------------------------------
        # 检测所有蓝色方块
        # ------------------------------------------------------------

        detections, mask = (
            self.detect_blue_blocks(
                bgr_image
            )
        )

        debug_image = (
            bgr_image.copy()
        )

        valid_blocks = []
        detection_records = []

        # ------------------------------------------------------------
        # 对每一个蓝色方块计算 Depth 和 XYZ
        # ------------------------------------------------------------

        for index, detection in enumerate(
            detections
        ):

            u = detection['u']
            v = detection['v']

            depth_m = (
                self.get_depth_at_target(
                    depth_image,
                    depth_msg.encoding,
                    u,
                    v
                )
            )

            detection_record = {
                'u': int(u), 'v': int(v), 'type': detection['type'],
                'area': float(detection['area']),
                'box': (detection['box'].tolist()
                        if detection['box'] is not None else None),
                'depth_m': depth_m, 'rejected_reason': None,
            }
            detection_records.append(detection_record)
            if depth_m is None:
                detection_record['rejected_reason'] = 'no_valid_depth'
                continue

            xyz = self.pixel_to_camera(
                u,
                v,
                depth_m
            )

            if xyz is None:
                detection_record['rejected_reason'] = 'camera_intrinsics_unavailable'
                continue

            x, y, z = xyz

            block = {
                'index': index,
                'u': u,
                'v': v,
                'depth': depth_m,
                'x': x,
                'y': y,
                'z': z,
                'box': detection['box'],
                'type': detection['type']
            }

            valid_blocks.append(
                block
            )

        # ------------------------------------------------------------
        # 发布所有蓝块的 UVZ
        # ------------------------------------------------------------

        blocks_msg = (
            Float32MultiArray()
        )

        blocks_msg.data = []

        for block in valid_blocks:

            blocks_msg.data.extend(
                [
                    float(block['u']),
                    float(block['v']),
                    float(block['depth'])
                ]
            )

        self.blocks_pub.publish(
            blocks_msg
        )

        # ------------------------------------------------------------
        # 选择当前目标
        # ------------------------------------------------------------

        target = None

        if len(valid_blocks) > 0:

            if self.target_selection == 'nearest':
                near_depth = min(block['depth'] for block in valid_blocks)
                nearby = [block for block in valid_blocks
                          if block['depth'] <= near_depth + 0.03]
                target = min(nearby, key=lambda d: d['v'])
            elif self.target_selection == 'max_v':
                target = max(valid_blocks, key=lambda d: d['v'])
            else:
                target = min(valid_blocks, key=lambda d: d['v'])

        # ------------------------------------------------------------
        # 画检测结果
        # ------------------------------------------------------------

        for i, block in enumerate(
            valid_blocks
        ):

            u = block['u']
            v = block['v']

            # 独立检测出的方块可以画完整矩形。
            if block['box'] is not None:

                cv2.drawContours(
                    debug_image,
                    [block['box']],
                    -1,
                    (0, 255, 0),
                    2
                )

            # 画中心
            cv2.circle(
                debug_image,
                (u, v),
                6,
                (0, 255, 255),
                -1
            )

            # 画 5x5 深度采样区域
            cv2.rectangle(
                debug_image,
                (u - 2, v - 2),
                (u + 2, v + 2),
                (255, 0, 255),
                1
            )

            label = (
                f'B{i}: '
                f'u={u} v={v} '
                f'Z={block["z"]:.3f}m'
            )

            cv2.putText(
                debug_image,
                label,
                (
                    max(5, u - 80),
                    max(25, v - 15)
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

        # ------------------------------------------------------------
        # 发布被选中的目标三维点
        # ------------------------------------------------------------

        if target is not None:

            point_msg = PointStamped()

            point_msg.header.stamp = (
                rgb_msg.header.stamp
            )

            # 优先使用 CameraInfo 中真正的 frame_id。
            if self.camera_frame:

                point_msg.header.frame_id = (
                    self.camera_frame
                )

            else:

                point_msg.header.frame_id = (
                    rgb_msg.header.frame_id
                )

            point_msg.point.x = (
                target['x']
            )

            point_msg.point.y = (
                target['y']
            )

            point_msg.point.z = (
                target['z']
            )

            self.target_point_pub.publish(
                point_msg
            )

            # 在目标上额外画 TARGET
            cv2.putText(
                debug_image,
                'TARGET',
                (
                    target['u'] + 10,
                    target['v'] + 25
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

            self.frame_count += 1

            if (
                self.frame_count % 15
                == 0
            ):

                self.get_logger().info(
                    f'Detected '
                    f'{len(valid_blocks)} block(s). '
                    f'RGB candidates={len(detections)}, '
                    f'selection={self.target_selection}, '
                    f'type={target["type"]}. '
                    f'Target: '
                    f'u={target["u"]}, '
                    f'v={target["v"]}, '
                    f'XYZ=('
                    f'{target["x"]:.3f}, '
                    f'{target["y"]:.3f}, '
                    f'{target["z"]:.3f}) m'
                )

        # ------------------------------------------------------------
        # 图像左上角显示检测数量
        # ------------------------------------------------------------

        cv2.putText(
            debug_image,
            f'Blocks: {len(valid_blocks)}',
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 0),
            2
        )

        if self.selection_record_dir:
            self.latest_selection_frame = (
                bgr_image.copy(), depth_image.copy(), mask.copy(), debug_image.copy(),
                {
                    'rgb_stamp': {'sec': rgb_msg.header.stamp.sec,
                                  'nanosec': rgb_msg.header.stamp.nanosec},
                    'depth_stamp': {'sec': depth_msg.header.stamp.sec,
                                    'nanosec': depth_msg.header.stamp.nanosec},
                    'depth_encoding': depth_msg.encoding,
                    'intrinsics': [self.fx, self.fy, self.cx, self.cy],
                    'raw_candidate_count': len(detections),
                    'valid_candidate_count': len(valid_blocks),
                    'target_selection': self.target_selection,
                    'detections': detection_records,
                    'selected_uvz': ([target['u'], target['v'], target['depth']]
                                     if target is not None else None),
                }, time.monotonic()
            )

        # ------------------------------------------------------------
        # 发布调试图像
        # ------------------------------------------------------------

        try:

            debug_msg = (
                self.bridge.cv2_to_imgmsg(
                    debug_image,
                    encoding='bgr8'
                )
            )

            debug_msg.header = (
                rgb_msg.header
            )

            self.debug_pub.publish(
                debug_msg
            )

        except Exception as e:

            self.get_logger().error(
                f'Debug image publish error: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = BlueBlock3DNode()

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
