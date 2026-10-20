#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
blue_block_depth_node.py

功能：
1. 同步订阅 Aurora930 的 RGB 图像和 Depth 图像
2. 在 RGB 图像中使用 HSV 颜色空间寻找蓝色方块
3. 得到蓝色方块中心像素坐标 (u, v)
4. 在 Depth 图中读取 (u, v) 周围 5x5 像素区域
5. 去除无效深度值，然后取中位数作为目标深度
6. 发布调试图像
7. 发布 [u, v, depth_m]

当前版本只实现：
    第二关：蓝色方块检测
    第三关：深度读取

当前版本不实现：
    CameraInfo 三维反投影
    相机坐标系 XYZ
    手眼标定
    机械臂控制
    小车视觉伺服
"""


import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

from cv_bridge import CvBridge

# message_filters 用于把 RGB 和 Depth 按时间戳进行近似同步
from message_filters import Subscriber, ApproximateTimeSynchronizer


class BlueBlockDepthNode(Node):

    def __init__(self):
        super().__init__('blue_block_depth_node')

        # ============================================================
        # 1. ROS 参数
        # ============================================================
        #
        # 官方 LanderPi 的 Aurora930 launch 会把 Aurora 原始话题
        # remap 成 /ascamera/... 形式。
        #
        # 因此这里默认直接使用官方 LanderPi 工作空间的话题。
        #
        # 如果以后你直接运行：
        #
        # ros2 launch deptrum-ros-driver-aurora930 viewer930_launch.py
        #
        # 而没有经过幻尔的 remap，
        # 可以在运行节点时通过 ROS 参数修改话题。
        #

        self.declare_parameter(
            'rgb_topic',
            '/ascamera/camera_publisher/rgb0/image'
        )

        self.declare_parameter(
            'depth_topic',
            '/ascamera/camera_publisher/depth0/image_raw'
        )

        # 最小轮廓面积。
        # 面积过小一般是颜色噪声，不认为是目标。
        self.declare_parameter('min_contour_area', 300.0)

        # 因为我们的目标是正方形，
        # 最长边 / 最短边不能过大。
        #
        # 理想正方形 = 1.0
        # 考虑透视、相机角度等因素，这里放宽到 1.8。
        self.declare_parameter('max_aspect_ratio', 1.8)

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

        self.min_contour_area = (
            self.get_parameter('min_contour_area')
            .get_parameter_value()
            .double_value
        )

        self.max_aspect_ratio = (
            self.get_parameter('max_aspect_ratio')
            .get_parameter_value()
            .double_value
        )

        # ============================================================
        # 2. CvBridge
        # ============================================================
        #
        # ROS 的 sensor_msgs/Image 不能直接给 OpenCV 使用。
        #
        # CvBridge 用来完成：
        #
        # ROS Image <-> OpenCV numpy 数组
        #

        self.bridge = CvBridge()

        # ============================================================
        # 3. HSV 蓝色阈值
        # ============================================================
        #
        # OpenCV 中 HSV 的 H 范围不是 0~360，
        # 而是 0~179。
        #
        # 蓝色通常大约位于 H=100~130。
        #
        # 这里先给一个比较宽松的初始范围。
        # 后面如果你的蓝色方块识别不好，我们再根据实际画面调。
        #

        self.lower_blue = np.array(
            [95, 80, 50],
            dtype=np.uint8
        )

        self.upper_blue = np.array(
            [135, 255, 255],
            dtype=np.uint8
        )

        # ============================================================
        # 4. RGB 和 Depth 订阅
        # ============================================================
        #
        # 不建议分别：
        #
        # self.create_subscription(RGB)
        # self.create_subscription(Depth)
        #
        # 然后自己保存最新图像。
        #
        # 因为 RGB 和 Depth 是连续发布的，
        # 两个回调收到的图像可能不是同一时刻。
        #
        # 所以这里使用 message_filters。
        #

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

        # ApproximateTimeSynchronizer 的意思是：
        #
        # 不要求 RGB 和 Depth 时间戳完全一样，
        # 只要差距足够小，就把它们认为是同一组数据。
        #
        # queue_size=10：
        # 最多缓存 10 条消息。
        #
        # slop=0.08：
        # RGB 与 Depth 最多允许相差 0.08 秒。
        #
        # Aurora930 帧率不算特别高，这个值第一版比较宽松。
        #

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=10,
            slop=0.08
        )

        self.sync.registerCallback(self.image_callback)

        # ============================================================
        # 5. 发布结果
        # ============================================================

        # 发布处理后的 RGB 调试画面。
        #
        # 可以直接在 RViz2 或 rqt_image_view 中看。
        self.debug_pub = self.create_publisher(
            Image,
            '/rgbd_grasp/debug_image',
            10
        )

        # 发布目标信息：
        #
        # data[0] = u
        # data[1] = v
        # data[2] = depth，单位 m
        #
        # 第一版为了简单，直接用 Float32MultiArray。
        #
        # 第四关做 XYZ 时我们会换成
        # geometry_msgs/PointStamped。
        self.target_pub = self.create_publisher(
            Float32MultiArray,
            '/rgbd_grasp/target_uvz',
            10
        )

        # 用于控制终端输出频率。
        # 如果每帧都 print，15 FPS 时终端会刷得非常快。
        self.frame_count = 0

        self.get_logger().info('BlueBlockDepthNode started.')

        self.get_logger().info(
            f'RGB topic: {self.rgb_topic}'
        )

        self.get_logger().info(
            f'Depth topic: {self.depth_topic}'
        )


    # ================================================================
    # 蓝色方块检测
    # ================================================================
    def detect_blue_block(self, bgr_image):
        """
        输入：
            bgr_image:
                OpenCV BGR 图像

        返回：
            如果检测到蓝色方块：

                center = (u, v)
                box = 四个角点
                mask = 蓝色二值图

            如果没有检测到：

                center = None
                box = None
                mask = 蓝色二值图
        """

        # ------------------------------------------------------------
        # 1. 高斯滤波
        # ------------------------------------------------------------
        #
        # 相机图像中可能存在少量像素噪声。
        #
        # 轻微模糊后再做颜色分割会更稳定。
        #

        blurred = cv2.GaussianBlur(
            bgr_image,
            (5, 5),
            0
        )

        # ------------------------------------------------------------
        # 2. BGR -> HSV
        # ------------------------------------------------------------

        hsv = cv2.cvtColor(
            blurred,
            cv2.COLOR_BGR2HSV
        )

        # ------------------------------------------------------------
        # 3. 提取蓝色区域
        # ------------------------------------------------------------
        #
        # inRange 会把：
        #
        # 在蓝色范围内的像素设成 255
        # 其他像素设成 0
        #

        mask = cv2.inRange(
            hsv,
            self.lower_blue,
            self.upper_blue
        )

        # ------------------------------------------------------------
        # 4. 形态学处理
        # ------------------------------------------------------------
        #
        # 颜色分割之后可能出现：
        #
        # 小白点
        # 小黑洞
        # 断裂边缘
        #
        # 开运算主要用于去除小噪声。
        # 闭运算主要用于填补小孔洞。
        #

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
        # 5. 查找轮廓
        # ------------------------------------------------------------

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # 当前找到的最佳目标
        best_contour = None
        best_rect = None
        best_area = 0.0

        # ------------------------------------------------------------
        # 6. 从所有蓝色区域中寻找“像方块”的最大目标
        # ------------------------------------------------------------

        for contour in contours:

            area = cv2.contourArea(contour)

            # 太小的轮廓直接认为是噪声
            if area < self.min_contour_area:
                continue

            # minAreaRect 会求出一个可以旋转的最小外接矩形。
            #
            # 返回：
            #
            # ((center_x, center_y),
            #  (width, height),
            #  angle)
            rect = cv2.minAreaRect(contour)

            rect_width, rect_height = rect[1]

            if rect_width <= 1.0 or rect_height <= 1.0:
                continue

            # 正方形宽高应该接近。
            #
            # 为了不关心横竖方向：
            #
            # aspect_ratio 永远 >= 1。
            aspect_ratio = (
                max(rect_width, rect_height)
                / min(rect_width, rect_height)
            )

            if aspect_ratio > self.max_aspect_ratio:
                continue

            # 再检查轮廓对外接矩形的填充程度。
            #
            # 如果只是细长蓝线、奇怪的不规则形状，
            # 即使宽高看起来比较接近，
            # 填充比例通常也不会特别高。
            rect_area = rect_width * rect_height

            fill_ratio = area / rect_area

            if fill_ratio < 0.60:
                continue

            # 如果满足要求，
            # 选面积最大的蓝色方块作为目标。
            if area > best_area:
                best_area = area
                best_contour = contour
                best_rect = rect

        # ------------------------------------------------------------
        # 没有找到符合条件的蓝色方块
        # ------------------------------------------------------------

        if best_contour is None:
            return None, None, mask

        # ------------------------------------------------------------
        # 找到了方块
        # ------------------------------------------------------------

        center_x, center_y = best_rect[0]

        u = int(round(center_x))
        v = int(round(center_y))

        # 获取旋转矩形的 4 个角点
        box = cv2.boxPoints(best_rect)

        box = np.int32(box)

        return (u, v), box, mask


    # ================================================================
    # 从 Depth 图读取目标深度
    # ================================================================
    def get_depth_at_target(
        self,
        depth_image,
        depth_encoding,
        u,
        v
    ):
        """
        在 (u,v) 周围读取 5x5 像素深度。

        例如中心：

            (u,v)

        实际读取：

            u-2 ~ u+2
            v-2 ~ v+2

        总共最多 25 个像素。

        为什么不直接读取：

            depth[v, u]

        因为深度相机偶尔会出现某一个像素为：

            0
            NaN
            异常大值

        如果只读中心一个像素，结果很容易突然跳变。

        所以这里读取小区域，
        去掉异常值以后取 median（中位数）。

        中位数相比平均数更不容易受到极端异常值影响。
        """

        height, width = depth_image.shape[:2]

        # 防止目标中心跑出 Depth 图范围
        if (
            u < 0 or
            u >= width or
            v < 0 or
            v >= height
        ):
            return None

        # ------------------------------------------------------------
        # 5x5 像素窗口
        # ------------------------------------------------------------
        #
        # Python 切片右侧不包含终点。
        #
        # 所以：
        #
        # u-2 : u+3
        #
        # 一共有：
        #
        # u-2
        # u-1
        # u
        # u+1
        # u+2
        #
        # 正好 5 个像素。
        #

        x_min = max(0, u - 2)
        x_max = min(width, u + 3)

        y_min = max(0, v - 2)
        y_max = min(height, v + 3)

        depth_roi = depth_image[
            y_min:y_max,
            x_min:x_max
        ]

        # 转为 float，方便后面处理
        values = depth_roi.astype(
            np.float32
        ).reshape(-1)

        # 去掉 NaN 和 inf
        values = values[
            np.isfinite(values)
        ]

        if values.size == 0:
            return None

        # ------------------------------------------------------------
        # Aurora / ROS 深度图常见两种形式
        # ------------------------------------------------------------
        #
        # 16UC1：
        #
        #     uint16
        #     单位一般是 mm
        #
        # 例如：
        #
        #     435
        #
        # 表示：
        #
        #     435 mm
        #     0.435 m
        #
        #
        # 32FC1：
        #
        #     float32
        #     单位通常直接是 m
        #
        # 例如：
        #
        #     0.435
        #

        if (
            depth_encoding == '16UC1'
            or depth_encoding == 'mono16'
            or np.issubdtype(
                depth_image.dtype,
                np.integer
            )
        ):
            # 对当前 Aurora930 来说，
            # 官方代码也是按毫米处理 uint16 Depth。
            #
            # 这里过滤掉明显无效的数据。
            values = values[
                (values > 100.0)
                & (values < 4000.0)
            ]

            if values.size == 0:
                return None

            # 毫米 -> 米
            depth_m = (
                float(np.median(values))
                / 1000.0
            )

        else:
            # 32FC1 一般本身就是米
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
    # RGB + Depth 同步回调
    # ================================================================
    def image_callback(
        self,
        rgb_msg,
        depth_msg
    ):
        """
        每得到一组时间比较接近的 RGB 和 Depth，
        就执行一次。
        """

        try:
            # --------------------------------------------------------
            # ROS RGB Image -> OpenCV BGR
            # --------------------------------------------------------

            bgr_image = self.bridge.imgmsg_to_cv2(
                rgb_msg,
                desired_encoding='bgr8'
            )

            # --------------------------------------------------------
            # ROS Depth Image -> numpy
            # --------------------------------------------------------
            #
            # passthrough 非常重要。
            #
            # 我们不希望 CvBridge 自动改变深度图格式。
            #

            depth_image = self.bridge.imgmsg_to_cv2(
                depth_msg,
                desired_encoding='passthrough'
            )

        except Exception as e:
            self.get_logger().error(
                f'CvBridge conversion failed: {e}'
            )
            return

        # 有些情况下深度数组可能带一个多余维度
        depth_image = np.squeeze(depth_image)

        if depth_image.ndim != 2:
            self.get_logger().error(
                f'Unexpected depth image shape: '
                f'{depth_image.shape}'
            )
            return

        rgb_height, rgb_width = (
            bgr_image.shape[:2]
        )

        depth_height, depth_width = (
            depth_image.shape[:2]
        )

        # ------------------------------------------------------------
        # RGB 和 Depth 必须已经对齐
        # ------------------------------------------------------------
        #
        # 因为我们的逻辑是：
        #
        # RGB 中得到 (u,v)
        #
        # 然后直接：
        #
        # Depth[v,u]
        #
        # 如果 RGB 和 Depth 不是同一个坐标体系，
        # 这个操作没有意义。
        #
        # 官方 Aurora930 launch 中会开启 align_mode。
        #

        if (
            rgb_width != depth_width
            or rgb_height != depth_height
        ):
            self.get_logger().warning(
                'RGB and Depth resolutions are different. '
                'Please check Aurora930 align_mode. '
                f'RGB={rgb_width}x{rgb_height}, '
                f'Depth={depth_width}x{depth_height}'
            )
            return

        # ------------------------------------------------------------
        # 蓝色方块检测
        # ------------------------------------------------------------

        center, box, mask = (
            self.detect_blue_block(
                bgr_image
            )
        )

        # 调试图直接复制原始 RGB
        debug_image = bgr_image.copy()

        # 帧计数器
        self.frame_count += 1

        # ------------------------------------------------------------
        # 没找到蓝色方块
        # ------------------------------------------------------------

        if center is None:

            cv2.putText(
                debug_image,
                'Blue block: NOT FOUND',
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

            # 发布调试图
            self.publish_debug_image(
                debug_image,
                rgb_msg
            )

            return

        # ------------------------------------------------------------
        # 找到了蓝色方块
        # ------------------------------------------------------------

        u, v = center

        # 在图像上画方块轮廓
        cv2.drawContours(
            debug_image,
            [box],
            -1,
            (0, 255, 0),
            2
        )

        # 在中心画一个红点
        cv2.circle(
            debug_image,
            (u, v),
            5,
            (0, 0, 255),
            -1
        )

        # ------------------------------------------------------------
        # 获取深度
        # ------------------------------------------------------------

        depth_m = self.get_depth_at_target(
            depth_image,
            depth_msg.encoding,
            u,
            v
        )

        if depth_m is None:

            text = (
                f'u={u}, v={v}, '
                f'depth=INVALID'
            )

            cv2.putText(
                debug_image,
                text,
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        else:

            text = (
                f'u={u}, v={v}, '
                f'depth={depth_m:.3f} m'
            )

            cv2.putText(
                debug_image,
                text,
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

            # --------------------------------------------------------
            # 发布 [u, v, depth]
            # --------------------------------------------------------

            msg = Float32MultiArray()

            msg.data = [
                float(u),
                float(v),
                float(depth_m)
            ]

            self.target_pub.publish(msg)

            # 大约每 15 帧打印一次，
            # 避免终端刷屏。
            if self.frame_count % 15 == 0:

                self.get_logger().info(
                    f'Blue block: '
                    f'u={u}, '
                    f'v={v}, '
                    f'depth={depth_m:.3f} m'
                )

        # ------------------------------------------------------------
        # 把 5x5 Depth ROI 画出来
        # ------------------------------------------------------------
        #
        # 为了方便你直观看到程序到底在哪里读取深度。
        #

        cv2.rectangle(
            debug_image,
            (max(0, u - 2), max(0, v - 2)),
            (
                min(rgb_width - 1, u + 2),
                min(rgb_height - 1, v + 2)
            ),
            (255, 0, 255),
            1
        )

        self.publish_debug_image(
            debug_image,
            rgb_msg
        )


    # ================================================================
    # 发布调试图
    # ================================================================
    def publish_debug_image(
        self,
        image,
        source_msg
    ):

        try:
            debug_msg = (
                self.bridge.cv2_to_imgmsg(
                    image,
                    encoding='bgr8'
                )
            )

            # 保留原图时间戳和 frame_id
            debug_msg.header = (
                source_msg.header
            )

            self.debug_pub.publish(
                debug_msg
            )

        except Exception as e:
            self.get_logger().error(
                f'Failed to publish debug image: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = BlueBlockDepthNode()

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
