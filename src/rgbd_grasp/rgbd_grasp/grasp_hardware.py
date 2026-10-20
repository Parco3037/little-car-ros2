#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
grasp_hardware.py

RGB-D 视觉抓取使用的机械臂硬件访问层。

主要功能：
1. 调用幻尔官方 IK 服务
2. 向 /servo_controller 发布舵机位置命令
3. 直接通过 ros_robot_controller 查询真实总线舵机位置
4. 等待舵机真正到位并稳定
5. 提供底层 stop
6. 对偶发的总线读取失败进行有限重试

设计原则：
- fail closed：连续读取失败就停止抓取
- 不使用 /controller_manager/servo_states 代替真实硬件反馈
- 普通抓取过程中禁止修改 ID5
- 夹取后收拢到已配置的保持姿态时，才允许 ID5 移动
"""

import time

import numpy as np

from kinematics_msgs.srv import SetRobotPose, SetJointValue

from ros_robot_controller_msgs.msg import (
    BusServoState,
    GetBusServoCmd,
    SetBusServoState,
)

from ros_robot_controller_msgs.srv import (
    GetBusServoState,
)

from servo_controller_msgs.msg import (
    ServoPosition,
    ServosPosition,
)

from .grasp_core import (
    Rejected,
    validate_arm_pulses,
)


class GraspHardware:
    """
    抓取节点与真实机械臂硬件之间的适配层。

    舵机：

        ID1~ID4：
            机械臂主要运动关节

        ID5：
            搜索/接近阶段保持原位，夹取后收拢到 fixed_id5

        ID10：
            夹爪
    """

    IDS = (1, 2, 3, 4, 5, 10)

    # 单个舵机读取失败后的最大尝试次数
    READ_RETRIES = 3

    # 两次读取重试之间等待时间
    READ_RETRY_DELAY_S = 0.08

    # wait() 中硬件位置轮询周期
    WAIT_POLL_INTERVAL_S = 0.10

    def __init__(self, node, cancelled):

        self.node = node

        # cancelled 是一个回调函数：
        #
        # lambda: self.cancel_event.is_set()
        #
        # 如果 stop 服务被调用，
        # 后续动作都应该立即退出。
        self.cancelled = cancelled
        self.command_end = 0.0
        self.last_settle_measurement = None

        # ============================================================
        # 底层舵机状态读取服务
        # ============================================================

        self.read_client = node.create_client(
            GetBusServoState,
            '/ros_robot_controller/bus_servo/get_state',
            callback_group=node.callback_group,
        )

        # ============================================================
        # 幻尔官方 IK 服务
        # ============================================================

        self.ik_client = node.create_client(
            SetRobotPose,
            '/kinematics/set_pose_target',
            callback_group=node.callback_group,
        )
        self.fk_client = node.create_client(
            SetJointValue, '/kinematics/set_joint_value_target',
            callback_group=node.callback_group,
        )

        # ============================================================
        # 舵机控制
        # ============================================================
        #
        # controller_manager 订阅 /servo_controller，
        # 再将 pulse 命令送到真实舵机。
        #

        self.command_pub = node.create_publisher(
            ServosPosition,
            '/servo_controller',
            1,
        ) if self.p['motion_enabled'] else None

        # ============================================================
        # 底层 stop
        # ============================================================

        self.stop_pub = node.create_publisher(
            SetBusServoState,
            '/ros_robot_controller/bus_servo/set_state',
            1,
        ) if self.p['motion_enabled'] else None

    # ================================================================
    # 参数快捷访问
    # ================================================================

    @property
    def p(self):
        return self.node.parameters

    # ================================================================
    # Stop 检查
    # ================================================================

    def guard(self):
        """
        每一个可能执行较长时间的动作之前都检查 stop。

        如果用户调用：

            /grasp_once_node/stop

        cancel_event 会被设置，
        所有后续阶段都会抛出 Rejected。
        """

        if self.cancelled():
            raise Rejected(
                'stopped by operator'
            )

    # ================================================================
    # 通用 ROS Service 调用
    # ================================================================

    def call(self, client, request):
        """
        同步等待一个异步 ROS2 Service。

        注意：
        grasp_once 使用 MultiThreadedExecutor，
        所以这里等待 future 时其他 executor 线程仍然可以处理响应。
        """

        self.guard()

        # 服务必须存在
        if not client.wait_for_service(
            timeout_sec=0.25
        ):
            raise Rejected(
                'required service unavailable: '
                + client.srv_name
            )

        future = client.call_async(
            request
        )

        deadline = (
            time.monotonic()
            + float(
                self.p['service_timeout_s']
            )
        )

        while not future.done():

            self.guard()

            if time.monotonic() >= deadline:

                future.cancel()

                raise Rejected(
                    'service timeout: '
                    + client.srv_name
                )

            # 不要忙等
            time.sleep(0.01)

        response = future.result()

        if response is None:

            raise Rejected(
                'service returned no response: '
                + client.srv_name
            )

        if not response.success:

            raise Rejected(
                'service failed: '
                + client.srv_name
            )

        return response

    # ================================================================
    # 单个舵机真实位置读取
    # ================================================================

    def _read_one_servo(self, servo_id):
        """
        直接从 ros_robot_controller 查询一个舵机。

        为什么不一次把 6 个舵机都塞进同一个请求？

        你已经实测发现，一次请求多个舵机时可能返回：

            success=False
            position=[]

        对半双工总线舵机来说，逐个请求更容易定位问题，
        而且某一个舵机通信失败时可以单独重试。
        """

        servo_id = int(servo_id)

        last_error = None

        for attempt in range(
            1,
            self.READ_RETRIES + 1
        ):

            self.guard()

            try:

                request = (
                    GetBusServoState.Request()
                )

                command = GetBusServoCmd()

                command.id = servo_id

                # 只读取 position
                command.get_position = 1

                request.cmd.append(
                    command
                )

                response = self.call(
                    self.read_client,
                    request
                )

                # 单个请求理论上应该返回一个 state
                if len(response.state) != 1:

                    raise Rejected(
                        f'ID{servo_id} hardware response '
                        f'count={len(response.state)}, '
                        f'expected 1'
                    )

                state = response.state[0]

                # position 是数组
                if len(state.position) != 1:

                    raise Rejected(
                        f'ID{servo_id} hardware '
                        f'position is missing'
                    )

                pulse = int(
                    state.position[0]
                )

                # 总线舵机 pulse 合法范围
                if not 0 <= pulse <= 1000:

                    raise Rejected(
                        f'ID{servo_id} returned '
                        f'invalid pulse {pulse}'
                    )

                # 成功
                return pulse

            except Rejected as error:

                last_error = error

                self.node.get_logger().warning(
                    f'ID{servo_id} read '
                    f'attempt {attempt}/'
                    f'{self.READ_RETRIES} failed: '
                    f'{error}'
                )

                # 最后一次就不用再 sleep
                if attempt < self.READ_RETRIES:

                    time.sleep(
                        self.READ_RETRY_DELAY_S
                    )

        # 三次仍然失败
        raise Rejected(
            f'ID{servo_id} hardware read '
            f'failed after '
            f'{self.READ_RETRIES} attempts: '
            f'{last_error}'
        )

    # ================================================================
    # 读取舵机位置
    # ================================================================

    def read(self, ids=None):
        """
        读取真实硬件位置。

        ids=None：
            默认读取 ID1、2、3、4、5、10。

        也可以只读取指定舵机，例如：

            read((1, 2, 3, 4))

        返回：

            {
                1: 450,
                2: 794,
                3: 0,
                4: 292,
                5: 874,
                10: 298
            }

        这里故意逐个读取，而不是一次发送六个 GetBusServoCmd。
        """

        self.guard()

        if ids is None:

            ids = self.IDS

        # 去重，同时保持传入顺序
        requested_ids = []

        for servo_id in ids:

            servo_id = int(servo_id)

            if servo_id not in self.IDS:

                raise Rejected(
                    f'unexpected servo ID '
                    f'{servo_id} in hardware read'
                )

            if servo_id not in requested_ids:

                requested_ids.append(
                    servo_id
                )

        if not requested_ids:

            raise Rejected(
                'empty servo read request'
            )

        result = {}

        for servo_id in requested_ids:

            result[servo_id] = (
                self._read_one_servo(
                    servo_id
                )
            )

        return result

    # ================================================================
    # ID5 固定姿态检查
    # ================================================================

    def assert_fixed_id5(
        self,
        measured
    ):
        """
        当前抓取设计要求 ID5 始终保持：

            fixed_id5 = 875

        如果真实位置偏离过多，
        不允许继续抓取。
        """

        if 5 not in measured:

            raise Rejected(
                'ID5 is missing from '
                'hardware measurement'
            )

        expected = int(
            self.p['fixed_id5']
        )

        tolerance = int(
            self.p['id5_tolerance']
        )

        actual = int(
            measured[5]
        )

        if abs(actual - expected) > tolerance:

            raise Rejected(
                'ID5 is not at the fixed '
                'initial posture: '
                f'actual={actual}, '
                f'expected={expected}, '
                f'tolerance={tolerance}'
            )

    # ================================================================
    # IK
    # ================================================================

    def ik(
        self,
        position
    ):
        """
        调用幻尔官方：

            /kinematics/set_pose_target

        position 必须已经是幻尔 IK world 坐标。

        注意：
        这里绝对不能再传 ROS base_link XYZ。
        """

        self.guard()

        position = np.asarray(
            position,
            dtype=float
        )

        if (
            position.shape != (3,)
            or not np.all(
                np.isfinite(position)
            )
        ):

            raise Rejected(
                'invalid IK position'
            )

        request = SetRobotPose.Request()

        request.position = [
            float(value)
            for value in position
        ]

        request.pitch = float(
            self.p['pitch_deg']
        )

        request.pitch_range = [
            float(value)
            for value
            in self.p[
                'pitch_range_deg'
            ]
        ]

        request.resolution = 1.0

        response = self.call(
            self.ik_client,
            request
        )

        # 幻尔的 service 即使没有 IK 解，
        # response.success 仍可能是 True，
        # 但 pulse 为空。
        if len(response.pulse) != 5:

            raise Rejected(
                'IK has no complete solution'
            )

        # 进一步验证：
        #
        # 舵机范围
        # ID1 范围
        # ID5 固定值
        pulses = validate_arm_pulses(
            response.pulse,
            self.p['servo_limits'],
            self.p['id1_range'],
            self.p['fixed_id5'],
        )
        xyz = self.fk(pulses)
        error = float(np.linalg.norm(xyz - np.asarray(position)))
        if not np.isfinite(error) or error > 0.005:
            raise Rejected(f'IK/FK position error exceeds 5mm: {error:.4f}m')
        return pulses

    def fk(self, pulses):
        """Read the rigid-model endpoint without commanding any servo."""
        request = SetJointValue.Request()
        request.joint_value = [float(p) for p in pulses]
        check = self.call(self.fk_client, request)
        if not check.solution:
            raise Rejected('IK solution has no FK result')
        xyz = np.array([check.pose.position.x, check.pose.position.y,
                        check.pose.position.z])
        if not np.all(np.isfinite(xyz)):
            raise Rejected('FK returned non-finite position')
        return xyz

    # ================================================================
    # 发送舵机位置
    # ================================================================

    def command(
        self,
        values,
        duration=None,
        allow_id5=False,
        post_grasp_hold=False,
    ):
        """
        values 示例：

            {
                1: 450,
                2: 800,
                3: 0,
                4: 300
            }

        回位、搜索、接近阶段禁止修改 ID5。
        post_grasp_hold=True 仅允许夹取后的完整 carry_pose（ID1~5），
        且不得包含夹爪 ID10。allow_id5 为旧接口保留，不解除限制。
        """

        self.guard()

        if not self.p['motion_enabled']:

            raise Rejected(
                'motion disabled; '
                'precheck only'
            )

        self.node.assert_single_command_source()

        if not values:

            raise Rejected(
                'empty servo command'
            )

        # ============================================================
        # 检查舵机 ID
        # ============================================================

        allowed_ids = {
            1,
            2,
            3,
            4,
            10,
        }

        # Wrist motion is authorized only for the requested post-grasp pose.
        # Preserve ID10's existing close command throughout this transition.
        if post_grasp_hold:
            if not self.node.holding:
                raise Rejected('post-grasp hold requires a retained grasp')
            carry = validate_arm_pulses(
                self.p['carry_pose'], self.p['servo_limits'],
                self.p['id1_range'], self.p['fixed_id5'],
            )
            if (set(values) != {1, 2, 3, 4, 5}
                    or any(float(values[sid]) != float(carry[sid - 1])
                           for sid in (1, 2, 3, 4, 5))):
                raise Rejected('post-grasp hold must match carry_pose and exclude ID10')
            allowed_ids = {1, 2, 3, 4, 5}

        for servo_id in values:

            servo_id = int(
                servo_id
            )

            if servo_id not in allowed_ids:

                if servo_id == 5:

                    raise Rejected(
                        'ID5 command is forbidden '
                        'outside post-grasp hold'
                    )

                raise Rejected(
                    f'unexpected servo command '
                    f'ID{servo_id}'
                )

        # ============================================================
        # duration
        # ============================================================

        if duration is None:

            command_duration = float(
                self.p[
                    'motion_duration_s'
                ]
            )

        else:

            command_duration = float(
                duration
            )

        if (
            not np.isfinite(
                command_duration
            )
            or command_duration <= 0.0
            or command_duration > 30.0
        ):

            raise Rejected(
                'invalid servo command duration'
            )

        # ============================================================
        # 构造 ServosPosition
        # ============================================================

        message = ServosPosition()

        message.position_unit = 'pulse'

        message.duration = (
            command_duration
        )

        for servo_id, pulse in (
            values.items()
        ):

            servo_id = int(
                servo_id
            )

            pulse = int(
                pulse
            )

            if not 0 <= pulse <= 1000:

                raise Rejected(
                    f'ID{servo_id} command pulse '
                    f'outside [0,1000]: '
                    f'{pulse}'
                )

            item = ServoPosition()

            item.id = servo_id

            item.position = float(
                pulse
            )

            message.position.append(
                item
            )

        self.command_pub.publish(
            message
        )
        self.command_end = time.monotonic() + command_duration

    # ================================================================
    # 机械臂 ID1~4 命令
    # ================================================================

    def command_arm(
        self,
        pulses
    ):
        """
        IK 返回五个 pulse：

            ID1
            ID2
            ID3
            ID4
            ID5

        此接近运动接口不改变 ID5，
        所以实际抓取运动只控制 ID1~4。
        """

        pulses = np.asarray(
            pulses,
            dtype=float
        )

        if (
            pulses.shape != (5,)
            or not np.all(
                np.isfinite(pulses)
            )
        ):

            raise Rejected(
                'invalid arm pulse command'
            )

        self.command({
            1: int(pulses[0]),
            2: int(pulses[1]),
            3: int(pulses[2]),
            4: int(pulses[3]),
        })

    # ================================================================
    # 等待硬件到位
    # ================================================================

    def wait(
        self,
        ids,
        goals=None,
        enforce_id5=True,
        tolerance=None,
    ):
        """
        等待指定舵机到位并稳定。

        例如：

            wait(
                (1,2,3,4),
                [450,700,200,300]
            )

        goals=None：

            不要求到达具体 pulse，
            只要求舵机位置停止变化。

        这用于夹爪闭合非常合适：

            ID10 可能因为夹住方块，
            永远无法真正到 110。

        enforce_id5=True：

            普通抓取期间同时检查
            ID5 是否仍然接近配置的 fixed_id5。
            收拢等待还会将 ID5 纳入目标到位与稳定检查。
        """

        self.guard()

        ids = tuple(
            int(servo_id)
            for servo_id in ids
        )

        if len(ids) == 0:

            raise Rejected(
                'empty wait servo list'
            )

        for servo_id in ids:

            if servo_id not in self.IDS:

                raise Rejected(
                    f'unexpected wait servo '
                    f'ID{servo_id}'
                )

        # ============================================================
        # 目标位置
        # ============================================================

        if goals is not None:

            goal_array = np.asarray(
                goals,
                dtype=float
            )

            if (
                goal_array.shape
                != (len(ids),)
                or not np.all(
                    np.isfinite(
                        goal_array
                    )
                )
            ):

                raise Rejected(
                    'invalid wait goals'
                )

        else:

            goal_array = None

        # ============================================================
        # 只读取真正需要的舵机
        # ============================================================
        #
        # 原来每 50ms 都读 6 个舵机，
        # 对串口总线压力太大。
        #
        # 现在：
        #
        # 等 ID1~4：
        #   只读 1,2,3,4,5
        #
        # 等夹爪：
        #   只读 10,5
        #

        read_ids = list(ids)

        if (
            enforce_id5
            and 5 not in read_ids
        ):

            read_ids.append(5)

        # Goal tolerance describes servo accuracy, not stopped motion. Do not
        # accept a slow-moving arm before the commanded duration has elapsed.
        while time.monotonic() < self.command_end:
            self.guard()
            time.sleep(min(0.02, max(0.0, self.command_end - time.monotonic())))

        deadline = (
            time.monotonic()
            + float(
                self.p[
                    'settle_timeout_s'
                ]
            )
        )

        previous = None

        stable = 0

        if tolerance is None:
            tolerance = float(
                self.p[
                    'settle_tolerance'
                ]
            )
        else:
            tolerance = float(tolerance)

        if tolerance < 0:
            raise Rejected('wait tolerance must be non-negative')

        required_samples = int(
            self.p[
                'stable_joint_samples'
            ]
        )

        if required_samples < 1:

            raise Rejected(
                'stable_joint_samples '
                'must be >= 1'
            )

        # ============================================================
        # 循环检查
        # ============================================================

        while (
            time.monotonic()
            < deadline
        ):

            self.guard()

            measured = self.read(
                read_ids
            )
            self.last_settle_measurement = dict(measured)

            if enforce_id5:

                self.assert_fixed_id5(
                    measured
                )

            values = np.asarray(
                [
                    measured[servo_id]
                    for servo_id in ids
                ],
                dtype=float,
            )

            # --------------------------------------------
            # 是否接近目标
            # --------------------------------------------

            if goal_array is None:

                # 夹爪抓物体时允许停在任意位置
                close = True

            else:

                close = (
                    np.max(
                        np.abs(
                            values
                            - goal_array
                        )
                    )
                    <= tolerance
                )

            # --------------------------------------------
            # 是否已经停止变化
            # --------------------------------------------

            if previous is None:

                unchanged = False

            else:

                unchanged = (
                    np.max(
                        np.abs(
                            values
                            - previous
                        )
                    )
                    <= min(2.0, tolerance)
                )

            if close and unchanged:

                stable += 1

            else:

                stable = 0

            if stable >= required_samples:

                return measured

            previous = values

            # 以前 0.05 秒查询一次，
            # 现在改成 0.10 秒降低总线压力。
            time.sleep(
                self.WAIT_POLL_INTERVAL_S
            )

        raise Rejected(
            'hardware settling timeout: '
            f'ids={ids}, goals={goals}, measured={self.last_settle_measurement}, '
            f'tolerance={tolerance}, stable_samples={stable}/{required_samples}'
        )

    # ================================================================
    # 底层 Stop
    # ================================================================

    def stop(self):
        """
        向真实总线舵机发送 stop。

        stop 不会主动打开夹爪，
        所以如果当前正在夹住物体，
        stop 后仍然保持当前夹爪状态。
        """

        if self.stop_pub is None:
            return
        message = SetBusServoState()

        for servo_id in self.IDS:

            state = BusServoState()

            # 官方协议：
            #
            # present_id[0] = 是否设置
            # present_id[1] = servo ID
            state.present_id = [
                1,
                int(servo_id),
            ]

            # stop[0] = 是否执行 stop
            state.stop = [1]

            message.state.append(
                state
            )

        self.stop_pub.publish(
            message
        )
