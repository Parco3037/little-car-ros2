"""Nonblocking, bounded physical servo reads and explicitly requested motion."""
import inspect
import threading
import time

from .delivery_arm import SERVO_IDS, validate_pose
from .gripper_calibration import load_gripper_profile


class FieldArmIO:
    TERMINAL = {'idle', 'complete', 'failed'}

    def __init__(self, node, journal):
        from ros_robot_controller_msgs.srv import GetBusServoState
        from ros_robot_controller.ros_robot_controller_sdk import Board
        if 'RAICOM_BOUNDED_SERVO_READ_V1' not in inspect.getsource(Board.bus_servo_read_and_unpack):
            raise RuntimeError('缺少已验证的舵机物理读回超时保护')
        self.node, self.journal = node, journal
        self.client = node.create_client(GetBusServoState, '/ros_robot_controller/bus_servo/get_state')
        self.publisher = self.stopper = None
        self.state = 'idle'
        self.error = ''
        self.future = None
        self.positions = {}
        self.sent_ids = []
        self.grip_reference = None
        self.heartbeat = time.monotonic()
        self.lock = threading.RLock()
        self.watchdog = node.create_timer(.1, self._watchdog)

    @property
    def busy(self):
        return self.state not in self.TERMINAL

    def release_control(self):
        """Give the teammate exclusive command publishing ownership while parked."""
        with self.lock:
            if self.busy:
                raise RuntimeError('机械臂动作尚未结束，不能交还控制权')
            for name in ('publisher', 'stopper'):
                publisher = getattr(self, name)
                if publisher is not None:
                    self.node.destroy_publisher(publisher)
                    setattr(self, name, None)

    def _watchdog(self):
        with self.lock:
            if self.busy and (time.monotonic() - self.heartbeat > .7
                              or self.node.external_stop_requested
                              or not self.node.zero_velocity_hold_ok()):
                self.cancel('操作中断、界面失联或底盘未停稳')

    def start(self, kind, pose=None, *, holding_reference=None):
        from servo_controller_msgs.msg import ServosPosition
        from ros_robot_controller_msgs.msg import SetBusServoState
        with self.lock:
            if self.busy:
                raise RuntimeError('机械臂操作尚未结束')
            if not self.node.zero_velocity_hold_ok() or self.node.external_stop_requested:
                raise RuntimeError('请先停车，解除当前停止请求后重试')
            if kind not in {'capture', 'grip', 'verify_grip', 'pose', 'hold_pose', 'init_pose'}:
                raise ValueError('未知机械臂操作')
            if kind == 'init_pose':
                from .field_tool_motion_init import init_pose
                pose = init_pose()
                self.grip_reference = holding_reference
                if holding_reference is not None and (type(holding_reference) is not int or not 61 <= holding_reference <= 334):
                    raise ValueError('init运输夹持读回无效')
                self.gripper_profile = dict(close=50, open=350, tolerance=15)
            if kind in {'pose','hold_pose'}:
                validate_pose(pose)
            if kind in {'grip','verify_grip','hold_pose'}:
                self.gripper_profile = load_gripper_profile()
            if kind=='hold_pose':
                if type(holding_reference) is not int or not 0<=holding_reference<=1000:
                    raise ValueError('运输 / 落位需要本次夹稳时的真实夹爪读回')
                self.grip_reference=holding_reference
            if kind in {'pose', 'grip','hold_pose', 'init_pose'} and self.publisher is None:
                self.publisher = self.node.create_publisher(ServosPosition, '/servo_controller', 1)
                self.stopper = self.node.create_publisher(SetBusServoState, '/ros_robot_controller/bus_servo/set_state', 1)
            self.kind = kind
            self.ids = list(SERVO_IDS if kind in {'capture', 'pose','hold_pose','init_pose'} else (10,))
            self.target = (dict(pose['positions']) if kind in {'pose','hold_pose','init_pose'} else
                           {'10': self.gripper_profile['close']} if kind in {'grip', 'verify_grip'} else None)
            self.duration = float(pose['duration_s']) if kind in {'pose','hold_pose','init_pose'} else 3.
            self.tolerance = pose['tolerance_pulse'] if kind in {'pose','hold_pose','init_pose'} else 10
            self.positions, self.first_read, self.sent_ids = {}, {}, []
            self.error = ''
            self.pass_number = 0
            self.heartbeat = time.monotonic()
            self.deadline = self.heartbeat + 3.
            self.state = 'ready'
            self.journal({'kind': 'arm_begin', 'operation': kind})

    def cancel(self, reason='操作已取消'):
        from ros_robot_controller_msgs.msg import SetBusServoState, BusServoState
        with self.lock:
            if not self.busy:
                return
            self.state, self.error = 'failed', reason
            try:
                if self.future is not None:
                    self.client.remove_pending_request(self.future)
                    self.future.cancel()
            finally:
                self.future = None
                if self.sent_ids:
                    self.stopper.publish(SetBusServoState(state=[
                        BusServoState(present_id=[1, sid], stop=[1]) for sid in self.sent_ids]))
            try:
                self.journal({'kind': 'arm_failed', 'reason': reason, 'sent_ids': self.sent_ids})
            except OSError as exc:
                self.node.get_logger().error('Arm canceled; failed to append evidence: ' + str(exc))

    def poll(self):
        with self.lock:
            self.heartbeat = time.monotonic()
            if not self.busy:
                return
            try:
                self._poll()
            except Exception as exc:
                self.cancel(str(exc))

    def _poll(self):
        from ros_robot_controller_msgs.msg import GetBusServoCmd
        from ros_robot_controller_msgs.srv import GetBusServoState
        from servo_controller_msgs.msg import ServosPosition, ServoPosition
        now = time.monotonic()
        if self.node.external_stop_requested or not self.node.zero_velocity_hold_ok():
            raise RuntimeError('机械臂操作要求底盘持续停稳')
        if self.kind in {'pose', 'grip','hold_pose', 'init_pose'}:
            for topic in ('/servo_controller','/ros_robot_controller/bus_servo/set_state'):
                owners = self.node.get_publishers_info_by_topic(topic)
                if (len(owners) != 1 or owners[0].node_name != self.node.get_name()
                        or owners[0].node_namespace != self.node.get_namespace()):
                    raise RuntimeError('机械臂两个命令通道的控制权尚未独占交还')
        if self.state == 'ready':
            ready = self.client.service_is_ready()
            if self.kind in {'pose', 'grip','hold_pose', 'init_pose'}:
                ready = ready and self.publisher.get_subscription_count() == 1 and self.stopper.get_subscription_count() == 1
            if not ready:
                if now > self.deadline:
                    raise RuntimeError('舵机接口未就绪')
                return
            # Physical pre-read before sending; missing servo must never trigger motion.
            self.read_phase = 'before' if self.kind in {'pose', 'grip','hold_pose','init_pose'} else 'after'
            self.state = 'reading'
        elif self.state == 'moving':
            if now >= self.deadline:
                self.state = 'reading'
            return
        if self.state != 'reading':
            return
        if self.future is None:
            sid = self.ids[len(self.positions)]
            self.future = self.client.call_async(GetBusServoState.Request(cmd=[GetBusServoCmd(id=sid, get_position=1)]))
            self.deadline = now + .8
            return
        if not self.future.done():
            if now >= self.deadline:
                raise RuntimeError('舵机物理读回超时')
            return
        response = self.future.result()
        self.future = None
        sid = self.ids[len(self.positions)]
        if response is None or not response.success or len(response.state) != 1 or len(response.state[0].position) != 1:
            raise RuntimeError(f'舵机 {sid} 无有效物理读回')
        pulse = response.state[0].position[0]
        if type(pulse) is not int or not 0 <= pulse <= 1000:
            raise RuntimeError('舵机读回脉冲无效')
        init_holding = self.kind == 'init_pose' and self.grip_reference is not None
        holding_read = sid==10 and (self.kind in {'grip','verify_grip','hold_pose'} or init_holding)
        if holding_read and (self.read_phase=='after' or self.kind=='hold_pose' or init_holding):
            if abs(pulse-self.gripper_profile['open'])<=self.gripper_profile['tolerance']:
                raise RuntimeError('夹爪读回仍处于松开位置')
            if (self.kind in {'verify_grip','hold_pose'} or init_holding) and self.grip_reference is not None and abs(pulse-self.grip_reference)>self.gripper_profile['tolerance']:
                raise RuntimeError('夹爪位置在夹持期间发生变化')
        if self.read_phase == 'after' and self.target and not holding_read and abs(pulse - self.target[str(sid)]) > self.tolerance:
            raise RuntimeError(f'舵机 {sid} 未到位：实际 {pulse}')
        self.positions[str(sid)] = pulse
        if len(self.positions) != len(self.ids):
            return
        self.journal({'kind': 'servo_readback', 'operation': self.kind, 'phase': self.read_phase,
                      'pass': self.pass_number + 1, 'positions': dict(self.positions)})
        if self.read_phase == 'before':
            if self.kind=='hold_pose':self.grip_reference=self.positions['10']
            if self.kind == 'init_pose' and all(
                    (k == '10' and init_holding) or abs(v-self.target[k]) <= self.tolerance
                    for k, v in self.positions.items()):
                self.read_phase = 'after'
                self.positions = {}
                return  # Already init: verify twice without resending any servo command.
            msg = ServosPosition(duration=self.duration, position_unit='pulse')
            command_ids=[i for i in self.ids if not ((self.kind=='hold_pose' or init_holding) and i==10)]
            msg.position = [ServoPosition(id=i, position=float(self.target[str(i)])) for i in command_ids]
            self.sent_ids = command_ids
            self.publisher.publish(msg)
            self.journal({'kind': 'servo_command', 'positions': {str(i):self.target[str(i)] for i in command_ids}, 'duration_s': self.duration})
            self.state, self.deadline, self.read_phase = 'moving', now + self.duration + .6, 'after'
            self.positions = {}
            return
        self.pass_number += 1
        if self.pass_number == 1:
            self.first_read, self.positions = dict(self.positions), {}
        else:
            if any(abs(self.positions[k] - self.first_read[k]) > 10 for k in self.positions):
                raise RuntimeError('两次物理读回不稳定，请保持机械臂静止后重试')
            self.state = 'complete'
            if self.kind in {'grip','verify_grip','hold_pose'}:self.grip_reference=self.positions['10']

    def captured_pose(self, evidence):
        if self.state != 'complete' or self.kind != 'capture':
            raise RuntimeError('当前没有完整姿态读回')
        return {'positions': dict(self.positions), 'duration_s': 4., 'tolerance_pulse': 10,
                'physical_readback_verified': True, 'capture_evidence': str(evidence)}
