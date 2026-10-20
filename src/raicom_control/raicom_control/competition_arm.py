"""Competition-only arm driver. Startup has no actuator publishers or commands."""
import inspect
import time

IDS = (1, 2, 3, 4, 5, 10)
TOPICS = ('/servo_controller', '/ros_robot_controller/bus_servo/set_state')


class PoseNotReached(RuntimeError):
    pass


class CompetitionArm:
    def __init__(self, node, calibration, journal, clock=time.monotonic):
        from ros_robot_controller_msgs.srv import GetBusServoState
        from ros_robot_controller.ros_robot_controller_sdk import Board
        if 'RAICOM_BOUNDED_SERVO_READ_V1' not in inspect.getsource(Board.bus_servo_read_and_unpack):
            raise RuntimeError('当前舵机驱动缺少物理读回超时保护')
        self.node, self.calibration, self.journal, self.clock = node, calibration, journal, clock
        self.client = node.create_client(GetBusServoState, '/ros_robot_controller/bus_servo/get_state')
        self.publisher = self.stopper = self.future = None
        self.state, self.error = 'idle', ''
        self.sent_ids, self.positions, self.reads, self.completed = [], {}, [], []
        self.reference = None

    @property
    def busy(self): return self.state not in ('idle', 'complete', 'failed')

    def start(self, entry, program, *, placement_candidates=None):
        if self.busy or self.publisher is not None: raise RuntimeError('机械臂控制权尚未释放')
        if any(self.node.get_publishers_info_by_topic(t) for t in TOPICS):
            raise RuntimeError('机械臂仍被其他程序占用')
        if not self.node.safe_to_operate(): raise RuntimeError('底盘、定位或本轮租约未就绪')
        if not self.calibration.unchanged(): raise RuntimeError('动作标定文件已改变')
        self.entry, self.program = entry, list(program)
        self.placement_candidates = list(placement_candidates or [])
        self.placement_candidate = 0
        self.placement_attempt = 0
        self.placement_failures = []
        self.index, self.completed, self.sent_ids = 0, [], []
        self.init_command_completed = False
        self.readback_steps = []
        self.reference, self.error = None, ''
        if program:
            from servo_controller_msgs.msg import ServosPosition
            from ros_robot_controller_msgs.msg import SetBusServoState
            self.publisher = self.node.create_publisher(ServosPosition, TOPICS[0], 1)
            self.stopper = self.node.create_publisher(SetBusServoState, TOPICS[1], 1)
        self.state, self.deadline = 'ready', self.clock()+3
        self.positions, self.reads = {}, []
        self.read_phase = 'entry'

    def fail(self, reason):
        if not self.busy: return
        self.error = str(reason)
        if self.future is not None:
            self.client.remove_pending_request(self.future); self.future.cancel(); self.future = None
        if self.sent_ids and self.stopper is not None:
            from ros_robot_controller_msgs.msg import SetBusServoState, BusServoState
            self.stopper.publish(SetBusServoState(state=[BusServoState(present_id=[1,sid],stop=[1]) for sid in self.sent_ids]))
            self.state, self.positions, self.reads = 'stopping', {}, []
            self.stop_read_at, self.stop_retry_at = self.clock(), self.clock()
        else:
            self.state = 'failed'
        self.journal(dict(kind='arm_cancel', reason=self.error, stopped_ids=list(self.sent_ids)))

    def release(self):
        if self.busy: raise RuntimeError('机械臂操作尚未停止')
        for key in ('publisher','stopper'):
            publisher = getattr(self,key)
            if publisher is not None: self.node.destroy_publisher(publisher); setattr(self,key,None)

    def poll(self):
        if not self.busy: return
        try:
            if self.state == 'stopping': self._poll_stop()
            else: self._poll()
        except Exception as exc: self.fail(exc)

    def _poll_stop(self):
        """Cancel keeps command ownership until physical motion has stopped."""
        now = self.clock()
        if now < self.stop_retry_at: return
        from ros_robot_controller_msgs.msg import GetBusServoCmd
        from ros_robot_controller_msgs.srv import GetBusServoState
        if self.future is None:
            if not self.client.service_is_ready():
                self.stop_retry_at = now+.5; return
            sid = IDS[len(self.positions)]
            self.future = self.client.call_async(GetBusServoState.Request(cmd=[GetBusServoCmd(id=sid,get_position=1)]))
            self.deadline = now+.8; return
        if not self.future.done():
            if now >= self.deadline:
                self.client.remove_pending_request(self.future); self.future.cancel(); self.future = None
                self.positions, self.reads, self.stop_retry_at = {}, [], now+.5
            return
        result = self.future.result(); self.future = None
        if (result is None or not result.success or len(result.state) != 1 or len(result.state[0].position) != 1
                or type(result.state[0].position[0]) is not int or not 0 <= result.state[0].position[0] <= 1000):
            self.positions, self.reads, self.stop_retry_at = {}, [], now+.5; return
        self.positions[str(IDS[len(self.positions)])] = result.state[0].position[0]
        if len(self.positions) != len(IDS): return
        self.reads.append(dict(self.positions))
        self.journal(dict(kind='cancel_readback',positions=dict(self.positions)))
        if len(self.reads) >= 2 and all(abs(self.reads[-1][k]-self.reads[-2][k]) <= 3 for k in self.positions):
            self.state = 'failed'
        else:
            self.positions = {}; self.stop_retry_at = now+.3

    def _poll(self):
        now = self.clock()
        if not self.node.safe_to_operate(): raise RuntimeError('底盘/定位失效、急停或本轮租约丢失')
        for topic in TOPICS:
            owners = self.node.get_publishers_info_by_topic(topic)
            if self.program:
                if (len(owners) != 1 or owners[0].node_name != self.node.get_name()
                        or owners[0].node_namespace != self.node.get_namespace()):
                    raise RuntimeError('机械臂控制通道出现其他发布者')
            elif owners: raise RuntimeError('物理采集期间机械臂被占用')
        if self.state == 'ready':
            ready = self.client.service_is_ready()
            if self.program: ready = ready and all(p.get_subscription_count() == 1 for p in (self.publisher,self.stopper))
            if not ready:
                if now > self.deadline: raise RuntimeError('舵机通道未就绪')
                return
            self.state = 'reading'
        if self.state == 'moving':
            if now < self.deadline: return
            self.state = 'reading'
        if self.state != 'reading': return
        if self.future is None:
            from ros_robot_controller_msgs.msg import GetBusServoCmd
            from ros_robot_controller_msgs.srv import GetBusServoState
            sid = self._read_ids()[len(self.positions)]
            self.future = self.client.call_async(GetBusServoState.Request(cmd=[GetBusServoCmd(id=sid,get_position=1)]))
            self.deadline = now+.8
            return
        if not self.future.done():
            if now >= self.deadline: raise RuntimeError('舵机物理读回超时')
            return
        response = self.future.result(); self.future = None
        sid = self._read_ids()[len(self.positions)]
        if response is None or not response.success or len(response.state) != 1 or len(response.state[0].position) != 1:
            raise RuntimeError(f'舵机{sid}没有有效物理读回')
        value = response.state[0].position[0]
        if type(value) is not int or not 0 <= value <= 1000: raise RuntimeError('舵机脉冲无效')
        self.positions[str(sid)] = value
        if len(self.positions) != len(self._read_ids()): return
        try:
            self._check_positions()
        except PoseNotReached as exc:
            if not self.placement_candidates or self.read_phase != 'after' or self.program[self.index][0]!='place':
                raise
            # Retry only a settled, nonempty grasp at the park. Read failures,
            # cancellation and lost grip never fall through to another action.
            self.reads.append(dict(self.positions))
            if len(self.reads)<2:
                self.positions={};return
            if any(abs(self.reads[-1][k]-self.reads[-2][k])>3 for k in self.positions):
                raise RuntimeError('未到位且仍在运动，不能重新发送放置动作')
            self._retry_placement(now,str(exc));return
        self.reads.append(dict(self.positions))
        self.journal(dict(kind='physical_readback', phase=self.read_phase,
            step=self.index, positions=dict(self.positions), pass_number=len(self.reads)))
        if len(self.reads) < 2:
            self.positions = {}; return
        if any(abs(self.reads[0][k]-self.reads[1][k]) > 3 for k in self.positions):
            raise RuntimeError('两次舵机物理读回未稳定')
        if self.read_phase == 'entry':
            self.reference = self.positions['10']
            if not self.program: self.state = 'complete'; return
        else:
            if self._init_step():
                self.init_command_completed = True
                self.journal(dict(kind='init_command_completed', step=self.program[self.index][0],
                                  physical_pose_verified=False, power_kept_on=True))
            else:
                self.readback_steps.append(self.program[self.index][0])
            self.completed.append(self.program[self.index][0]); self.index += 1
            if self.index == len(self.program): self.state = 'complete'; return
        self._send(now)

    def _init_step(self):
        return (getattr(self.calibration, 'command_wrist', False) and self.read_phase == 'after'
                and self.program[self.index][0] in ('transport', 'withdraw'))

    def _read_ids(self):
        # Named init is a timed return, not another joint-position admission gate.
        # Keep the gripper check so an empty/lost load is never called held.
        if getattr(self.calibration, 'command_wrist', False) and (self.read_phase == 'entry' or self._init_step()):
            return (10,)
        return IDS

    def _check_positions(self):
        c, actual = self.calibration.value, self.positions
        wrist, grip = c['fixed_wrist'], c['gripper']
        if not getattr(self.calibration,'command_wrist',False) and abs(actual['5']-wrist['position']) > wrist['tolerance']:
            raise RuntimeError('5号固定腕部位置改变，禁止继续')
        pose, mode = (self.entry,'holding') if self.read_phase == 'entry' else self.program[self.index][1:]
        joint_ids=('1','2','3','4','5') if getattr(self.calibration,'command_wrist',False) else ('1','2','3','4')
        if mode == 'holding':
            if not grip['holding_min'] <= actual['10'] <= grip['holding_max']:
                raise RuntimeError('夹爪读回接近空夹或张开位置，保持状态未通过')
            reclosing = (getattr(self.calibration,'command_wrist',False) and self.read_phase=='after'
                         and self.program[self.index][0]=='transport')
            if not reclosing and self.reference is not None and abs(actual['10']-self.reference) > grip['tolerance']:
                raise RuntimeError('夹爪在运输/落位期间发生变化')
        elif mode == 'recognition':
            # 50 is the close command, not an attainable encoder position when
            # a block resists closure. Verify the measured grasp is nonempty.
            if not grip['holding_min'] <= actual['10'] <= grip['holding_max']:
                raise RuntimeError('识别姿态夹爪读回接近空夹或张开位置')
        elif abs(actual['10']-grip['open']) > grip['tolerance']:
            raise RuntimeError('夹爪尚未达到实测松开位置')
        if self._read_ids() == (10,):
            return
        if any(abs(actual[k]-pose['positions'][k]) > pose['tolerance_pulse'] for k in joint_ids):
            raise PoseNotReached('机械臂未到标定姿态或起点与标定路径不同')

    def _retry_placement(self, now, reason):
        current=self.placement_candidates[self.placement_candidate]
        event=dict(kind='placement_attempt_failed',profile_id=current['profile_id'],slot=current['slot'],
                   attempt=self.placement_attempt,limit=20,reason=reason,positions=dict(self.positions))
        self.placement_failures.append(event);self.journal(event)
        if self.placement_attempt>=20:
            self.placement_candidate+=1
            if self.placement_candidate>=len(self.placement_candidates):
                raise RuntimeError('所有未使用放置姿态均已尝试20次；仍夹持，未松爪或虚报完成')
            alternative=self.placement_candidates[self.placement_candidate]
            # The initial approach is already complete. Change the remaining
            # placement/release/return-init targets together, keeping ID10 closed.
            self.program[self.index:]=alternative['program'][1:]
            self.placement_attempt=0
            self.journal(dict(kind='placement_next_pose',profile_id=alternative['profile_id'],slot=alternative['slot']))
        self._send(now)

    def _send(self, now):
        from servo_controller_msgs.msg import ServosPosition, ServoPosition
        name, pose, mode = self.program[self.index]
        if name == 'approach' and getattr(self.calibration, 'command_wrist', False):
            # Transport already returned to init. Do not send that same pose again
            # at the park; the next command is the selected placement posture.
            self.completed.append(name)
            self.journal(dict(kind='approach_kept_init', command_sent=False))
            self.index += 1
            return self._send(now)
        if name=='place':self.placement_attempt+=1
        ids = [10] if mode == 'release' else ([1,2,3,4,5] if getattr(self.calibration,'command_wrist',False) else [1,2,3,4])
        if getattr(self.calibration,'command_wrist',False) and name in ('recognition','transport'):
            ids = [1,2,3,4,5,10]
        # Finger 10 keeps its existing command throughout transport and approach.
        # Named targets explicitly include wrist 5; legacy measured programs keep it fixed.
        self.journal(dict(kind='arm_command', phase=name, positions={str(i):pose['positions'][str(i)] for i in ids},
                          duration_s=pose['duration_s']))
        self.sent_ids = sorted(set(self.sent_ids+ids))
        self.publisher.publish(ServosPosition(duration=float(pose['duration_s']),position_unit='pulse',
            position=[ServoPosition(id=i,position=float(pose['positions'][str(i)])) for i in ids]))
        self.state, self.deadline, self.read_phase = 'moving', now+pose['duration_s']+.6, 'after'
        self.positions, self.reads = {}, []
