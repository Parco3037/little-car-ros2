"""Bounded calibrated servo sequences using physical readback, never joint cache."""
import math
import time

SERVO_IDS=(1,2,3,4,5,10)
PHASES=('approach','place','release','withdraw')


def validate_pose(p):
    if not isinstance(p,dict) or p.get('physical_readback_verified') is not True or not p.get('capture_evidence'):
        raise ValueError('Pose has no physical calibration evidence')
    positions=p.get('positions',{})
    if set(positions)!={str(v) for v in SERVO_IDS} or any(type(v) is not int or not 0<=v<=1000 for v in positions.values()):
        raise ValueError('Incomplete calibrated pulses')
    if type(p.get('duration_s')) not in (int,float) or not 3.<=p['duration_s']<=6.:
        raise ValueError('Pose duration must be 3..6 s')
    if type(p.get('tolerance_pulse')) is not int or not 1<=p['tolerance_pulse']<=15:
        raise ValueError('Invalid physical readback tolerance')
    return p


def validate_calibration(c,bundle_sha):
    if c.get('route_bundle_sha256')!=bundle_sha:raise ValueError('Calibration belongs to different route geometry')
    if c.get('transport_envelope_verified') is not True:raise ValueError('Transport envelope not verified')
    transport=validate_pose(c.get('transport'))
    if set(c.get('placements',{}))!={'1','2','3','4'}:raise ValueError('Four placement programs required')
    for slot,program in c['placements'].items():
        if set(program)!=set(PHASES):raise ValueError('Incomplete placement phases')
        for phase in PHASES:validate_pose(program[phase])
        closed=transport['positions']['10']
        if any(program[phase]['positions']['10']!=closed for phase in ('approach','place')):
            raise ValueError('Gripper must stay closed before release')
        if program['release']['positions']['10']==closed:raise ValueError('Release has no calibrated gripper change')
        if program['withdraw']['positions']['10']!=program['release']['positions']['10']:
            raise ValueError('Withdrawal must preserve calibrated release')
    if c.get('landing_verified')!={'pick_A':[1,2,3,4],'pick_B':[1,2,3,4]}:
        raise ValueError('Both parks need independently verified landing slots')
    return c


class ArmSequence:
    def __init__(self,node):
        from ros_robot_controller_msgs.srv import GetBusServoState
        from servo_controller_msgs.msg import ServosPosition
        from ros_robot_controller_msgs.msg import SetBusServoState
        from ros_robot_controller.ros_robot_controller_sdk import Board
        import inspect
        if 'RAICOM_BOUNDED_SERVO_READ_V1' not in inspect.getsource(Board.bus_servo_read_and_unpack):
            raise RuntimeError('Bounded physical servo read patch is not loaded')
        self.node=node;self.client=node.create_client(GetBusServoState,'/ros_robot_controller/bus_servo/get_state')
        self.publisher=node.create_publisher(ServosPosition,'/servo_controller',1)
        self.stopper=node.create_publisher(SetBusServoState,'/ros_robot_controller/bus_servo/set_state',1)
        self.state='idle';self.program=[];self.index=0;self.future=None;self.actual={};self.pass_number=0
        self.error='';self.sent=False

    def start(self,program):
        if self.state not in ('idle','complete','failed'):raise RuntimeError('Arm sequence pending')
        if not program:raise ValueError('Empty arm program')
        for _,p in program:validate_pose(p)
        self.program=program;self.index=0;self.state='send';self.error='';self.sent=False

    def fail(self,reason):
        from ros_robot_controller_msgs.msg import SetBusServoState,BusServoState
        self.state='failed';self.error=reason
        if self.future is not None:
            self.client.remove_pending_request(self.future);self.future.cancel();self.future=None
        if self.sent:
            self.stopper.publish(SetBusServoState(state=[BusServoState(present_id=[1,i],stop=[1]) for i in SERVO_IDS]))
        self.node.append({'kind':'arm_failed','reason':reason,'command_sent':self.sent})

    def poll(self):
        if self.state in ('idle','complete','failed'):return
        try:self._poll()
        except Exception as exc:self.fail(str(exc))

    def _poll(self):
        from ros_robot_controller_msgs.msg import GetBusServoCmd
        from ros_robot_controller_msgs.srv import GetBusServoState
        from servo_controller_msgs.msg import ServosPosition,ServoPosition
        now=time.monotonic()
        if not self.node.zero_velocity_hold_ok() or self.node.external_stop_requested:
            raise RuntimeError('Arm requires continuing chassis zero hold and no stop request')
        if self.node.heartbeat_at is None or now-self.node.heartbeat_at>.5:
            raise RuntimeError('Operator heartbeat lost during arm sequence')
        owners=self.node.get_publishers_info_by_topic('/servo_controller')
        if len(owners)!=1 or owners[0].node_name!=self.node.get_name():raise RuntimeError('Arm control has not been handed back exclusively')
        name,p=self.program[self.index]
        if self.state=='send':
            if (not self.client.service_is_ready() or self.publisher.get_subscription_count()!=1
                    or self.stopper.get_subscription_count()!=1):raise RuntimeError('Servo interfaces not ready')
            msg=ServosPosition(duration=float(p['duration_s']),position_unit='pulse')
            msg.position=[ServoPosition(id=i,position=float(p['positions'][str(i)])) for i in SERVO_IDS]
            self.publisher.publish(msg);self.sent=True;self.deadline=now+p['duration_s']+.6;self.state='moving'
            self.node.append({'kind':'arm_command','phase':name,'positions':p['positions'],'monotonic':now})
        elif self.state=='moving' and now>=self.deadline:
            self.state='reading';self.actual={};self.pass_number=0
        elif self.state=='reading':
            if self.future is None:
                sid=SERVO_IDS[len(self.actual)]
                self.future=self.client.call_async(GetBusServoState.Request(cmd=[GetBusServoCmd(id=sid,get_position=1)]))
                self.deadline=now+.8
            elif self.future.done():
                response=self.future.result();self.future=None;sid=SERVO_IDS[len(self.actual)]
                if (response is None or not response.success or len(response.state)!=1 or len(response.state[0].position)!=1):
                    raise RuntimeError('Missing physical servo response')
                actual=response.state[0].position[0]
                if not 0<=actual<=1000 or abs(actual-p['positions'][str(sid)])>p['tolerance_pulse']:
                    raise RuntimeError('Servo readback outside calibrated target: '+str(sid))
                self.actual[str(sid)]=actual
                if len(self.actual)==len(SERVO_IDS):
                    self.node.append({'kind':'servo_physical_readback','phase':name,'pass':self.pass_number+1,'positions':self.actual})
                    self.pass_number+=1
                    if self.pass_number==2:
                        self.index+=1;self.state='complete' if self.index==len(self.program) else 'send'
                    else:self.actual={}
            elif now>=self.deadline:raise RuntimeError('Servo physical readback timed out')
