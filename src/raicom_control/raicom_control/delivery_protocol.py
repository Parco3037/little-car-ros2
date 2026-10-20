"""String+JSON pick handshake and eight-item delivery state, without ROS I/O."""
import copy
import math
import uuid

IDS=('task_id','request_id','round')


def identity(message):
    if (not isinstance(message,dict) or
            any(not isinstance(message.get(k),str) or not message[k] for k in IDS[:2]) or
            type(message.get('round')) is not int or not 1<=message['round']<=8):
        raise ValueError('Invalid task/request/round identity')
    return tuple(message[k] for k in IDS)


class PickExchange:
    def __init__(self):
        self.current=None;self.deadline=0.;self.state='idle';self.reason='';self.accepted=None

    def start(self,task_id,round_number,now,timeout=60.):
        if self.state=='waiting':raise ValueError('A pick request is already pending')
        if not math.isfinite(now) or not 1<=timeout<=300:raise ValueError('Invalid deadline')
        request={'task_id':task_id,'request_id':uuid.uuid4().hex,'round':round_number,'command':'start'}
        identity(request)
        self.current=request;self.deadline=now+timeout;self.state='waiting';self.reason='';self.accepted=None
        return copy.deepcopy(request)

    def cancel(self,reason='cancelled'):
        if self.current is None:return None
        self.state='cancelled';self.reason=reason;self.accepted=None
        return dict(self.current,command='cancel')

    def tick(self,now):
        if self.state=='waiting' and now>=self.deadline:
            return self.cancel('pick timeout')
        return None

    def receive(self,message,now):
        self.tick(now)
        try:key=identity(message)
        except ValueError:return False
        if self.state!='waiting' or key!=identity(self.current):return False
        if message.get('success') is False:
            self.state='failed';self.reason=str(message.get('reason','pick failed'));return False
        if not (message.get('success') is True and message.get('category') in ('T','G') and
                message.get('gripper_closed') is True and message.get('holding') is True and
                message.get('arm_control_returned') is True):
            self.reason='invalid category, grip, or arm handoff';return False
        self.accepted=copy.deepcopy(message);self.state='succeeded';return True


class DeliveryTask:
    """Counts only confirmed physical placements; never retries a release."""
    def __init__(self,task_id):
        if not isinstance(task_id,str) or not task_id:raise ValueError('Missing task ID')
        self.task_id=task_id;self.stage='await_start';self.location=None;self.mapping=None
        self.completed=0;self.counts={'T':0,'G':0};self.slots={'pick_A':0,'pick_B':0}
        self.category=None;self.target_park=None;self.reason='';self.exchange=PickExchange()
        self.placements=[];self.place_id=None

    def arrived(self,name,report):
        if not (report.get('nav_status')==4 and report.get('stopped') is True and
                report.get('accepted') is True and
                type(report.get('position_error_m')) in (int,float) and
                type(report.get('yaw_error_deg')) in (int,float) and
                0<=report['position_error_m']<=.02 and 0<=report['yaw_error_deg']<=10):
            self.fail('navigation failed final simultaneous acceptance');return
        expected={'await_start':'start','to_recognition':'shibie','to_pick':'pick',
                  'transport':self.target_park}.get(self.stage)
        if expected!=name:raise ValueError('Unexpected arrival for current stage')
        self.location=name
        self.stage={'await_start':'to_recognition','to_recognition':'recognition',
                    'to_pick':'pick_ready','transport':'placement_ready'}[self.stage]

    def set_mapping(self,mapping):
        if self.stage!='recognition' or set(mapping)!= {'T','G'} or set(mapping.values())!={'pick_A','pick_B'}:
            raise ValueError('Invalid round mapping')
        self.mapping=dict(mapping);self.stage='to_pick'

    def request_pick(self,now):
        if self.stage!='pick_ready':raise ValueError('Pick is not ready')
        self.stage='picking'
        return self.exchange.start(self.task_id,self.completed+1,now)

    def pick_result(self,message,now):
        if self.stage!='picking' or not self.exchange.receive(message,now):return False
        category=self.exchange.accepted['category']
        if self.counts[category]>=4:
            self.fail('more than four items of one category');return False
        self.category=category;self.target_park=self.mapping[category];self.stage='transport_pose'
        return True

    def transport_ready(self,physical_readback_ok,holding):
        if self.stage!='transport_pose':raise ValueError('Unexpected transport posture')
        if physical_readback_ok is not True or holding is not True:
            self.fail('transport pose or holding not verified');return
        self.stage='transport'

    def placement_request(self):
        if self.stage!='placement_ready':raise ValueError('Not parked at destination')
        self.place_id=uuid.uuid4().hex;self.stage='placing'
        return {'task_id':self.task_id,'placement_id':self.place_id,'round':self.completed+1,
                'park':self.target_park,'slot':self.slots[self.target_park]+1,'category':self.category}

    def placed(self,result):
        if self.stage!='placing' or not isinstance(result,dict):return False
        if type(result.get('round')) is not int or type(result.get('slot')) is not int:return False
        expected={'task_id':self.task_id,'placement_id':self.place_id,'round':self.completed+1,
                  'park':self.target_park,'slot':self.slots[self.target_park]+1}
        if any(result.get(k)!=v for k,v in expected.items()):return False
        if not (result.get('success') is True and result.get('physical_placement_confirmed') is True
                and result.get('servo_readback_ok') is True and result.get('gripper_released') is True):
            self.fail('placement not physically confirmed');return False
        self.placements.append(copy.deepcopy(result));self.completed+=1
        self.counts[self.category]+=1;self.slots[self.target_park]+=1
        self.stage='complete' if self.completed==8 else 'to_pick'
        return True

    def fail(self,reason):
        cancel=self.exchange.cancel(reason);self.stage='failed';self.reason=reason
        return cancel

    def status(self):
        waiting={'recognition':'等待连续三帧一致识别','picking':'等待当前请求的夹取回报',
                 'transport_pose':'等待运输姿态物理读回','placing':'执行放置并等待实物落点确认',
                 'await_start':'等待起点核验'}
        return {'task_id':self.task_id,'stage':self.stage,'target_park':self.target_park,
                'completed':self.completed,'reason':self.reason or self.exchange.reason or waiting.get(self.stage,''),
                'round':min(8,self.completed+1),'counts':dict(self.counts),'slots':dict(self.slots)}
