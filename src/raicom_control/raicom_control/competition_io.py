"""Numbered competition companion interface; no driver or actuator publishers."""
from collections import deque
from threading import Event, Lock, Thread
import json
import math
import time
import uuid

SCHEMA = 'raicom.competition/v1'
PREFIX = '/raicom/competition'
KEYS = ('backend_id', 'client_id', 'task_id', 'request_id', 'item', 'geometry_identity', 'mapping_id', 'pick_request_id', 'operation')
OPERATIONS = {'classify', 'transport', 'place', 'verify_placement'}
REQUIRED_OPERATIONS = {'classify', 'transport', 'place'}


def recent(value, now):
    stamp = value.get('stamp_unix')
    return type(stamp) in (float, int) and math.isfinite(stamp) and -.5 <= now - stamp <= 3


class CompanionSession:
    def __init__(self):
        self.client_id = uuid.uuid4().hex
        self.health, self.health_at = {}, -math.inf
        self.pending = self.result = None
        self.progress = None
        self.cancelled = False

    def observe_health(self, value, now, wall):
        if (not isinstance(value, dict) or value.get('schema') != SCHEMA or not recent(value, wall)
                or not isinstance(value.get('backend_id'), str) or not value['backend_id']
                or type(value.get('seq')) is not int): return False
        if self.health.get('backend_id') == value['backend_id'] and value['seq'] <= self.health['seq']: return False
        self.health, self.health_at = value, now
        return True

    def fresh(self, now): return bool(self.health and now - self.health_at <= 3)

    def ready(self, now):
        return (not self.pending and self.fresh(now) and self.health.get('ready') is True
                and self.health.get('arm_released') is True
                and all(x in self.health.get('capabilities', []) for x in REQUIRED_OPERATIONS))

    def start(self, operation, context, fields, now, wall):
        if operation not in OPERATIONS or not self.ready(now) or operation not in self.health.get('capabilities', []):
            raise RuntimeError('分类/运输/放置后端未就绪或不支持本次操作')
        timeout=600 if operation=='place' else 120
        self.pending = dict(fields, **context, schema=SCHEMA, backend_id=self.health['backend_id'],
                            client_id=self.client_id, request_id=uuid.uuid4().hex, operation=operation,
                            command='start', stamp_unix=wall, expires_unix=wall+3, timeout_s=timeout)
        self.deadline, self.cancelled, self.result = now+timeout, False, None
        self.progress = None
        return dict(self.pending)

    def receive(self, value, wall):
        if (not self.pending or not isinstance(value, dict) or value.get('schema') != SCHEMA
                or not recent(value, wall) or any(value.get(k) != self.pending[k] for k in KEYS)):
            return False
        for key in ('profile_id', 'park', 'point', 'slot', 'placement_request_id'):
            if key in self.pending and value.get(key) != self.pending[key]:return False
        if value.get('status')=='running':
            self.progress=value
            return True
        if value.get('status') not in {'success', 'failed', 'cancelled'} or value.get('arm_released') is not True:return False
        self.result = dict(value, accepted=value['status']=='success' and not self.cancelled)
        self.pending = None
        return True

    def cancel(self, wall):
        if not self.pending: return None
        self.cancelled = True
        return dict(self.pending, command='cancel', stamp_unix=wall, expires_unix=wall+3)


class CompetitionIO:
    def __init__(self, node):
        from std_msgs.msg import String
        self.node, self.String = node, String
        self.session = CompanionSession()
        self.lock, self.queue = Lock(), deque(maxlen=100)
        self.vision_health = {}
        self.pub = node.create_publisher(String, PREFIX+'/request', 10)
        self.events = node.create_publisher(String, PREFIX+'/event', 10)
        self.lease = node.create_publisher(String, PREFIX+'/lease', 10)
        self.subs = []
        for channel, topic in [('health', PREFIX+'/health'), ('result', PREFIX+'/result'),
                               ('camera', '/raicom/camera/status'), ('vision', '/raicom/vision/status')]:
            self.subs.append(node.create_subscription(String, topic, lambda m, c=channel:self.receive(c,m), 10))
        self.lease_at = 0.
        self._lease_lock = Lock()
        self._lease_state = None
        self._lease_wakeup = Event()
        self._lease_closed = False
        self._lease_thread = None

    def receive(self, channel, message):
        if len(message.data)>65536: return
        try: value=json.loads(message.data)
        except ValueError:return
        with self.lock:self.queue.append((channel,value,time.monotonic(),time.time()))

    def send(self, packet):
        if packet:self.pub.publish(self.String(data=json.dumps(packet,ensure_ascii=False,allow_nan=False)))

    def poll(self):
        with self.lock:messages=list(self.queue);self.queue.clear()
        for channel,value,now,wall in messages:
            if channel=='health':self.session.observe_health(value,now,wall)
            elif channel=='result':self.session.receive(value,wall)
            elif isinstance(value,dict) and type(value.get('at_unix')) in (int,float) and -.5<=wall-value['at_unix']<=8:
                self.vision_health[channel]=(value,now)
        p=self.session.pending
        if p and not self.session.cancelled and (not self.session.fresh(time.monotonic()) or time.monotonic()>self.session.deadline
                                                or self.session.health.get('backend_id')!=p['backend_id']):
            self.cancel()

    def unique(self):
        return self.node.count_subscribers(PREFIX+'/request')==1 and self.node.count_publishers(PREFIX+'/result')==1 and self.node.count_publishers(PREFIX+'/health')==1

    def start(self, operation, context, fields):
        if not self.unique(): raise RuntimeError('需要唯一比赛分类/放置适配节点')
        self.send(self.session.start(operation,context,fields,time.monotonic(),time.time()))

    def cancel(self):self.send(self.session.cancel(time.time()))

    def confirm(self, kind, label=None):
        s = self.session
        if (not s.pending or s.cancelled or not s.fresh(time.monotonic()) or not s.progress
                or s.progress.get('manual_required') is not True
                or kind != s.pending['operation'] or kind not in ('classify','verify_placement')):
            raise RuntimeError('当前没有等待人工确认的本件请求')
        if kind == 'classify' and label not in ('1','2'): raise ValueError('请选择T或G')
        wall = time.time()
        self.send(dict(s.pending,command='confirm_classification' if kind == 'classify' else 'confirm_placement',
                       operator_label=label,stamp_unix=wall,expires_unix=wall+3))

    def event(self, event, **fields):
        self.events.publish(self.String(data=json.dumps(dict(schema=SCHEMA,event=event,stamp_unix=time.time(),**fields),ensure_ascii=False)))

    def heartbeat(self, task, active):
        now=time.monotonic()
        with self._lease_lock:
            if self._lease_closed:return
            self._lease_state=(task.task_id,bool(active),now)
            if self._lease_thread is None:
                self._lease_thread=Thread(target=self._lease_loop,daemon=True,name='competition-lease')
                self._lease_thread.start()
        self._lease_wakeup.set()

    def _lease_loop(self):
        last=None
        while True:
            with self._lease_lock:
                if self._lease_closed:return
                state=self._lease_state
            now=time.monotonic()
            if state is not None:
                task_id,requested,observed_at=state
                active=(requested and now-observed_at<=5. and
                        not getattr(self.node,'external_stop_requested',False))
                identity=(task_id,active)
                if identity!=last or now-self.lease_at>=.4:
                    self.lease_at=now
                    self.lease.publish(self.String(data=json.dumps(dict(
                        schema=SCHEMA,task_id=task_id,client_id=self.session.client_id,
                        active=active,expires_unix=time.time()+1.5))))
                    last=identity
            self._lease_wakeup.wait(.1)
            self._lease_wakeup.clear()

    def close(self):
        with self._lease_lock:
            self._lease_closed=True
        self._lease_wakeup.set()
        if self._lease_thread is not None:self._lease_thread.join(timeout=.5)

    def vision_ready(self, now):
        return all(kind in self.vision_health and now-self.vision_health[kind][1]<=8
                   and self.vision_health[kind][0].get('backend_reachable') is True for kind in ('camera','vision'))
