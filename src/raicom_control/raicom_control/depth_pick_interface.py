"""Workbench-only depth-pick contract. No camera launch or actuator access."""
import json
import math
import time
import uuid
from copy import deepcopy
from collections import deque
from threading import Lock

SCHEMA = 'raicom.depth_pick/v1'
PREFIX = '/raicom/depth_pick'
TERMINAL = {'succeeded', 'failed', 'cancelled'}
STATES = TERMINAL | {'idle', 'running', 'cancelling'}


class DepthPickSession:
    """A cancel/timeout stays locked until this exact request is stopped."""
    def __init__(self):
        self.client_id = uuid.uuid4().hex
        self.status = None
        self.received_at = -math.inf
        self.pending = None
        self.cancel_requested = False
        self.message = '接口未连接 · 等待抓取后端接入'
        self.completed = None
        self.acknowledged = False

    def receive(self, value, now, wall):
        if not isinstance(value, dict) or value.get('schema') != SCHEMA:
            return False
        stamp = value.get('stamp_unix')
        if (type(stamp) not in (int, float) or not math.isfinite(stamp)
                or not -.5 <= wall - stamp <= 3
                or value.get('state') not in STATES
                or not isinstance(value.get('backend_id'), str) or not value['backend_id']
                or type(value.get('seq')) is not int or value['seq'] < 0):
            return False
        previous = self.status
        if previous and previous['backend_id'] == value['backend_id'] and value['seq'] <= previous['seq']:
            return False
        self.status, self.received_at = dict(value), now
        if self.pending:
            matches = all(value.get(k) == self.pending[k] for k in ('backend_id', 'client_id', 'request_id'))
            if not matches:
                self.message = '等待本次抓取回执 · 后端任务或实例已变化'
                return True
            self.acknowledged = True
            if value['state'] in TERMINAL and value.get('arm_released') is True:
                accepted = value['state'] == 'succeeded' and not self.cancel_requested and value.get('holding') is True
                self.completed = dict(value, accepted=accepted)
                self.message = ('抓取完成 · 后端报告已夹持' if accepted else
                                '本次已结束 · ' + ('取消后结果不计成功' if self.cancel_requested else str(value.get('reason') or '未确认夹持成功')))
                self.pending = None
                self.cancel_requested = False
            else:
                self.message = ('正在取消 · 等待机械臂停止回执' if self.cancel_requested else
                                '抓取阶段 · ' + str(value.get('stage') or value['state'])[:160])
        else:
            self.message = ('已连接 · 可以请求单次抓取' if self.ready(now) else
                            '后端未就绪 · ' + str(value.get('reason') or value.get('stage') or value['state'])[:160])
        return True

    def fresh(self, now):
        return self.status is not None and now - self.received_at <= 3

    def ready(self, now):
        return bool(not self.pending and self.fresh(now) and self.status.get('ready') is True
                    and self.status.get('state') == 'idle' and self.status.get('arm_released') is True)

    def busy(self, now):
        return bool(self.pending or (self.status and
                    (self.status.get('state') in {'running', 'cancelling'} or self.status.get('arm_released') is not True)))

    def start(self, now, wall, competition=None):
        if not self.ready(now):
            raise RuntimeError('抓取后端尚未就绪；不能发送抓取请求')
        if competition is not None:
            from .competition_pick_contract import validate_context
            competition=validate_context(competition)
        self.pending = dict(schema=SCHEMA, command='start', client_id=self.client_id,
                            request_id=uuid.uuid4().hex, backend_id=self.status['backend_id'],
                            target='blue_cube', stamp_unix=wall, expires_unix=wall + 3,
                            timeout_s=120)
        if competition is not None:self.pending['competition']=competition
        self.deadline = now + 120
        self.cancel_requested = False
        self.completed = None
        self.acknowledged = False
        self.ack_deadline = now + 5
        self.message = '已请求单次抓取 · 等待后端接收'
        return deepcopy(self.pending)

    def cancel(self, wall):
        if not self.pending:
            return None
        self.cancel_requested = True
        self.message = '已请求取消 · 等待机械臂停止回执'
        return dict(self.pending, command='cancel', stamp_unix=wall, expires_unix=wall + 3)

    def tick(self, now, wall):
        if self.pending and not self.cancel_requested and (now >= self.deadline or not self.fresh(now)
                                                          or (not self.acknowledged and now >= self.ack_deadline)):
            packet = self.cancel(wall)
            self.message = '抓取超时或接口失联 · 已请求取消，停止状态未确认'
            return packet
        if not self.pending and self.status and not self.fresh(now):
            self.message = '抓取接口失联 · 等待新状态'
        return None


class DepthPickClient:
    """Lazy opt-in ROS transport; executor callbacks never access Tk."""
    def __init__(self, node):
        from std_msgs.msg import String
        from sensor_msgs.msg import CompressedImage
        from rclpy.qos import qos_profile_sensor_data
        self.node, self.String = node, String
        self.session = DepthPickSession()
        self.lock = Lock()
        self.events = deque(maxlen=30)
        self.image = None
        self.publisher = node.create_publisher(String, PREFIX + '/request', 10)
        self.subscription = node.create_subscription(String, PREFIX + '/status', self._receive, 10)
        self.images = node.create_subscription(CompressedImage, PREFIX + '/preview/compressed', self._image, qos_profile_sensor_data)

    def _receive(self, msg):
        if len(msg.data) > 16384:
            return
        try:
            value = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self.lock:
            self.events.append((value, time.monotonic(), time.time()))

    def _image(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if not 0 < len(msg.data) <= 2_000_000 or not -.5 <= time.time() - stamp <= 3:
            return
        with self.lock:
            self.image = (bytes(msg.data), time.monotonic(), stamp)

    def send(self, packet):
        if packet is not None:
            self.publisher.publish(self.String(data=json.dumps(packet)))

    def poll(self):
        with self.lock:
            events = list(self.events)
            self.events.clear()
        for value, received, wall in events:
            self.session.receive(value, received, wall)
        self.send(self.session.tick(time.monotonic(), time.time()))

    def start(self, competition=None):
        if self.node.count_subscribers(PREFIX + '/request') != 1 or self.node.count_publishers(PREFIX + '/status') != 1:
            raise RuntimeError('需要唯一抓取后端；当前接口未连接或有重复实例')
        self.send(self.session.start(time.monotonic(), time.time(), competition))

    def cancel(self):
        self.send(self.session.cancel(time.time()))

    def confirm_holding(self):
        pending = self.session.pending
        status = self.session.status or {}
        if (not pending or self.session.cancel_requested or not self.session.fresh(time.monotonic())
                or status.get('awaiting_confirmation') is not True
                or any(status.get(k) != pending[k] for k in ('backend_id', 'client_id', 'request_id'))):
            raise RuntimeError('当前没有待确认的夹持结果')
        wall = time.time()
        self.send(dict(pending, command='confirm_holding', stamp_unix=wall, expires_unix=wall+3))

    def confirm_empty(self):
        result = self.session.completed
        if self.session.pending or not result or result.get('arm_released') is not True:
            raise RuntimeError('本次动作尚未停止，不能开始下一次')
        wall = time.time()
        self.send(dict(result, command='confirm_empty', stamp_unix=wall, expires_unix=wall+3))

    def take_image(self):
        with self.lock:
            result, self.image = self.image, None
        return result
