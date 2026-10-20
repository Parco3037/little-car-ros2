"""Correlated asynchronous String/JSON client, shared by station workflows."""
from concurrent.futures import Future
import json
import hashlib
import threading
import time
import uuid


class VisionClient:
    def __init__(self,node,*,with_images=False):
        from std_msgs.msg import String
        self.node,self.message_type=node,String
        self.publisher=node.create_publisher(String,'/raicom/vision/request',10)
        self.subscription=node.create_subscription(String,'/raicom/vision/result',self.receive,10)
        self.pending=None;self.lock=threading.RLock()
        self.health={}
        self.status_subscriptions=[node.create_subscription(String,topic,self.receive_status,10)
                                   for topic in ('/raicom/camera/status','/raicom/vision/status')]
        self.image_subscription=None;self.images={};self.completed=None;self.preview=None
        if with_images:
            from sensor_msgs.msg import CompressedImage
            self.image_subscription=node.create_subscription(CompressedImage,'/raicom/vision/image/compressed',self.receive_image,2)

    def ready(self):
        return self.publisher.get_subscription_count()==1 and len(self.node.get_publishers_info_by_topic('/raicom/vision/result'))==1

    def receive_status(self,msg):
        try:value=json.loads(msg.data)
        except (ValueError,TypeError):return
        if not isinstance(value,dict) or value.get('node') not in ('yolo_camera','yolo_infer'):return
        with self.lock:self.health[value['node']]=(time.monotonic(),value)

    def connection_status(self):
        requests=self.publisher.get_subscription_count()
        results=len(self.node.get_publishers_info_by_topic('/raicom/vision/result'))
        if requests>1 or results>1:return '发现重复识别节点，未发送识别请求'
        if requests!=1 or results!=1:return '相机节点连接中 · 等待 yolo_infer'
        with self.lock:health=dict(self.health)
        for name,port in [('yolo_camera',8000),('yolo_infer',8080)]:
            received,value=health.get(name,(0.,{}))
            if time.monotonic()-received<8 and value.get('backend_reachable') is False:
                return f'{name} 已连接，但 {port} 服务不可用：'+str(value.get('error','连接失败'))
        return '识别节点已连接 · 等待拍照结果'

    def request(self,task_id):
        with self.lock:
            self.poll()
            if self.pending is not None:raise RuntimeError('识别请求尚未结束')
            if not self.ready():raise RuntimeError('等待唯一 yolo_infer ROS2 节点')
            message={'task_id':task_id,'request_id':uuid.uuid4().hex,'command':'start'}
            future=Future();self.pending=(message,future,time.monotonic()+12.)
            self.images.clear();self.completed=None
            try:self.publisher.publish(self.message_type(data=json.dumps(message)))
            except Exception:self.pending=None;raise
            return future

    def receive(self,msg):
        try:result=json.loads(msg.data)
        except ValueError:return
        if not isinstance(result,dict):return
        with self.lock:
            self.poll()
            if self.pending is None:return
            request,future,_=self.pending
            if any(result.get(k)!=request[k] for k in ('task_id','request_id')):return
            self.pending=None
            if future.done():return
            if result.get('status')=='success' and isinstance(result.get('reply'),dict):
                self.completed=result;self._pair_preview();future.set_result(result['reply'])
            else:
                self.images.clear();self.completed=self.preview=None
                future.set_exception(RuntimeError(result.get('error','视觉请求取消或失败')))

    def receive_image(self,msg):
        with self.lock:
            if self.pending is None and self.completed is None:return
            if msg.format!='jpeg' or not 0<len(msg.data)<=5_000_000:return
            key=(msg.header.stamp.sec,msg.header.stamp.nanosec)
            self.images[key]=bytes(msg.data)
            while len(self.images)>2:self.images.pop(next(iter(self.images)))
            self._pair_preview()

    def _pair_preview(self):
        if self.completed is None:return
        metadata=self.completed['reply'].get('preview',{})
        stamp=metadata.get('stamp') if isinstance(metadata,dict) else None
        if not isinstance(stamp,list) or len(stamp)!=2 or any(type(v) is not int for v in stamp):return
        jpeg=self.images.get(tuple(stamp))
        if jpeg is None or hashlib.sha256(jpeg).hexdigest()!=metadata.get('sha256'):return
        self.preview=dict(self.completed,jpeg=jpeg)
        self.images.clear();self.completed=None

    def take_preview(self):
        with self.lock:
            result=self.preview;self.preview=None
            return result

    def poll(self):
        with self.lock:
            if self.pending and (time.monotonic()>=self.pending[2] or self.pending[1].cancelled()):
                request,future,_=self.pending;self.pending=None
                self.images.clear();self.completed=self.preview=None
                if not future.done():future.set_exception(TimeoutError('ROS2视觉请求超时'))
                self.publisher.publish(self.message_type(data=json.dumps(dict(request,command='cancel'))))

    def cancel(self):
        with self.lock:
            self.images.clear();self.completed=self.preview=None
            if self.pending:
                request,future,_=self.pending;self.pending=None;future.cancel()
                self.publisher.publish(self.message_type(data=json.dumps(dict(request,command='cancel'))))

    def close(self):
        try:self.cancel()
        finally:
            self.node.destroy_subscription(self.subscription);self.node.destroy_publisher(self.publisher)
            for subscription in self.status_subscriptions:self.node.destroy_subscription(subscription)
            if self.image_subscription is not None:self.node.destroy_subscription(self.image_subscription)
