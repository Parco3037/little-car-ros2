"""Two ROS2 proxy nodes; neither opens a camera nor starts/modifies Docker."""
from concurrent.futures import ThreadPoolExecutor
import json
import hashlib
import time
import uuid

from .vision_http import VisionBackend, valid_request


def run(kind,args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from sensor_msgs.msg import CompressedImage

    class HttpNode(Node):
        def __init__(self):
            super().__init__('yolo_camera' if kind=='camera' else 'yolo_infer')
            for name,value in [('camera_url','http://127.0.0.1:8000'),('infer_url','http://127.0.0.1:8080'),('timeout_s',5.)]:
                self.declare_parameter(name,value)
            self.backend=VisionBackend(**{k:self.get_parameter(k).value for k in ('camera_url','infer_url')},
                                       timeout=float(self.get_parameter('timeout_s').value))
            self.prefix='/raicom/camera' if kind=='camera' else '/raicom/vision'
            self.result_pub=self.create_publisher(String,self.prefix+'/result',10)
            self.status_pub=self.create_publisher(String,self.prefix+'/status',10)
            self.image_pub=self.create_publisher(CompressedImage,self.prefix+'/image/compressed',1)
            self.sub=self.create_subscription(String,self.prefix+'/request',self.request,10)
            self.service=self.create_service(Trigger,self.prefix+('/capture' if kind=='camera' else '/infer'),self.trigger)
            self.pool=ThreadPoolExecutor(max_workers=1)
            self.future=None;self.health_future=None
            self.current=None;self.cancelled=False;self.seen=set();self.health_at=0.
            self.timer=self.create_timer(.05,self.poll)

        def publish(self,pub,value):pub.publish(String(data=json.dumps(value,ensure_ascii=False,allow_nan=False)))

        def enqueue(self,request):
            if self.future is not None or request['request_id'] in self.seen:return False
            if len(self.seen)>=4096:raise RuntimeError('vision request budget exhausted; restart idle adapter')
            self.seen.add(request['request_id']);self.current=dict(request);self.cancelled=False
            self.future=self.pool.submit(self.backend.perform,kind)
            return True

        def request(self,msg):
            try: value=json.loads(msg.data)
            except ValueError:return
            if not valid_request(value):return
            if value['command']=='cancel':
                if self.current and all(value[k]==self.current[k] for k in ('request_id','task_id')):self.cancelled=True
                return
            try:accepted=self.enqueue(value)
            except RuntimeError:accepted=False
            if not accepted:
                # A duplicate current request must not abort the original future.
                if self.current and all(value[k]==self.current[k] for k in ('request_id','task_id')):return
                self.publish(self.result_pub,dict(value,status='error',error='vision node busy or request already consumed'))

        def trigger(self,_request,response):
            request={'request_id':uuid.uuid4().hex,'task_id':'manual-service','command':'start'}
            try:response.success=self.enqueue(request)
            except RuntimeError:response.success=False
            response.message=json.dumps(dict(request,queued=response.success))
            return response  # Never block an executor callback on HTTP.

        def poll(self):
            if self.health_future is not None and self.health_future.done():
                future=self.health_future;self.health_future=None
                try:health=future.result()
                except Exception as exc:health={'backend_reachable':False,'error':str(exc),'recognition_validated':False}
                self.publish(self.status_pub,dict(health,node=self.get_name(),at_unix=time.time()))
            if self.future is not None and self.future.done():
                future,current=self.future,self.current
                self.future=self.current=None
                if current is None:
                    self.publish(self.status_pub,dict(future.result(),node=self.get_name(),at_unix=time.time()))
                else:
                    result={k:current[k] for k in ('task_id','request_id')}
                    image=None
                    try:
                        reply=future.result()
                        jpeg=reply.pop('jpeg',None)
                        result.update(status='cancelled' if self.cancelled else 'success',reply=reply)
                        if jpeg and not self.cancelled:
                            image=CompressedImage();image.header.stamp=self.get_clock().now().to_msg()
                            image.header.frame_id='pi_camera'
                            image.format='jpeg';image.data=jpeg
                            if kind=='infer':
                                reply['preview']={'stamp':[image.header.stamp.sec,image.header.stamp.nanosec],
                                                  'sha256':hashlib.sha256(jpeg).hexdigest()}
                    except Exception as exc:result.update(status='error',error=str(exc))
                    # Deliver the correlated result before optional JPEG data.
                    # A slow or failed preview must not change successful inference.
                    self.publish(self.result_pub,result)
                    if image is not None and result['status']=='success':
                        try:self.image_pub.publish(image)
                        except Exception as exc:self.get_logger().warning('预览图片发布失败：'+str(exc))
                    self.health_at=time.monotonic()
            if self.future is None and self.health_future is None and time.monotonic()-self.health_at>=3.:
                self.health_at=time.monotonic()
                # Keep health separate from the correlated request. A request
                # arriving during a health GET queues behind it in this same
                # single worker instead of being rejected as "busy".
                self.health_future=self.pool.submit(self.backend.health,kind)

        def close(self):
            self.cancelled=True;self.pool.shutdown(wait=False,cancel_futures=True)
            self.destroy_node()

    rclpy.init(args=args);node=HttpNode()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:node.close();rclpy.try_shutdown()


def camera_main(args=None):run('camera',args)
def infer_main(args=None):run('infer',args)
