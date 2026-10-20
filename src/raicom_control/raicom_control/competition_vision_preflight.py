"""Verify a current capture/inference before permitting the first navigation.

Preparation is allowed to see zero markers. Only the stopped recognition stage
may adopt the category/park mapping. A reachable HTTP landing page is not proof
that the sensor opened. Preview delivery is optional; a successful infer
response already comes from the camera capture inside the original API.
"""
from concurrent.futures import CancelledError
import math
import uuid


class VisionPreflight:
    def __init__(self):
        self.task_id='competition-preflight-'+uuid.uuid4().hex
        self.future=self.reply=None
        self.valid_until=-math.inf
        self.request_deadline=-math.inf
        self.next_attempt=0.
        self.detail='等待相机实际抓拍与推理检查'

    def _failed(self, client, now, reason):
        self.future=self.reply=None
        self.valid_until=-math.inf
        self.request_deadline=-math.inf
        self.next_attempt=now+3.
        self.detail='视觉待就绪：'+str(reason)+'；3秒后重试'
        client.cancel()
        return False

    def poll(self, client, now, *, backend_ready, allow_request=True):
        if not backend_ready or not client.ready():
            self.valid_until=-math.inf
            if self.future is not None or self.reply is not None:
                return self._failed(client,now,'节点或相机服务失联')
            self.detail=client.connection_status()
            return False
        if now<self.valid_until:
            return True
        if not allow_request:
            self.detail='等待当前相机使用结束后进行比赛抓拍检查'
            return False
        try:
            if self.future is not None:
                # Match VisionClient's request limit even if its result arrived
                # before a delayed GUI poll; old captures cannot become fresh.
                if now>=self.request_deadline:
                    raise TimeoutError('抓拍/识别请求超时，等待重新检查')
                if not self.future.done():
                    return False
                self.reply=self.future.result();self.future=None
                payload=self.reply.get('payload',{})
                size=payload.get('image_size')
                if (payload.get('status')!='success' or not isinstance(payload.get('detections'),list)
                        or any(not isinstance(item,dict) for item in payload['detections'])
                        or self.reply.get('error') or not isinstance(size,(list,tuple)) or len(size)!=2
                        or any(type(v)is not int or not 0<v<=8192 for v in size)):
                    raise RuntimeError(self.reply.get('error') or '抓拍/识别响应无效')
                self.reply=None;self.valid_until=min(now+10.,self.request_deadline)
                self.detail='本次抓拍与推理已通过；预览画面不影响运行'
                return True
            if now>=self.next_attempt:
                self.future=client.request(self.task_id)
                self.request_deadline=now+12.
                self.detail='正在检查实际抓拍和推理 · 尚未发导航目标'
        except CancelledError:
            return self._failed(client,now,'抓拍/识别请求已取消')
        except (ValueError,RuntimeError,OSError,TypeError,AttributeError) as exc:
            return self._failed(client,now,exc)
        return False
