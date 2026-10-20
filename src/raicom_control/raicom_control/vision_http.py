"""Bounded HTTP adapter for the existing, unmodified camera/YOLO containers."""
from contextlib import contextmanager
import json
from pathlib import Path
import time
from urllib.request import Request, urlopen
from urllib.parse import urlsplit


def validate_base(url):
    p=urlsplit(url)
    if p.scheme not in ('http','https') or not p.hostname or p.username or p.password or p.query or p.fragment or p.path not in ('','/'):
        raise ValueError('Camera/YOLO URL must be a base HTTP(S) address without credentials')
    return url.rstrip('/')


def request_bytes(url, method='GET', *, timeout=5., limit=2_000_000, opener=urlopen):
    started=time.monotonic()
    with opener(Request(url,method=method),timeout=timeout) as response:
        data=bytearray()
        while True:
            if time.monotonic()-started>timeout: raise TimeoutError('HTTP response deadline exceeded')
            chunk=getattr(response,'read1',response.read)(min(65536,limit+1-len(data)))
            data.extend(chunk)
            if len(data)>limit: raise ValueError('HTTP response exceeds size limit')
            if not chunk: break
        return bytes(data)


@contextmanager
def shared_capture_lock(path='/tmp/raicom_vision_http.lock'):
    # Linux ROS host: serialize capture and inference across both adapter nodes.
    # The original web buttons are outside this lock; avoid using them during a task.
    import fcntl
    with Path(path).open('a') as stream:
        try: fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError: raise RuntimeError('camera/inference request already in progress') from None
        try: yield
        finally: fcntl.flock(stream,fcntl.LOCK_UN)


class VisionBackend:
    def __init__(self,camera_url='http://127.0.0.1:8000',infer_url='http://127.0.0.1:8080',
                 timeout=5.,fetch=request_bytes,lock=shared_capture_lock):
        self.camera_url,self.infer_url=validate_base(camera_url),validate_base(infer_url)
        if not 1<=timeout<=10:raise ValueError('HTTP timeout must be 1..10 seconds')
        self.timeout,self.fetch,self.lock=timeout,fetch,lock
        self.marker_cache = None
        if fetch is request_bytes:
            from .vision_mapping_cache import MarkerCache
            boot = Path('/proc/sys/kernel/random/boot_id')
            if boot.is_file():
                self.marker_cache = MarkerCache(Path.home()/'raicom_runtime/vision/marker_cache.json',
                                                boot.read_text().strip(),[self.camera_url,self.infer_url])

    def health(self,kind):
        url=self.camera_url+'/api/params' if kind=='camera' else self.infer_url+'/'
        try:
            self.fetch(url,timeout=min(2.,self.timeout),limit=200_000)
            return {'backend_reachable':True,'url':url,'recognition_validated':False}
        except Exception as exc:
            return {'backend_reachable':False,'url':url,'error':str(exc),'recognition_validated':False}

    def perform(self,kind):
        if kind not in ('camera','infer'):raise ValueError('Unknown vision operation')
        started=time.monotonic()
        with self.lock():
            url=self.camera_url+'/api/capture_temp' if kind=='camera' else self.infer_url+'/api/do_infer'
            def reject_constant(value):raise ValueError('Nonfinite JSON value: '+value)
            payload=json.loads(self.fetch(url,'POST',timeout=self.timeout),parse_constant=reject_constant)
            if not isinstance(payload,dict) or payload.get('status')!='success':
                raise RuntimeError(str(payload.get('error','backend business failure')) if isinstance(payload,dict) else 'Malformed response')
            if kind=='infer' and not isinstance(payload.get('detections'),list):raise ValueError('YOLO detections missing')
            reply={'started_monotonic':started,'finished_monotonic':time.monotonic(),'payload':payload,
                   'image_source':'pi_camera','timestamp_is_request_time_not_exposure':True}
            if kind=='infer':
                from .vision_decision import decide_markers
                reply['decision']=decide_markers(payload)
                if self.marker_cache is not None:
                    try:
                        self.marker_cache.observe(payload)
                        cached=self.marker_cache.fallback(payload)
                        if cached is not None:reply['marker_fallback']=cached
                    except OSError as exc:reply['mapping_cache_error']=str(exc)
            if kind=='camera':
                reply['jpeg']=self.fetch(self.camera_url+'/api/temp_image',timeout=self.timeout,limit=5_000_000)
                if not reply['jpeg'].startswith(b'\xff\xd8'):raise ValueError('Camera response is not JPEG')
            else:
                # Fetch the inference input under the SAME cross-process lock.
                # A later GET from the GUI could pair another capture with these boxes.
                try:
                    jpeg=self.fetch(self.camera_url+'/api/temp_image',timeout=min(2.,self.timeout),limit=5_000_000)
                    if not jpeg.startswith(b'\xff\xd8'):raise ValueError('Camera response is not JPEG')
                    reply['jpeg']=jpeg
                except Exception as exc:
                    # Preview failure must not change the existing recognition contract.
                    reply['preview_error']=str(exc)
            return reply


def valid_request(value):
    return (isinstance(value,dict) and all(isinstance(value.get(k),str) and 0<len(value[k])<=128
            for k in ('request_id','task_id')) and value.get('command') in ('start','cancel'))
