"""Persistent camera and installed grasp host; numbered automatic transactions."""
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Empty
from sensor_msgs.msg import CompressedImage
from ros_robot_controller_msgs.srv import GetBusServoState
from ros_robot_controller_msgs.msg import GetBusServoCmd

from .depth_pick_runner import OriginalGraspBackend, STAGES, valid_request
from .depth_pick_interface import SCHEMA, PREFIX
from .depth_pick_release import RELEASE, verify_release
from .depth_warm_logic import automatic_holding, marker_result
from .competition_pick_contract import validate_context
from .vision_http import VisionBackend


def perception_launch_command(launch):
    driver_setup=Path('/home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash')
    project_setup=Path('/home/ubuntu/ros2_ws/install/setup.bash')
    if not driver_setup.is_file() or not project_setup.is_file():
        raise RuntimeError('深度相机或项目 ROS 环境不存在，不能启动夹取预热')
    # The GUI's overlay omits the separately installed Aurora930 driver.
    return ['/bin/bash','-c',
            'source /opt/ros/humble/setup.bash && '
            'source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash && '
            'source /home/ubuntu/ros2_ws/install/setup.bash && '
            'exec ros2 launch "$1"', 'depth_perception_warm', str(launch)]


class WarmGraspBackend(OriginalGraspBackend):
    def __init__(self, host):
        self.host = host
        self.warm_child = None
        self.frame_at = 0.
        self.holding_proof = self.destination = None
        self.stop_readback_samples = []
        self.stop_readback_errors = []
        super().__init__()
        self.read_client = self.create_client(GetBusServoState, '/ros_robot_controller/bus_servo/get_state')
        self.create_subscription(Empty, '/raicom/nav_stop', self.emergency_stop, 10)
        self.vision = VisionBackend()
        if RELEASE not in ('rgbd-platform-comp-20260923-smooth','rgbd-platform-comp-20260923-adaptive-center',
                           'rgbd-platform-comp-20260923-center20',
                           'rgbd-platform-comp-20260923-u320-floor'):
            self.version_error = '常驻适配只接受已核对的20260923原生版本'
        if not self.version_error:
            self.start_perception()

    def emergency_stop(self, _):
        with self.lock:
            if self.state in ('running', 'cancelling'):
                self.cancel.set()
                self.reason = '人工急停'

    def start_perception(self):
        names = self.get_node_names()
        if any(n in {'blue_block_3d_node','target_to_ik_node','kinematics','depth_camera_mount_tf'}
               or 'aurora' in n.lower() for n in names):
            self.version_error = '已有深度相机或运动学实例，未重复启动'
            return
        log = self.runs/('warm_'+self.backend_id+'.log')
        launch = Path(__file__).resolve().parents[1]/'launch/depth_perception_warm.launch.py'
        if not launch.is_file():
            from ament_index_python.packages import get_package_share_directory
            launch = Path(get_package_share_directory('raicom_control'))/'launch/depth_perception_warm.launch.py'
        command=perception_launch_command(launch)
        with log.open('xb') as stream:
            self.warm_child = subprocess.Popen(command, stdout=stream,
                stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)

    def warm_ready(self):
        h = self.host.hardware
        return bool(self.warm_child and self.warm_child.poll() is None
            and time.monotonic()-self.frame_at <= 3 and h.ik_client.service_is_ready()
            and h.fk_client.service_is_ready() and h.read_client.service_is_ready()
            and self.get_node_names().count('grasp_once_node') == 1)

    def ready(self):
        return super().ready() and self.warm_ready()

    def receive(self, message):
        if len(message.data)>16384:return
        try:value=json.loads(message.data)
        except ValueError:return
        if not valid_request(value,self.backend_id,time.time()):return
        if value['command']=='confirm_holding':return
        if value['command']=='start' and 'competition' in value:
            try:validate_context(value['competition'])
            except ValueError:return
        with self.lock:
            before = self.state
            super().receive(message)
            if self.state=='idle' and before in ('succeeded','failed','cancelled'):
                # Reset is an identity-scoped software latch, never a release command.
                self.host.holding=False
                self.holding_proof=self.destination=None

    def native_status(self, value):
        # Direct callback from our own host while its worker owns this request.
        # Anonymous ROS messages cannot complete a new transaction.
        with self.lock:
            if self.host.owner is not self or self.state not in ('running','cancelling'):return
            self.stage=STAGES.get(value['stage'],self.stage)
            self.motion_evidence.observe(value)
            with (self.run_dir/'stages.jsonl').open('a',encoding='utf-8') as stream:
                stream.write(json.dumps(dict(value,at_unix=time.time()),ensure_ascii=False)+'\n')
            if value['stage'] in ('auto_grasp_complete','failed','rejected','cancelled','stopped'):
                self.original_terminal=value
            if value['stage']=='stopped':self.cancel.set()

    def _read_positions(self):
        """Read-only stop verification also works after cancellation is latched."""
        values={}
        for sid in (1,2,3,4,5,10):
            future=self.read_client.call_async(GetBusServoState.Request(cmd=[GetBusServoCmd(id=sid,get_position=1)]))
            deadline=time.monotonic()+1.0
            while not future.done() and time.monotonic()<deadline:time.sleep(.02)
            if not future.done():
                self.read_client.remove_pending_request(future);future.cancel()
                raise RuntimeError('停止后的舵机读回超时')
            reply=future.result()
            if (reply is None or not reply.success or len(reply.state)!=1 or len(reply.state[0].position)!=1
                    or type(reply.state[0].position[0])is not int or not 0<=reply.state[0].position[0]<=1000):
                raise RuntimeError(f'停止后的舵机ID{sid}读回无效')
            values[sid]=reply.state[0].position[0]
        return values

    def run_original(self):
        problem=''; acquired=False; samples=[]
        self.destination=self.holding_proof=None
        self.stop_readback_samples=[]
        self.stop_readback_errors=[]
        try:
            version=verify_release()
            (self.run_dir/'version.json').write_text(json.dumps(version,indent=2),encoding='utf-8')
            if not self.warm_ready():raise RuntimeError('深度相机预热未完成或已失联')
            context=self.request.get('competition')
            if context and context.get('classification_stage','before_grasp') != 'after_grasp':
                self.stage='先识别当前物品标志'
                for attempt in range(20):
                    if self.cancel.is_set() or not self.stationary(True):raise RuntimeError('标志识别已取消或未停稳')
                    if time.monotonic()>=self.deadline:raise RuntimeError('本次标志识别超时')
                    try:self.destination=marker_result(context,self.vision.perform('infer')['payload'])
                    except (ValueError,RuntimeError,OSError) as exc:
                        self.stage=f'标志识别 {attempt+1}/20：{exc}'
                    else:break
                if self.destination is None:raise RuntimeError('20次未取得本件有效1/2，未发抓取动作')
                (self.run_dir/'marker.json').write_text(json.dumps(self.destination,ensure_ascii=False),encoding='utf-8')
            self.host.acquire(self);acquired=True
            self.stage='抓取程序已预热，执行本次抓取'
            self.host.auto_prepare_worker()  # Installed, hash-checked trajectory unchanged.
            if not self.motion_evidence.complete(self.original_terminal):
                problem=str((self.original_terminal or {}).get('detail') or self.motion_evidence.error or '抓取动作未完成')
        except Exception as exc:problem=str(exc)
        finally:
            if acquired or self.host.owner is self:
                self.host.hardware.stop()
                readback_failures=0
                # Do not return ownership on a timeout without physical stop evidence.
                while True:
                    try:
                        samples.append(self._read_positions());samples=samples[-2:]
                        if len(samples)==2 and all(abs(samples[1][i]-samples[0][i])<=3 for i in samples[0]):break
                    except Exception as exc:
                        samples=[];readback_failures+=1
                        self.stop_readback_errors.append(str(exc))
                        self.stop_readback_errors=self.stop_readback_errors[-20:]
                        # Match the native read policy: at most two transient
                        # read retries, never replay a physical grasp. A bad
                        # sample cannot permanently cancel a recovered request.
                        if readback_failures>=3:
                            self.reason=self.reason or '停止读回累计三次失败：'+str(exc)
                            self.cancel.set()
                    time.sleep(.15)
                self.host.relinquish()
            end=time.monotonic()+3
            while not self.arm_free() and time.monotonic()<end:time.sleep(.05)
            with self.lock:
                self.released=self.arm_free() and self.host.owner is None
                self.stop_readback_samples=[dict(values) for values in samples]
                try:
                    if problem or self.cancel.is_set() or not self.released:
                        raise ValueError(problem or self.reason or '本次已取消或控制权尚未释放')
                    self.holding_proof=automatic_holding(self.motion_evidence,self.original_terminal,samples)
                    self.state,self.stage,self.holding='succeeded','抓取及非空夹爪读回通过',True
                except ValueError as exc:
                    self.state='cancelled' if self.cancel.is_set() else 'failed'
                    self.reason=self.reason or str(exc)
                self.awaiting_confirmation=False
                self.save_result()

    def save_result(self):
        super().save_result()
        if hasattr(self,'run_dir'):
            path=self.run_dir/'result.json'
            value=json.loads(path.read_text(encoding='utf-8'))
            value.update(confirmation_mode='automatic_only',holding_confirmation=self.holding_proof,
                         stop_readback_samples=getattr(self,'stop_readback_samples',[]),
                         stop_readback_errors=getattr(self,'stop_readback_errors',[]),
                         competition_result=self.destination if self.state=='succeeded' else None)
            path.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')

    def tick(self):
        with self.lock:
            if self.state in ('running','cancelling') and (time.monotonic()>=self.deadline or not self.stationary()):
                self.cancel.set();self.reason=self.reason or '超时、移动或停车反馈失联'
            self.seq+=1
            warm=self.warm_ready();ready=self.ready()
            packet=dict(schema=SCHEMA,backend_id=self.backend_id,seq=self.seq,stamp_unix=time.time(),
                state=self.state,stage=self.stage,reason=self.reason or self.version_error or ('' if warm else '相机/识别/运动学预热中'),
                ready=ready,arm_released=self.released,holding=self.holding,awaiting_confirmation=False,
                release=RELEASE,confirmation_mode='automatic_only',automatic_holding_available=True,
                holding_confirmation=self.holding_proof,competition_result=self.destination if self.state=='succeeded' else None,
                motion_result=self.motion_evidence.packet(),prewarmed=warm,gripper_close=50,gripper_open=350)
            if self.request:packet.update({k:self.request[k] for k in ('client_id','request_id')})
            self.status_pub.publish(String(data=json.dumps(packet,ensure_ascii=False)))

    def preview(self,msg):
        self.frame_at=time.monotonic()
        if self.frame_at-self.last_image<.3:return
        self.last_image=self.frame_at
        try:
            import cv2
            from cv_bridge import CvBridge
            ok,data=cv2.imencode('.jpg',CvBridge().imgmsg_to_cv2(msg,'bgr8'),[cv2.IMWRITE_JPEG_QUALITY,75])
            if ok:self.image_pub.publish(CompressedImage(header=msg.header,format='jpeg',data=data.tobytes()))
        except Exception as exc:self.get_logger().warning('相机预览转换失败：'+str(exc))

    def close_perception(self):
        if self.warm_child:
            for sig,wait in ((signal.SIGINT,3),(signal.SIGTERM,2),(signal.SIGKILL,1)):
                if self.warm_child.poll()is not None:break
                try:os.killpg(self.warm_child.pid,sig);self.warm_child.wait(timeout=wait)
                except (ProcessLookupError,subprocess.TimeoutExpired):pass


def main():
    # Keep the persistent image/IK workers from oversubscribing the Pi CPU.
    os.environ['OPENBLAS_NUM_THREADS']='1'
    os.environ['OMP_NUM_THREADS']='1'
    from rclpy.signals import SignalHandlerOptions
    from .depth_warm_host import WarmGraspHost
    lock=open('/tmp/raicom_depth_pick_backend.lock','a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:return
    rclpy.init(args=[],signal_handler_options=SignalHandlerOptions.NO)
    host=WarmGraspHost();node=WarmGraspBackend(host)
    executor=MultiThreadedExecutor(num_threads=4);executor.add_node(host);executor.add_node(node)
    def close(*_):node.closing.set();node.cancel.set()
    signal.signal(signal.SIGINT,close);signal.signal(signal.SIGTERM,close)
    try:
        while not node.closing.is_set() or (node.worker is not None and node.worker.is_alive()):
            executor.spin_once(timeout_sec=.05)
    finally:
        node.close_perception();executor.shutdown();host.destroy_node();node.destroy_node();rclpy.shutdown();lock.close()


if __name__=='__main__':main()
