"""Workbench lifecycle adapter for the field-accepted native grasp launch.

Idle startup creates only status/image publishers. A fresh explicit start launches
the bound native program once; cancellation stops that process before release.
"""
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

from .depth_pick_interface import SCHEMA, PREFIX
from .depth_pick_release import COMMAND, RELEASE, verify_release
from .depth_pick_motion import GraspMotionEvidence

STAGES = {'waiting_for_prepare':'相机启动中', 'prepare':'观察姿态',
          'observing':'识别蓝色方块', 'prepared':'观察就绪',
          'auto_grasp':'等待稳定目标', 'search':'寻找蓝色方块',
          'centering':'自动对准', 'centered':'已对准', 'precheck':'检查抓取路径',
          'ready':'抓取路径就绪', 'open_gripper':'张开夹爪',
          'move_to_pregrasp':'接近方块', 'approach':'前伸夹取',
          'close_gripper':'夹紧方块', 'move_to_hold_pose':'收拢保持',
          'holding_unconfirmed':'保持姿态已到位，等待夹稳检查',
          'auto_grasp_complete':'夹取动作完成，交还机械臂控制权',
          'stopped':'抓取已停止，夹爪保持'}


def valid_request(value, backend_id, wall):
    if not isinstance(value, dict) or value.get('schema') != SCHEMA or value.get('backend_id') != backend_id:
        return False
    if value.get('command') not in {'start', 'cancel', 'confirm_holding', 'confirm_empty'}:
        return False
    if any(not isinstance(value.get(k), str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,96}', value[k])
           for k in ('client_id', 'request_id')):
        return False
    stamp, expires = value.get('stamp_unix'), value.get('expires_unix')
    return (type(stamp) in (int, float) and type(expires) in (int, float)
            and math.isfinite(stamp) and math.isfinite(expires)
            and -.5 <= wall-stamp <= 3 and wall <= expires <= stamp+3.1)


def is_rgbd_grasp_launch(argv):
    """Match only the exact RGB-D launch command, never a camera or ROS helper."""
    for index, arg in enumerate(argv):
        if os.path.basename(arg) == 'ros2' and argv[index + 1:index + 4] == [
                'launch', 'rgbd_grasp', 'rgbd_grasp.launch.py']:
            return True
    return False


class OriginalGraspBackend(Node):
    def __init__(self):
        super().__init__('raicom_depth_pick_backend')
        self.backend_id = uuid.uuid4().hex
        self.lock = threading.RLock()
        self.seq = 0
        self.request = None
        self.state, self.stage, self.reason = 'idle', '等待抓取', ''
        self.released = True
        self.holding = False
        self.awaiting_confirmation = False
        self.empty_confirmed = True
        self.child = self.worker = None
        self.cancel = threading.Event()
        self.closing = threading.Event()
        self.terminal_event = threading.Event()
        self.original_terminal = None
        self.motion_evidence = GraspMotionEvidence()
        self.velocity_at = self.odom_at = self.zero_since = 0.
        self.base_zero = False
        self.moving_since = None
        self.nav_active = False
        self.last_image = 0.
        self.seen = set()
        self.runs = Path.home()/'raicom_tools/depth_pick_runs'
        self.runs.mkdir(parents=True, exist_ok=True)
        self.status_pub = self.create_publisher(String, PREFIX+'/status', 10)
        self.image_pub = self.create_publisher(CompressedImage, PREFIX+'/preview/compressed', qos_profile_sensor_data)
        self.create_subscription(String, PREFIX+'/request', self.receive, 10)
        self.create_subscription(String, '/grasp_once_node/status', self.grasp_status, 10)
        self.create_subscription(Twist, '/controller/cmd_vel', self.velocity, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odometry, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/raicom/navigation_active', lambda m: setattr(self, 'nav_active', m.data), latched)
        self.create_subscription(Image, '/rgbd_grasp/debug_image_3d', self.preview, qos_profile_sensor_data)
        self.stop_client = self.create_client(Trigger, '/grasp_once_node/stop')
        try:
            self.version = verify_release()
            self.version_error = ''
        except Exception as exc:
            self.version, self.version_error = None, str(exc)
        self.create_timer(.25, self.tick)

    def velocity(self, msg):
        now = time.monotonic()
        zero = all(abs(v) <= 1e-7 for v in (msg.linear.x, msg.linear.y, msg.linear.z,
                                          msg.angular.x, msg.angular.y, msg.angular.z))
        self.zero_since = self.zero_since if self.base_zero and zero else now
        self.velocity_at, self.base_zero = now, zero

    def odometry(self, msg):
        now = time.monotonic()
        self.odom_at = now
        v = msg.twist.twist
        moving = math.hypot(v.linear.x, v.linear.y) > .01 or abs(v.angular.z) > .02
        self.moving_since = (self.moving_since or now) if moving else None

    def stationary(self, starting=False):
        now = time.monotonic()
        publishers = self.get_publishers_info_by_topic('/controller/cmd_vel')
        # A short IMU angular spike from arm vibration is not a drive command.
        # Stale feedback, any nonzero command, or sustained motion still cancels.
        movement = self.moving_since is not None and (starting or now-self.moving_since >= .3)
        return bool(self.base_zero and now-self.zero_since >= .5 and now-self.velocity_at <= .5
                    and now-self.odom_at <= .5 and not movement and not self.nav_active
                    and len(publishers) == 1 and publishers[0].node_name == 'cmd_vel_guard')

    def arm_free(self):
        return all(not self.get_publishers_info_by_topic(topic) for topic in
                   ('/servo_controller', '/ros_robot_controller/bus_servo/set_state'))

    def previous_grasp_pids(self):
        """Find same-user native launch processes; ignore unrelated ROS nodes."""
        result = []
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                if entry.stat().st_uid != os.getuid():
                    continue
                argv = [part.decode(errors='replace') for part in
                        (entry/'cmdline').read_bytes().split(b'\0') if part]
                if is_rgbd_grasp_launch(argv):
                    result.append(int(entry.name))
            except (FileNotFoundError, ProcessLookupError, PermissionError):
                continue
        return sorted(result)

    def arm_owned_by_previous_grasp(self):
        if not self.previous_grasp_pids():
            return False
        publishers = [info for topic in
                      ('/servo_controller', '/ros_robot_controller/bus_servo/set_state')
                      for info in self.get_publishers_info_by_topic(topic)]
        return all(info.node_name == 'grasp_once_node' for info in publishers)

    def close_previous_grasp(self):
        pids = self.previous_grasp_pids()
        if not pids:
            return
        self.stage = '停止上一抓取实例'
        if self.stop_client.wait_for_service(timeout_sec=.5):
            future = self.stop_client.call_async(Trigger.Request())
            deadline = time.monotonic() + 2.0
            while not future.done() and time.monotonic() < deadline:
                if self.cancel.is_set() or self.closing.is_set():
                    raise RuntimeError('清理旧抓取实例时已取消')
                time.sleep(.02)
        for sig, timeout in ((signal.SIGINT, 3.0), (signal.SIGTERM, 2.0)):
            for pid in pids:
                if pid in self.previous_grasp_pids():
                    try:
                        os.kill(pid, sig)
                    except ProcessLookupError:
                        pass
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline and any(
                    pid in self.previous_grasp_pids() for pid in pids):
                if self.cancel.is_set() or self.closing.is_set():
                    raise RuntimeError('清理旧抓取实例时已取消')
                time.sleep(.1)
            if not any(pid in self.previous_grasp_pids() for pid in pids):
                return
        raise RuntimeError('旧 RGB-D 抓取进程未退出，未启动新实例')

    def ready(self):
        return bool(self.state == 'idle' and self.released and self.empty_confirmed
                    and not self.version_error and self.stationary(True)
                    and (self.arm_free() or self.arm_owned_by_previous_grasp()))

    def receive(self, msg):
        if len(msg.data) > 16384:
            return
        try:
            value = json.loads(msg.data)
        except ValueError:
            return
        if not valid_request(value, self.backend_id, time.time()):
            return
        key = (value['client_id'], value['request_id'])
        same = self.request and key == (self.request['client_id'], self.request['request_id'])
        with self.lock:
            if value['command'] == 'cancel':
                self.seen.add(key)  # Cancellation can arrive before start.
                if same and self.state in {'running', 'cancelling'}:
                    self.cancel.set()
                    self.state, self.reason = 'cancelling', '操作员取消'
                elif self.state == 'idle':
                    self.request = value
                    self.state, self.reason = 'cancelled', '启动前已取消'
                return
            if value['command'] == 'confirm_holding':
                if same and self.awaiting_confirmation and not self.cancel.is_set():
                    self.awaiting_confirmation = False
                    self.state, self.stage, self.holding = 'succeeded', '已确认夹稳', True
                    self.save_result()
                return
            if value['command'] == 'confirm_empty':
                if same and self.released and self.state in {'succeeded', 'failed', 'cancelled'}:
                    self.empty_confirmed, self.holding = True, False
                    self.state, self.stage, self.reason = 'idle', '等待下一次抓取', ''
                return
            if key in self.seen or self.state != 'idle':
                return
            self.seen.add(key)
            self.request = value
            timeout = value.get('timeout_s')
            if (value.get('target') != 'blue_cube' or type(timeout) not in (int, float)
                    or not math.isfinite(timeout) or not 1 <= timeout <= 120 or not self.ready()):
                self.state, self.reason = 'failed', '抓取条件未就绪：需要停车、空闲机械臂和固定版本'
                return
            self.cancel.clear()
            self.terminal_event.clear()
            self.original_terminal = None
            self.motion_evidence = GraspMotionEvidence()
            self.empty_confirmed = False
            self.state, self.stage, self.reason, self.released = 'running', '启动深度相机', '', False
            self.deadline = time.monotonic() + timeout
            token = hashlib.sha256((':'.join((self.backend_id, *key))).encode()).hexdigest()
            self.run_dir = self.runs/token
            self.run_dir.mkdir(exist_ok=False)
            (self.run_dir/'request.json').write_text(json.dumps(value, ensure_ascii=False), encoding='utf-8')
            self.worker = threading.Thread(target=self.run_original, daemon=True)
            self.worker.start()

    def grasp_status(self, msg):
        if (self.state not in {'running', 'cancelling'} or self.child is None
                or self.awaiting_confirmation or self.terminal_event.is_set() or len(msg.data) > 32768):
            return
        try:
            value = json.loads(msg.data)
        except ValueError:
            return
        if not isinstance(value, dict) or not isinstance(value.get('stage'), str):
            return
        # The native topic has no request ID. Accept only the sole native node
        # while our own fresh child launch is active, then attach our identity.
        publishers = self.get_publishers_info_by_topic('/grasp_once_node/status')
        if len(publishers) != 1 or publishers[0].node_name != 'grasp_once_node':
            return
        stage = value.get('stage')
        self.stage = STAGES.get(stage, self.stage)
        self.motion_evidence.observe(value)
        with (self.run_dir/'stages.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(at_unix=time.time(), **value), ensure_ascii=False)+'\n')
        if stage == 'stopped':
            self.reason = '原生抓取收到停止请求，夹爪保持'
            self.cancel.set()
        if stage in {'auto_grasp_complete', 'rejected', 'failed', 'cancelled', 'stopped'}:
            self.original_terminal = value
            self.terminal_event.set()

    def run_original(self):
        problem = ''
        stop_ack = False
        try:
            version = verify_release()
            if not self.stationary(True) or self.cancel.is_set():
                raise RuntimeError('启动前现场条件发生变化')
            self.close_previous_grasp()
            forbidden = {'grasp_once_node', 'target_to_ik_node', 'blue_block_3d_node', 'kinematics', 'aurora'}
            graph_deadline = time.monotonic() + 10.0
            while True:
                names = self.get_node_names()
                occupied = (forbidden.intersection(names)
                            or any('aurora' in name.lower() for name in names))
                if not occupied:
                    break
                if self.cancel.is_set() or self.closing.is_set():
                    raise RuntimeError('启动等待期间已取消')
                if time.monotonic() >= graph_deadline:
                    raise RuntimeError('已有相机或抓取实例，未重复启动')
                self.stage = '等待上一抓取实例退出'
                time.sleep(.1)
            if not self.stationary(True) or not self.arm_free():
                raise RuntimeError('等待抓取实例退出期间现场条件发生变化')
            (self.run_dir/'version.json').write_text(json.dumps(version, indent=2))
            with (self.run_dir/'launch.log').open('xb') as log:
                self.child = subprocess.Popen(COMMAND, stdout=log, stderr=subprocess.STDOUT,
                                              stdin=subprocess.DEVNULL, start_new_session=True)
                while not self.terminal_event.wait(.03):
                    if self.cancel.is_set() or self.closing.is_set():
                        break
                    if time.monotonic() >= self.deadline:
                        self.reason = '单次抓取超过 120 秒'
                        self.cancel.set()
                        break
                    if self.child.poll() is not None:
                        raise RuntimeError('抓取程序提前退出，查看本次日志')
        except Exception as exc:
            problem = str(exc)
        finally:
            child = self.child
            if child is not None:
                if self.stop_client.wait_for_service(timeout_sec=1.):
                    future = self.stop_client.call_async(Trigger.Request())
                    end = time.monotonic()+2.
                    while not future.done() and time.monotonic() < end:
                        time.sleep(.02)
                    stop_ack = bool(future.done() and future.result() and future.result().success)
                for sig, wait in ((signal.SIGINT, 3.), (signal.SIGTERM, 2.), (signal.SIGKILL, 1.)):
                    if child.poll() is not None:
                        break
                    try:
                        os.killpg(child.pid, sig)
                        child.wait(timeout=wait)
                    except (ProcessLookupError, subprocess.TimeoutExpired):
                        pass
            end = time.monotonic()+2.
            while not self.arm_free() and time.monotonic() < end:
                time.sleep(.05)
            with self.lock:
                self.released = self.arm_free() and (child is None or child.poll() is not None)
                self.child = None
                completed = self.motion_evidence.complete(self.original_terminal)
                if completed and not self.cancel.is_set() and not problem and self.released and stop_ack:
                    self.awaiting_confirmation = True
                    self.state, self.stage = 'running', '保持姿态已到位，等待夹稳检查'
                else:
                    self.state = 'cancelled' if self.cancel.is_set() else 'failed'
                    self.reason = self.reason or problem or self.motion_evidence.error or str((self.original_terminal or {}).get('detail') or '动作未完成')
                self.stop_ack = stop_ack
                self.save_result()

    def save_result(self):
        if hasattr(self, 'run_dir'):
            payload = dict(state=self.state, stage=self.stage, reason=self.reason,
                           holding=self.holding, arm_released=self.released, release=RELEASE,
                           original_terminal=self.original_terminal, motion_result=self.motion_evidence.packet(),
                           at_unix=time.time())
            (self.run_dir/'result.json').write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')

    def tick(self):
        with self.lock:
            active = self.state in {'running', 'cancelling'}
            if active and (time.monotonic() >= self.deadline or
                           (not self.awaiting_confirmation and not self.stationary())):
                self.cancel.set()
                self.reason = self.reason or '超时或车辆停车条件不再满足'
            if self.awaiting_confirmation and self.cancel.is_set():
                self.awaiting_confirmation = False
                self.state = 'cancelled'
                self.save_result()
            self.seq += 1
            ready = self.ready()
            reason = self.reason or self.version_error
            if self.state == 'idle' and not ready and not reason:
                reason = '等待车辆停稳及相机、机械臂空闲'
            packet = dict(schema=SCHEMA, backend_id=self.backend_id, seq=self.seq,
                          stamp_unix=time.time(), state=self.state, stage=self.stage,
                          reason=reason, ready=ready, arm_released=self.released,
                          holding=self.holding, awaiting_confirmation=self.awaiting_confirmation,
                          release=RELEASE, motion_result=self.motion_evidence.packet())
            if self.request:
                packet.update({k:self.request[k] for k in ('client_id', 'request_id')})
            self.status_pub.publish(String(data=json.dumps(packet, ensure_ascii=False)))

    def preview(self, msg):
        if self.child is None or time.monotonic()-self.last_image < .3:
            return
        self.last_image = time.monotonic()
        try:
            import cv2
            from cv_bridge import CvBridge
            frame = CvBridge().imgmsg_to_cv2(msg, 'bgr8')
            ok, data = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                packet = CompressedImage(header=msg.header, format='jpeg', data=data.tobytes())
                self.image_pub.publish(packet)
        except Exception as exc:
            self.get_logger().warning('预览帧转换失败：'+str(exc))


def main():
    from rclpy.signals import SignalHandlerOptions
    lock = open('/tmp/raicom_depth_pick_backend.lock', 'a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = OriginalGraspBackend()
    def close(*_):
        node.closing.set()
        node.cancel.set()
    signal.signal(signal.SIGINT, close)
    signal.signal(signal.SIGTERM, close)
    try:
        while not node.closing.is_set():
            rclpy.spin_once(node, timeout_sec=.05)
        # Keep processing stop-service responses while the worker shuts down.
        while node.worker is not None and node.worker.is_alive():
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        lock.close()


if __name__ == '__main__':
    main()
