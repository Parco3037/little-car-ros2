"""Optional automatic-confirmation adapter around the untouched released grasp.

The original command, physical execution and manual fallback are preserved. A
separate calibrated observer supplies scoped holding proof; absence is not success.
"""
import argparse
import fcntl
import hashlib
import json
from pathlib import Path
import signal
import time

import rclpy
from std_msgs.msg import String

from .depth_pick_runner import OriginalGraspBackend, valid_request
from .depth_pick_interface import SCHEMA, PREFIX
from .depth_pick_release import RELEASE
from .competition_confirmation import Confirmation


class CompetitionGraspBackend(OriginalGraspBackend):
    def __init__(self, provider_id=''):
        self.confirmation = None
        self.provider_id = provider_id
        super().__init__()
        self.check_pub = self.create_publisher(String,PREFIX+'/holding_check',10)
        self.create_subscription(String,PREFIX+'/holding_feedback',self.holding_feedback,10)

    def holding_feedback(self, message):
        if len(message.data) > 16384: return
        try: value = json.loads(message.data)
        except ValueError: return
        with self.lock:
            if (not self.confirmation or not self.awaiting_confirmation or self.cancel.is_set()
                    or not self.stationary(True) or not self.released or not self.arm_free()
                    or time.monotonic() >= self.deadline or self.count_publishers(PREFIX+'/holding_feedback') != 1): return
            if self.confirmation.observe(value,time.time()): self.finish_confirmation()

    def finish_confirmation(self):
        self.awaiting_confirmation = False
        self.state,self.stage,self.holding = 'succeeded',self.confirmation.reason,True
        self.save_result()

    def receive(self, message):
        if len(message.data)>16384:return
        try: value = json.loads(message.data)
        except ValueError: return
        if not valid_request(value,self.backend_id,time.time()): return
        if value.get('command') == 'confirm_holding':
            with self.lock:
                if (not self.confirmation or not self.awaiting_confirmation or self.cancel.is_set()
                        or not self.stationary(True) or not self.released or not self.arm_free()
                        or time.monotonic() >= self.deadline): return
                current = {k:value.get(k) for k in ('backend_id','client_id','request_id')}
                if self.confirmation.operator(current,time.time()): self.finish_confirmation()
            return
        if value.get('command') == 'cancel' and self.request and all(value.get(k) == self.request.get(k) for k in ('client_id','request_id')):
            if self.confirmation: self.confirmation.cancel()
        super().receive(message)
        if self.state == 'idle': self.confirmation = None
        elif self.state == 'running' and not self.awaiting_confirmation:
            if self.confirmation and self.confirmation.identity.get('request_id') != value.get('request_id'): self.confirmation = None

    def save_result(self):
        super().save_result()
        if hasattr(self,'run_dir') and self.confirmation:
            path = self.run_dir/'result.json'
            value = json.loads(path.read_text())
            value.update(confirmation_mode='automatic_with_manual_fallback',holding_confirmation=self.confirmation.proof)
            path.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')

    def tick(self):
        with self.lock:
            now = time.monotonic()
            active = self.state in ('running','cancelling')
            if active and (now >= self.deadline or not self.stationary()):
                self.cancel.set(); self.reason = self.reason or '超时或车辆停车条件不再满足'
            if self.awaiting_confirmation and self.cancel.is_set():
                self.awaiting_confirmation,self.state = False,'cancelled'
                if self.confirmation: self.confirmation.cancel()
                self.save_result()
            if self.awaiting_confirmation and not self.cancel.is_set():
                if self.confirmation is None:
                    identity = dict(backend_id=self.backend_id,client_id=self.request['client_id'],request_id=self.request['request_id'])
                    self.confirmation = Confirmation('holding',identity,self.provider_id,now,time.time())
                    self.check_pub.publish(String(data=json.dumps(self.confirmation.request())))
                    if not self.provider_id:
                        self.confirmation.fallback('自动夹稳检测尚未标定，请人工检查当前方块')
                self.confirmation.tick(now); self.stage = self.confirmation.reason
            self.seq += 1
            ready = self.ready(); reason = self.reason or self.version_error
            if self.state == 'idle' and not ready and not reason: reason = '等待车辆停稳及相机、机械臂空闲'
            manual = bool(self.confirmation and self.confirmation.state == 'manual_required')
            packet = dict(schema=SCHEMA,backend_id=self.backend_id,seq=self.seq,stamp_unix=time.time(),state=self.state,
                stage=self.stage,reason=reason,ready=ready,arm_released=self.released,holding=self.holding,
                awaiting_confirmation=self.awaiting_confirmation and manual,release=RELEASE,
                confirmation_mode='automatic_with_manual_fallback',automatic_holding_available=bool(self.provider_id
                    and self.count_publishers(PREFIX+'/holding_feedback')==1 and self.count_subscribers(PREFIX+'/holding_check')==1),
                holding_confirmation=self.confirmation.proof if self.confirmation else None,
                motion_result=self.motion_evidence.packet())
            if self.request: packet.update({k:self.request[k] for k in ('client_id','request_id')})
            self.status_pub.publish(String(data=json.dumps(packet,ensure_ascii=False)))


def holding_provider(path):
    """A configured observer ID is accepted only with field validation evidence."""
    path = Path(path).resolve()
    if not path.exists(): return ''
    data = json.loads(path.read_text(encoding='utf-8'))
    proof = data.get('evidence',{})
    relative = Path(proof.get('path',''))
    evidence = (path.parent/relative).resolve()
    if (data.get('schema') != 'raicom.holding_observer/v1' or data.get('verified') is not True
            or not isinstance(data.get('provider_id'),str) or not data['provider_id']
            or relative.is_absolute() or path.parent not in evidence.parents
            or hashlib.sha256(evidence.read_bytes()).hexdigest() != proof.get('sha256')):
        raise ValueError('自动夹稳检测尚无有效现场标定证据')
    return data['provider_id']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--holding-profile',default=str(Path.home()/'raicom_runtime/competition/holding_observer.json'))
    args = parser.parse_args()
    lock = open('/tmp/raicom_depth_pick_backend.lock','a')
    try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError: return
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try: provider = holding_provider(args.holding_profile)
    except (OSError,ValueError,TypeError): provider = ''
    node = CompetitionGraspBackend(provider)
    def stop(*_): node.closing.set(); node.cancel.set()
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    try:
        while not node.closing.is_set() or (node.worker is not None and node.worker.is_alive()):
            rclpy.spin_once(node,timeout_sec=.05)
    finally: node.destroy_node(); rclpy.shutdown(); lock.close()


if __name__ == '__main__': main()
