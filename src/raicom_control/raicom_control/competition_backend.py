"""Competition companion: arm readback, HTTP classification and return to init.

No commands on startup. This module does not replace the frozen depth-pick release.
Run with --execute-calibrated-actions only after field calibration and acceptance.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformListener

from .competition_arm import CompetitionArm, TOPICS
from .competition_backend_logic import RequestGate, item_identity, valid_mapping, feedback_valid
from .competition_calibration import classify_frame
from .competition_named_program import load_program
from .vision_decision import decide_markers, decide_held_class
from .competition_io import SCHEMA, PREFIX, OPERATIONS, recent
from .competition_pick_contract import destination_from_pick
from .vision_http import VisionBackend


def decode(message):
    if len(message.data) > 65536: return None
    try:
        value = json.loads(message.data,parse_constant=lambda s: (_ for _ in ()).throw(ValueError(s)))
        return value if isinstance(value,dict) else None
    except ValueError: return None


class CompetitionBackend(Node):
    def __init__(self, calibration_path, run_root, execute=False):
        super().__init__('raicom_competition_backend')
        self.backend_id = uuid.uuid4().hex
        self.gate = RequestGate(self.backend_id)
        self.execute = execute
        self.root = Path(run_root); self.root.mkdir(parents=True,exist_ok=True)
        self.calibration = self.arm = None
        self.calibration_error = ''
        self.calibration_path = Path(calibration_path)
        self.seq, self.request_dir = 0, None
        self.phase, self.terminal = 'idle', None
        self.classified, self.transported, self.placed = {}, {}, {}
        self.used_slots = {}
        self.depth, self.feedback_health, self.feedback = {}, {}, None
        self.depth_at = self.feedback_at = 0.
        self.velocity_at = self.odom_at = self.zero_since = 0.
        self.base_zero, self.odom_zero, self.nav_active = False, False, False
        self.closing = False
        self.infer_future = None
        self.executor_pool = ThreadPoolExecutor(max_workers=1,thread_name_prefix='competition_http')
        self.vision = VisionBackend()
        self.buffer = Buffer(); self.listener = TransformListener(self.buffer,self)
        self.health_pub = self.create_publisher(String,PREFIX+'/health',10)
        self.result_pub = self.create_publisher(String,PREFIX+'/result',10)
        self.check_pub = self.create_publisher(String,PREFIX+'/placement_check',10)
        self.create_subscription(String,PREFIX+'/request',self.receive,10)
        self.create_subscription(String,PREFIX+'/lease',lambda m:self.gate.lease(decode(m),time.monotonic(),time.time()),10)
        self.create_subscription(String,PREFIX+'/placement_feedback',self.on_feedback,10)
        self.create_subscription(String,PREFIX+'/placement_health',self.on_feedback_health,10)
        self.create_subscription(String,'/raicom/depth_pick/status',self.on_depth,10)
        self.create_subscription(Twist,'/controller/cmd_vel',self.on_velocity,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        latched = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool,'/raicom/navigation_active',lambda m:setattr(self,'nav_active',m.data),latched)
        self.create_subscription(Empty,'/raicom/nav_stop',lambda m:self.stop('人工急停'),10)
        try:
            self.calibration = load_program(calibration_path)
            self.arm = CompetitionArm(self,self.calibration,self.append)
        except (ValueError,RuntimeError,OSError,KeyError,TypeError) as exc:
            self.calibration_error = str(exc)
        self.create_timer(.05,self.tick)
        self.create_timer(.25,self.health)

    def append(self, record):
        if self.request_dir is None: return
        with (self.request_dir/'events.jsonl').open('a',encoding='utf-8') as stream:
            stream.write(json.dumps(dict(record,stamp_unix=time.time()),ensure_ascii=False,allow_nan=False)+'\n')
            stream.flush(); os.fsync(stream.fileno())

    def on_velocity(self, msg):
        values = [msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z]
        now = time.monotonic(); zero = all(math.isfinite(v) and abs(v) <= 1e-7 for v in values)
        self.zero_since = self.zero_since if self.base_zero and zero else now
        self.base_zero, self.velocity_at = zero, now

    def on_odom(self, msg):
        v = msg.twist.twist
        self.odom_zero = all(math.isfinite(x) for x in (v.linear.x,v.linear.y,v.angular.z)) and math.hypot(v.linear.x,v.linear.y) <= .01 and abs(v.angular.z) <= .02
        self.odom_at = time.monotonic()

    def pose(self):
        try:
            t = self.buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
            age = (self.get_clock().now().nanoseconds-t.header.stamp.sec*10**9-t.header.stamp.nanosec)/1e9
            p, q = t.transform.translation,t.transform.rotation
            if not -.1 <= age <= .5: return None
            values = p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            return values if all(math.isfinite(v) for v in values) else None
        except Exception: return None

    def stationary_failure(self):
        now = time.monotonic()
        if not self.base_zero: return '底盘输出不是零速'
        if not self.odom_zero: return '里程计显示底盘仍在运动'
        if now-self.velocity_at > .5: return '底盘零速反馈已过期'
        if now-self.odom_at > .5: return '里程计反馈已过期'
        if now-self.zero_since < .5: return '底盘零速保持不足0.5秒'
        if self.nav_active: return '导航仍在执行'
        owners = self.get_publishers_info_by_topic('/controller/cmd_vel')
        if len(owners) != 1 or owners[0].node_name != 'cmd_vel_guard': return '底盘控制发布者不唯一或不是guard'
        return ''

    def stationary(self):
        # Robot-relative arm poses need physical standstill, not map localization.
        # World-position admission belongs to park placement below.
        self.safety_reason = self.stationary_failure()
        return not self.safety_reason

    def safe_to_operate(self):
        self.safety_reason = ''
        p = self.gate.pending
        if self.closing or p is None:
            self.safety_reason = '后端正在关闭或没有当前请求'; return False
        if not self.gate.check(time.monotonic()):
            self.safety_reason = self.gate.cancel_reason or '本轮租约未就绪'; return False
        if not self.stationary(): return False
        if p['operation'] in ('place','verify_placement'):
            point = self.calibration.value['route']['points'][p['point']]['reference']
            pose = self.pose()
            if pose is None:
                self.safety_reason = '放置点定位未就绪'; return False
            x,y,yaw = pose
            error = abs(math.atan2(math.sin(yaw-point['yaw_rad']),math.cos(yaw-point['yaw_rad'])))
            if math.hypot(x-point['x'],y-point['y']) > .02 or error > math.radians(10):
                self.safety_reason = '尚未到达指定放置点'; return False
        return True

    def arm_free(self): return all(not self.get_publishers_info_by_topic(t) for t in TOPICS)

    def on_depth(self, msg):
        v = decode(msg)
        if v and v.get('schema') == 'raicom.depth_pick/v1' and recent(v,time.time()):
            self.depth, self.depth_at = v,time.monotonic()

    def picked(self, request):
        v, identity = self.depth,request.get('pick_identity',{})
        return (isinstance(identity,dict) and all(isinstance(identity.get(k),str) and identity[k] for k in ('backend_id','client_id','request_id'))
            and time.monotonic()-self.depth_at <= 3 and v.get('state') == 'succeeded'
            and v.get('holding') is True and v.get('arm_released') is True
            and v.get('request_id') == request['pick_request_id']
            and all(identity[k] == v.get(k) for k in ('backend_id','client_id','request_id'))
            and self.count_publishers('/raicom/depth_pick/status') == 1)

    def on_feedback_health(self, msg):
        v = decode(msg)
        if v and v.get('schema') == SCHEMA and recent(v,time.time()):
            self.feedback_health, self.feedback_at = v,time.monotonic()

    def observer_ready(self):
        return bool(self.calibration and time.monotonic()-self.feedback_at <= 3
            and self.calibration.value['placement_feedback'].get('verified') is True
            and self.feedback_health.get('ready') is True
            and self.feedback_health.get('provider_id') == self.calibration.value['placement_feedback']['provider_id']
            and self.count_publishers(PREFIX+'/placement_health') == 1
            and self.count_publishers(PREFIX+'/placement_feedback') == 1
            and self.count_subscribers(PREFIX+'/placement_check') == 1)

    def on_feedback(self, msg):
        value = decode(msg)
        if self.phase in ('verify_placement','manual_placement') and self.observer_ready() and self.gate.pending and feedback_valid(
                value,self.gate.pending,self.calibration.value['placement_feedback']['provider_id'],time.time(),self.verify_started):
            self.feedback = value

    def receive(self, msg):
        value = decode(msg)
        admission = self.gate.admit(value,time.monotonic(),time.time())
        if admission == 'cancel': self.stop('取消当前请求'); return
        if admission in ('confirm_placement','confirm_classification'):
            p = self.gate.pending
            if (not self.safe_to_operate() or not self.arm_free()
                    or any(value.get(k) != p.get(k) for k in ('profile_id','park','point','slot','placement_request_id'))): return
            if admission == 'confirm_placement' and self.phase == 'manual_placement':
                self.append(dict(kind='operator_placement_confirmation',request_id=p['request_id']))
                self.succeed(placement_confirmed=True,gripper_empty=True,confirmation_source='operator')
            elif admission == 'confirm_classification' and self.phase == 'manual_classify' and value.get('operator_label') in ('1','2'):
                self.classification_success(value['operator_label'],'operator')
            return
        if admission in ('invalid','duplicate','busy'): return
        if admission == 'cancelled_before_start':
            self.publish_result(value,'cancelled','启动前已取消',arm_released=self.arm_free()); return
        if admission == 'lease_missing':
            self.publish_result(value,'failed','缺少当前工作台租约',arm_released=self.arm_free()); return
        self.request_dir, self.terminal = None, None
        try:
            if not self.execute: raise RuntimeError('候选后端未启用实物动作')
            if self.calibration_error or self.calibration is None: raise RuntimeError('动作标定未就绪：'+self.calibration_error)
            if not self.calibration.unchanged(): raise RuntimeError('标定文件已变化，须停车后重新加载')
            if value['geometry_identity'] != self.calibration.value['geometry_identity']: raise RuntimeError('当前地图与动作标定不匹配')
            if not self.safe_to_operate(): raise RuntimeError(self.safety_reason or '底盘未停稳')
            if not self.arm_free(): raise RuntimeError('机械臂尚未交还')
            if not self.picked(value): raise RuntimeError('没有当前请求已夹稳及控制权交还的抓取结果')
            token = hashlib.sha256((value['client_id']+':'+value['request_id']).encode()).hexdigest()
            self.request_dir = self.root/token; self.request_dir.mkdir(exist_ok=False)
            self.append(dict(kind='admitted',request=value,calibration_sha256=self.calibration.sha256))
            self.terminal = self.feedback = None
            key = item_identity(value); operation = value['operation']
            if operation == 'classify':
                if not valid_mapping(value.get('mapping'),value): raise RuntimeError('分类缺少本轮有效园区映射')
                if value.get('classification_mode')=='pick_result':
                    self.classification_decision=None
                    try:
                        destination=destination_from_pick(self.depth,value)
                    except ValueError as exc:
                        self.append(dict(kind='pick_destination_unavailable',reason=str(exc)))
                        self.stop(str(exc),failed=True)
                    else:
                        self.append(dict(kind='pick_destination_accepted',result=destination))
                        self.succeed(**destination)
                    self.progress();return
                self.labels = []; self.infer_count = 0; self.infer_next_at = 0.
                self.classification_decision=None
                if value.get('classification_mode')=='post_grasp_pose':
                    if not getattr(self.calibration,'command_wrist',False):
                        raise RuntimeError('夹取后识别需要已配置的指定识别姿态')
                    self.arm.start(self.calibration.classify_pose,
                                   [('recognition',self.calibration.classify_pose,'recognition')])
                else:
                    self.arm.start(self.calibration.value['classification']['pose'],[])
                self.phase = 'classify_pose'
            elif operation == 'transport':
                if key not in self.classified: raise RuntimeError('当前物品尚未完成有效分类')
                entry, program = self.calibration.program(value); self.arm.start(entry,program); self.phase = operation
            elif operation == 'place':
                classified = self.classified.get(key,{})
                if key not in self.transported or classified.get('park') != value.get('park') or classified.get('point') != value.get('point'):
                    raise RuntimeError('当前物品尚未运输或园区与分类不符')
                entry, program = self.calibration.program(value)
                candidates=[]
                if getattr(self.calibration,'command_wrist',False):
                    occupied=self.used_slots.get((value['client_id'],value['task_id'],value['park']),set())
                    if value['slot'] in occupied:raise RuntimeError('该放置位置已使用')
                    slots=list(range(value['slot'],5))+list(range(1,value['slot']))
                    for slot in slots:
                        if slot in occupied:continue
                        profile=self.calibration.value['config']['placements'][value['park']][slot-1]
                        _,steps=self.calibration.program(dict(value,slot=slot,profile_id=profile))
                        candidates.append(dict(slot=slot,profile_id=profile,program=steps))
                self.arm.start(entry,program,placement_candidates=candidates);self.phase=operation
            else:
                place = self.placed.get(key,{})
                if (place.get('request_id') != value.get('placement_request_id')
                        or any(place.get(k) != value.get(k) for k in ('profile_id','park','point','slot'))):
                    raise RuntimeError('没有本物品对应落点的放置执行回执')
                self.phase, self.verify_started = operation,time.time()
                self.check_pub.publish(String(data=json.dumps(dict(value,stamp_unix=self.verify_started,
                    expires_unix=self.verify_started+min(60,value['timeout_s']),action_evidence=place['evidence']),ensure_ascii=False)))
            self.progress()
        except Exception as exc: self.stop(str(exc),failed=True)

    def publish_result(self, request, status, reason='', **fields):
        packet = dict(request,status=status,reason=reason,stamp_unix=time.time(),**fields)
        self.result_pub.publish(String(data=json.dumps(packet,ensure_ascii=False,allow_nan=False)))

    def progress(self):
        if self.gate.pending:
            labels = {'manual_placement':'自动落位未能确认，请检查本件落点及空爪后人工确认',
                      'manual_classify':'自动分类未能确认，请人工核对本件T/G'}
            stage=labels.get(self.phase,self.phase)
            candidates=getattr(self.arm,'placement_candidates',[])
            if self.phase=='place' and candidates:
                idx=min(self.arm.placement_candidate,len(candidates)-1)
                stage=f"放置姿态 {candidates[idx]['profile_id']} · 第 {max(1,self.arm.placement_attempt)}/20 次"
            self.publish_result(self.gate.pending,'running',stage=stage,
                manual_required=self.phase in ('manual_placement','manual_classify'),arm_released=self.arm_free(),
                decision=getattr(self,'classification_decision',None) if self.phase in ('classify','manual_classify') else None)

    def classification_success(self, label, source):
        p = self.gate.pending
        park = p['mapping']['label_to_park'][label]
        point = p['mapping']['category_to_point']['T' if label == '1' else 'G']
        self.append(dict(kind='classification_confirmed',label=label,source=source))
        self.succeed(label=label,park=park,point=point,confirmed_frames=(1 if p.get('classification_mode')=='post_grasp_pose' else 3) if source != 'operator' else 0,
                     classification_source=source,decision=getattr(self,'classification_decision',None))

    def stop(self, reason, failed=False):
        if self.gate.pending is None: return
        self.gate.cancel_reason = str(reason)
        self.terminal = ('failed' if failed else 'cancelled',str(reason),{})
        if self.arm and self.arm.busy: self.arm.fail(reason)
        self.phase = 'stopping'

    def succeed(self, **fields):
        self.terminal = ('success','',fields)
        self.phase = 'finishing'

    def finish(self):
        if self.arm and self.arm.busy: return
        if self.arm: self.arm.release()
        if not self.arm_free(): return  # DDS teardown must actually become visible.
        if self.infer_future is not None:
            if not self.infer_future.done(): return
            self.infer_future = None  # Discard late HTTP result after cancellation.
        if self.terminal is None: return
        status, reason, fields = self.terminal
        request = self.gate.pending
        if status == 'success' and not self.gate.check(time.monotonic()):
            status,reason,fields = 'cancelled',self.gate.cancel_reason,{}
        evidence = str(self.request_dir/'events.jsonl') if self.request_dir else ''
        fields = dict(fields,arm_released=True,evidence=evidence)
        self.append(dict(kind='terminal',status=status,reason=reason,result=fields,request=request))
        packet = dict(request,**fields)
        key = item_identity(request)
        if status == 'success':
            if request['operation'] == 'classify': self.classified[key] = packet
            elif request['operation'] == 'transport': self.transported[key] = packet
            elif request['operation'] == 'place':
                if fields.get('completion_criterion')=='arrival_and_action_readback':
                    slot=fields.get('executed_slot',request['slot'])
                    self.used_slots.setdefault((request['client_id'],request['task_id'],request['park']),set()).add(slot)
                    # This item is complete; no later visual-verification request follows.
                    for cache in (self.classified,self.transported,self.placed):cache.pop(key,None)
                else:self.placed[key] = packet
            elif request['operation'] == 'verify_placement':
                for cache in (self.classified,self.transported,self.placed): cache.pop(key,None)
        self.publish_result(request,status,reason,**fields)
        self.gate.pending = self.terminal = self.request_dir = None
        self.phase = 'idle'

    def tick(self):
        if self.gate.pending is None: return
        try:
            if self.phase in ('stopping','finishing'):
                if self.arm and self.arm.busy: self.arm.poll()
                self.finish(); return
            if not self.safe_to_operate(): raise RuntimeError(self.gate.cancel_reason or self.safety_reason or '底盘未停稳')
            if self.arm and self.arm.busy: self.arm.poll()
            if self.arm and self.arm.state in ('failed','stopping'): raise RuntimeError(self.arm.error)
            p = self.gate.pending
            if self.phase == 'classify_pose' and self.arm.state == 'complete':
                self.arm.release()
                self.phase = 'classify_release'
            if self.phase == 'classify_release':
                if not self.arm_free():return
                self.phase = 'classify'
            if self.phase == 'classify':
                if not self.arm_free(): raise RuntimeError('分类期间机械臂被其他程序占用')
                if self.infer_future is None:
                    if time.monotonic()>=getattr(self,'infer_next_at',0.):
                        self.infer_future = self.executor_pool.submit(self.vision.perform,'infer')
                elif self.infer_future.done():
                    try: reply = self.infer_future.result()
                    except Exception as exc:
                        self.append(dict(kind='classification_unavailable',reason=str(exc)))
                        self.infer_future = None
                        self.infer_count += 1
                        self.infer_next_at=time.monotonic()+.5
                        if self.infer_count>=20:self.stop('自动分类累计20次不可用：'+str(exc),failed=True)
                        return
                    self.infer_future = None
                    self.infer_count += 1
                    payload = reply['payload']; self.append(dict(kind='classification_frame',payload=payload))
                    profile=self.calibration.value['classification']
                    decision=decide_markers(payload,roi=profile.get('roi'),expected_size=profile.get('image_size'))
                    if decision['label'] is None:
                        fallback=decide_held_class(payload)
                        if fallback is not None:
                            decision.update(status='selected',label=fallback['label'],
                                            confidence=fallback['confidence'],
                                            reason='无有效框，采用本帧YOLO最高置信度类别',
                                            classification_source=fallback['source'])
                    self.append(dict(kind='classification_decision',decision=decision))
                    self.classification_decision=decision
                    label=decision['label']
                    if label is None: self.labels = []
                    elif self.labels and self.labels[-1] != label: self.labels = [label]
                    else: self.labels.append(label)
                    if len(self.labels) >= (1 if p.get('classification_mode')=='post_grasp_pose' else 3):
                        self.classification_success(label,decision.get('classification_source') or
                            profile.get('decision_source','http_yolo_held_roi'))
                    elif self.infer_count >= 20:self.stop('自动分类累计20次未取得有效结果',failed=True)
            elif self.phase in ('transport','place') and self.arm.state == 'complete':
                fields = dict(physical_readback_verified=True,servo_positions=dict(self.arm.positions))
                if getattr(self.calibration,'command_wrist',False):
                    fields.update(physical_readback_verified=False,
                        init_command_completed=self.arm.init_command_completed,
                        init_physical_pose_verified=False,holding_readback_verified=True,
                        action_readback_steps=list(self.arm.readback_steps))
                if self.phase == 'transport': fields['holding'] = True
                else:
                    fields['phases_completed'] = list(self.arm.completed)
                    if getattr(self.calibration,'command_wrist',False):
                        fields.update(return_pose='init',completion_criterion='arrival_and_action_readback',
                            confirmation_source='placement_action_readback',physical_placement_observed=False)
                        candidates=getattr(self.arm,'placement_candidates',[])
                        if candidates:
                            actual=candidates[self.arm.placement_candidate]
                            fields.update(executed_slot=actual['slot'],executed_profile_id=actual['profile_id'],
                                placement_attempt=self.arm.placement_attempt,placement_failures=self.arm.placement_failures)
                self.succeed(**fields)
            elif self.phase in ('verify_placement','manual_placement'):
                if self.feedback is not None and self.observer_ready():
                    self.append(dict(kind='physical_placement_feedback',feedback=self.feedback))
                    self.succeed(placement_confirmed=True,gripper_empty=True,physical_feedback=self.feedback,confirmation_source='physical_observation')
                elif time.time()-self.verify_started >= 12:self.stop('自动放置反馈超时',failed=True)
        except Exception as exc: self.stop(str(exc),failed=True)

    def health(self):
        # Reload edited bindings only while fully idle. Never swap an active program.
        if self.phase=='idle' and self.gate.pending is None and (self.arm is None or not self.arm.busy):
            try:
                if self.calibration is None or not self.calibration.unchanged():
                    updated=load_program(self.calibration_path)
                    if self.arm is None:self.arm=CompetitionArm(self,updated,self.append)
                    else:self.arm.calibration=updated
                    self.calibration=updated;self.calibration_error=''
                    self.classified.clear();self.transported.clear();self.placed.clear()
            except (ValueError,RuntimeError,OSError,KeyError,TypeError) as exc:self.calibration_error=str(exc)
        self.seq += 1
        error = self.calibration_error
        if not error and not self.execute: error = '候选待现场标定验收，实物动作未启用'
        ready = bool(not error and self.arm and self.gate.pending is None and self.stationary() and self.arm_free())
        packet = dict(schema=SCHEMA,backend_id=self.backend_id,seq=self.seq,stamp_unix=time.time(),ready=ready,
            arm_released=self.arm_free(),capabilities=sorted(OPERATIONS) if not error else [],reason=error,
            stage=self.phase,catalog=self.calibration.catalog if self.calibration else {},
            confirmation_mode='automatic_only',automatic_placement_available=self.observer_ready(),
            placement_confirmation_required=not getattr(self.calibration,'command_wrist',False),
            placement_feedback_source=('placement_action_readback' if getattr(self.calibration,'command_wrist',False)
                                       else 'external_physical_observer'))
        self.health_pub.publish(String(data=json.dumps(packet,ensure_ascii=False)))
        if self.gate.pending: self.progress()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration',default=str(Path.home()/'raicom_runtime/competition/calibration.json'))
    parser.add_argument('--runs',default=str(Path.home()/'raicom_runtime/competition/runs'))
    parser.add_argument('--execute-calibrated-actions',action='store_true')
    args = parser.parse_args()
    lock = open('/tmp/raicom_competition_backend.lock','a')
    try: fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError: return
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = CompetitionBackend(args.calibration,args.runs,args.execute_calibrated_actions)
    def stop(*_): node.closing = True; node.stop('比赛后端关闭')
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    try:
        while not node.closing or node.gate.pending:
            rclpy.spin_once(node,timeout_sec=.05)
    finally:
        node.executor_pool.shutdown(wait=True,cancel_futures=True)
        node.destroy_node(); rclpy.shutdown(); lock.close()


if __name__ == '__main__': main()
