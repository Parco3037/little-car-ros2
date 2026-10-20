"""Eight real deliveries; all calibration and onsite route gates fail closed."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import signal
from pathlib import Path
import time
import uuid
import rclpy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from .delivery_protocol import DeliveryTask
from .delivery_arm import ArmSequence,PHASES,validate_calibration
from .fixed_route_runner import FixedRouteNode,load_bundle
from .field_tool_navigation_logic import NavigationTarget,arrival_errors
from .recognition_station import MarkerConsensus,infer_once


class DeliveryNode(FixedRouteNode):
    def __init__(self,bundle,map_file,receipt,calibration):
        super().__init__(bundle,map_file,receipt)
        self.task=DeliveryTask('delivery-'+uuid.uuid4().hex);self.calibration=calibration
        self.pick_pub=self.create_publisher(String,'/raicom/pick/request',10)
        self.status_pub=self.create_publisher(String,'/raicom/task/status',10)
        self.create_subscription(String,'/raicom/pick/result',self.pick_result,10)
        self.create_subscription(String,'/raicom/place/confirmation',self.place_confirmation,10)
        self.arm=ArmSequence(self);self.consensus=MarkerConsensus();self.pool=ThreadPoolExecutor(max_workers=1)
        self.infer=None;self.nav_target=None;self.place=None;self.stage_started=time.monotonic()
        self.last_stage=None;self.handling_arm=False

    def publish_pick(self,request):
        if request:self.pick_pub.publish(String(data=json.dumps(request)));self.append({'kind':'pick_request',**request})

    def pick_result(self,msg):
        try:result=json.loads(msg.data)
        except ValueError:return
        accepted=self.task.pick_result(result,time.monotonic())
        self.append({'kind':'pick_result','accepted':accepted,'result':result})

    def place_confirmation(self,msg):
        try:result=json.loads(msg.data)
        except ValueError:return
        if self.task.stage!='placing' or self.arm.state!='complete' or not self.place:return
        if (not isinstance(result,dict) or any(result.get(k)!=self.place[k] for k in
                ('task_id','placement_id','round','park','slot'))):return
        if result.get('physical_placement_confirmed') is not True:return
        accepted=self.task.placed(dict(self.place,success=True,physical_placement_confirmed=True,
                                       servo_readback_ok=True,gripper_released=True))
        self.append({'kind':'placement_confirmation','accepted':accepted,'confirmation':result})
        self.place=None;self.handling_arm=False

    def fail_task(self,reason):
        self.navigation.stop();self.publish_pick(self.task.fail(reason))
        if self.arm.state not in ('idle','complete','failed'):self.arm.fail(reason)

    def begin_navigation(self,target):
        self.start_legs([self.task.location+'->'+target]);self.nav_target=target

    def tick_task(self):
        now=time.monotonic()
        if self.task.stage in ('failed','complete'):return
        if self.last_stage!=self.task.stage:
            self.last_stage=self.task.stage;self.stage_started=now
            self.append({'kind':'task_status',**self.task.status()})
        self.status_pub.publish(String(data=json.dumps(self.task.status())))
        if self.heartbeat_at is None or now-self.heartbeat_at>.5 or self.external_stop_requested:
            self.fail_task('operator heartbeat lost or emergency stop');return
        if self.nav_target:
            self.poll_fixed()
            if self.navigation.state==self.navigation.SUCCEEDED:
                self.task.arrived(self.nav_target,self.navigation.arrivals[-1]);self.nav_target=None
            elif self.navigation.state in (self.navigation.FAILED,self.navigation.STOPPED):
                self.fail_task(self.navigation.last_error)
            return
        if not self.zero_velocity_hold_ok():self.fail_task('stationary stage lost zero hold');return
        stage=self.task.stage
        if stage=='await_start':
            pose=self.capture_current_pose();target=NavigationTarget('start','start',*self.bundle['endpoints']['start'],.02,20.)
            d,yaw=arrival_errors(pose,target)
            if d>.02 or yaw>10:raise RuntimeError('Robot must begin at surveyed start pose')
            self.append({'kind':'start_pose_verified','pose':pose,'position_error_m':d,'yaw_error_deg':yaw})
            self.task.location='start';self.task.stage='to_recognition'
        elif stage in ('to_recognition','to_pick','transport'):
            self.begin_navigation('shibie' if stage=='to_recognition' else 'pick' if stage=='to_pick' else self.task.target_park)
        elif stage=='recognition':
            if now-self.stage_started>60:raise RuntimeError('marker consensus timed out')
            if self.infer is None:self.infer=self.pool.submit(infer_once)
            elif self.infer.done():
                try:reply=self.infer.result()
                except Exception as exc:reply={'payload':{},'error':str(exc)}
                self.infer=None;self.append({'kind':'recognition_frame',**reply})
                mapping=self.consensus.observe(reply['payload'],stopped=self.zero_velocity_hold_ok())
                if mapping:self.task.set_mapping(mapping)
        elif stage=='pick_ready':self.publish_pick(self.task.request_pick(now))
        elif stage=='picking':
            cancel=self.task.exchange.tick(now)
            if cancel:self.publish_pick(cancel);self.fail_task('pick timeout')
            elif self.task.exchange.state=='failed':self.fail_task(self.task.exchange.reason)
        elif stage=='transport_pose':
            if not self.handling_arm:
                self.arm.start([('transport',self.calibration['transport'])]);self.handling_arm=True
            self.arm.poll()
            if self.arm.state=='complete':
                self.task.transport_ready(True,self.task.exchange.accepted['holding']);self.handling_arm=False
            elif self.arm.state=='failed':self.fail_task(self.arm.error)
        elif stage=='placement_ready':
            self.place=self.task.placement_request()
            program=self.calibration['placements'][str(self.place['slot'])]
            self.arm.start([(p,program[p]) for p in PHASES]);self.handling_arm=True
            self.append({'kind':'placement_request',**self.place})
        elif stage=='placing':
            self.arm.poll()
            if self.arm.state=='failed':self.fail_task(self.arm.error)
            elif now-self.stage_started>120:self.fail_task('physical placement confirmation timed out')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('bundle','route','map','receipt','calibration'):p.add_argument('--'+key,required=True,type=Path)
    a,ros=p.parse_known_args();bundle=load_bundle(a.bundle,a.route,a.map)
    calibration=validate_calibration(json.loads(a.calibration.read_text(encoding='utf-8')),bundle['content_sha256'])
    with a.receipt.open('x',encoding='utf-8') as f:f.write(json.dumps({'kind':'delivery_start','bundle_sha256':bundle['content_sha256']})+'\n')
    rclpy.init(args=ros,signal_handler_options=SignalHandlerOptions.NO)
    node=DeliveryNode(bundle,a.map,a.receipt,calibration)
    previous={sig:signal.signal(sig,lambda *_:setattr(node,'external_stop_requested',True))
              for sig in (signal.SIGINT,signal.SIGTERM)}
    try:
        deadline=time.monotonic()+30.
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if node.external_stop_requested:raise RuntimeError('operator stop during startup')
            if node.navigation_ready() and node.readbacks_ready() and node.zero_velocity_hold_ok() and node.heartbeat_at is not None:break
        else:raise RuntimeError('Delivery prerequisites not ready')
        while rclpy.ok() and node.task.stage not in ('failed','complete'):
            rclpy.spin_once(node,timeout_sec=.02);node.tick_task()
        if node.task.stage=='failed':raise RuntimeError(node.task.reason)
    except Exception as exc:
        node.fail_task(str(exc));raise
    finally:
        if node.arm.state not in ('idle','complete','failed'):node.arm.fail('task shutting down')
        node.navigation.stop();node.publish_pick(node.task.exchange.cancel('task ended'))
        until=time.monotonic()+5.
        while time.monotonic()<until and node.navigation.cleanup_pending:
            rclpy.spin_once(node,timeout_sec=.02);node.navigation.poll()
        node.status_pub.publish(String(data=json.dumps(node.task.status())))
        node.append({'kind':'delivery_terminal',**node.task.status(),'cancellation_pending':node.navigation.cleanup_pending})
        node.pool.shutdown(wait=True,cancel_futures=True)
        node._health_stop.set();node._health_thread.join(timeout=1.)
        node.destroy_node();rclpy.shutdown()
        for sig,handler in previous.items():signal.signal(sig,handler)


if __name__=='__main__':main()
