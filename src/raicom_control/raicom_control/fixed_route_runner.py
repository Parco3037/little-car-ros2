"""FollowPath segment queue reusing the field-tool acceptance and guard lease."""
import argparse
import hashlib
import json
import math
import signal
from pathlib import Path
import time
import uuid
import rclpy
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import FollowPath
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.parameter import parameter_value_to_python
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Empty
from .field_tool_ros import FieldToolNode
from .field_tool_navigation_logic import NavigationController, NavigationTarget, arrival_errors
from .fixed_route_assets import assert_binding
from .fixed_route_builder import validate_bundle


class FixedController(NavigationController):
    ACTION_NAME = 'FollowPath'
    def _prepare_goal(self,target,now):
        if now>self.deadline:
            self._fail('fixed-mode lifecycle/parameter readback timed out');return
        if not self.node.navigation_ready() or not self.node.readbacks_ready():return
        if not self.node.zero_velocity_hold_ok():return
        part=self.node.parts[target.name]
        current=self.node.capture_current_pose()
        start=NavigationTarget('source','source',*part['start'],.02,20.)
        distance,yaw=arrival_errors(current,start)
        if distance>.02 or yaw>10:
            self._fail('current pose is outside the saved segment start');return
        self.goal_id=f'{self.mission_id}:{self.index}:{time.monotonic_ns()}'
        self.feedback={'distance_remaining_m':None,'recoveries':0}
        identity=self.goal_id
        self.node.publish_goal_context(identity,active=True,mission_id=self.mission_id)
        self.send_future=self.node.navigate_client.send_goal_async(
            self.node.make_navigation_goal(target),feedback_callback=lambda msg:self._on_feedback(msg,identity))
        self.deadline=now+5.

    def _on_feedback(self,message,identity):
        if identity==self.goal_id and self.state not in self.TERMINAL:
            self.feedback['distance_remaining_m']=float(message.feedback.distance_to_goal)

    def _poll_active(self,target,now):
        if self.result_future is not None and self.result_future.done():
            self.node.record_action_end(self.goal_id,int(self.result_future.result().status))
        super()._poll_active(target,now)

    def _drain_stopped_goal(self):
        if self.result_future is not None and self.result_future.done():
            self.node.record_action_end(self.goal_id,int(self.result_future.result().status))
        super()._drain_stopped_goal()


class FixedRouteNode(FieldToolNode):
    def __init__(self,bundle,map_file,receipt):
        super().__init__()
        self.bundle=bundle;self.map_file=str(Path(map_file).resolve());self.receipt=Path(receipt)
        self.parts={};self.action_results=[];self.heartbeat_at=None
        self.create_subscription(Empty,'/raicom/task/heartbeat',self._heartbeat,1)
        self.navigate_client.destroy()
        self.navigate_client=ActionClient(self,FollowPath,'/follow_path')
        for client in self.navigation_state_clients.values():self.destroy_client(client)
        self.navigation_state_clients={name:self.create_client(GetState,'/'+name+'/get_state')
            for name in ('controller_server','velocity_smoother','map_server','amcl')}
        self.navigation=FixedController(self)
        from .fixed_route_profile import controller_profile
        profile=controller_profile()['controller_server']['ros__parameters']
        checks={'controller_plugins':['FixedForward','FixedReverse']}
        for section in ('general_goal_checker','FixedForward','FixedReverse'):
            for k,v in profile[section].items():checks[section+'.'+k]=v
        self.expected={
            'controller_server':checks,
            'velocity_smoother':{'max_velocity':[.25,0.,.35],'min_velocity':[-.08,0.,-.35],
                                 'velocity_timeout':.15},
            'local_costmap/local_costmap':{'plugins':['inflation_layer'], 'inflation_layer.enabled':False,'footprint_padding':0.0,
                'footprint':json.dumps(bundle['footprint']['points_xy_m'],separators=(',',':'))},
            'map_server':{'yaml_filename':self.map_file},
        }
        self.read_clients={key:self.create_client(GetParameters,'/'+key+'/get_parameters') for key in self.expected}
        self.pending={};self.verified={};self.readback={};self.read_error=''
        self._written_results=set()

    def _heartbeat(self,_msg):self.heartbeat_at=time.monotonic()

    def append(self,record):
        with self.receipt.open('a',encoding='utf-8') as f:
            f.write(json.dumps(record,ensure_ascii=False,allow_nan=False)+'\n');f.flush()

    def record_action_end(self,goal_id,status):
        if goal_id not in self._written_results:
            self._written_results.add(goal_id)
            self.append({'kind':'action_result','action':'FollowPath','goal_id':goal_id,'status':status,'monotonic':time.monotonic()})

    def record_navigation_arrival(self,report):
        self.append(dict(report,kind='stopped_arrival',action='FollowPath'))

    def readbacks_ready(self):
        now=time.monotonic()
        for key,client in self.read_clients.items():
            pending=self.pending.get(key)
            if pending:
                future,started=pending
                if future.done():
                    response=future.result();del self.pending[key]
                    values={name:parameter_value_to_python(v) for name,v in zip(self.expected[key],response.values)} if response else {}
                    self.readback[key]=values
                    if values==self.expected[key]:self.verified[key]=now
                    else:
                        self.verified.pop(key,None);self.read_error='parameter mismatch: '+key
                elif now-started>1.:
                    client.remove_pending_request(future);future.cancel();del self.pending[key]
                    self.verified.pop(key,None)
            if key not in self.pending and now-self.verified.get(key,-100)>.4 and client.service_is_ready():
                self.pending[key]=(client.call_async(GetParameters.Request(names=list(self.expected[key]))),now)
        return all(now-self.verified.get(key,-100)<=1. for key in self.expected)

    def make_navigation_goal(self,target):
        part=self.parts[target.name]
        goal=FollowPath.Goal();goal.path.header.frame_id='map'
        goal.path.header.stamp=self.get_clock().now().to_msg()
        goal.controller_id=part['controller_id'];goal.goal_checker_id='general_goal_checker'
        for x,y,yaw in part['path']:
            p=PoseStamped();p.header=goal.path.header;p.pose.position.x=float(x);p.pose.position.y=float(y)
            p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2)
            goal.path.poses.append(p)
        return goal

    def start_legs(self,legs):
        if self.navigation.state not in self.navigation.TERMINAL:raise RuntimeError('Navigation already active')
        targets=[];self.parts={}
        for leg in legs:
            for i,part in enumerate(self.bundle['legs'][leg]):
                name=f'{len(targets)}:{leg}:{i}';self.parts[name]=part
                targets.append(NavigationTarget(name,name,*part['end'],.02,20.,part['timeout_s']))
        self.navigation.start(targets,'fixed-'+uuid.uuid4().hex)
        self.navigation.deadline=time.monotonic()+20.

    def poll_fixed(self):
        if self.navigation.state not in self.navigation.TERMINAL:
            heartbeat=self.heartbeat_at is not None and time.monotonic()-self.heartbeat_at<=.5
            exclusive=self.control_chain_ok()
            conflicts=set(self.get_node_names()) & {'bt_navigator','behavior_server','planner_server'}
            if not heartbeat or not exclusive or conflicts:
                self.navigation._fail('operator heartbeat lost or control owners conflict')
            elif self.navigation.state==self.navigation.ACTIVE and not self.readbacks_ready():
                self.navigation._fail('fixed-mode parameters stale or changed')
        self.navigation.poll()

    def control_chain_ok(self):
        expected={'/controller/cmd_vel':'cmd_vel_guard',
                  '/raicom/nav2_staging/raw_cmd_vel':'controller_server',
                  '/raicom/nav2_staging/cmd_vel':'velocity_smoother',
                  '/raicom/motion_enable':self.get_name()}
        for topic,name in expected.items():
            owners=self.get_publishers_info_by_topic(topic)
            if len(owners)!=1 or owners[0].node_name!=name or owners[0].node_namespace!='/':return False
        owners=self.get_publishers_info_by_topic('/raicom/teleop_cmd')
        names=sorted(p.node_name for p in owners)
        return (names in (['nav_cmd_bridge'],['keyboard_input','nav_cmd_bridge'])
                and all(p.node_namespace=='/' for p in owners))


def load_bundle(bundle_file,route_file,map_file):
    import yaml
    bundle=json.loads(Path(bundle_file).read_text(encoding='utf-8'));validate_bundle(bundle)
    route=json.loads(Path(route_file).read_text(encoding='utf-8'))
    meta=yaml.safe_load(Path(map_file).read_text(encoding='utf-8'))
    image=Path(map_file).parent/meta['image']
    assert_binding(bundle,route,hashlib.sha256(Path(map_file).read_bytes()).hexdigest(),hashlib.sha256(image.read_bytes()).hexdigest())
    return bundle


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('bundle','route','map','receipt'):p.add_argument('--'+name,required=True,type=Path)
    p.add_argument('--legs',required=True,nargs='+');a,ros=p.parse_known_args()
    bundle=load_bundle(a.bundle,a.route,a.map)
    # Evidence is created exclusively before any ROS goal or lease exists.
    with a.receipt.open('x',encoding='utf-8') as f:f.write(json.dumps({'kind':'start','bundle_sha256':bundle['content_sha256']})+'\n')
    rclpy.init(args=ros,signal_handler_options=SignalHandlerOptions.NO)
    node=FixedRouteNode(bundle,a.map,a.receipt)
    previous={sig:signal.signal(sig,lambda *_:setattr(node,'external_stop_requested',True))
              for sig in (signal.SIGINT,signal.SIGTERM)}
    try:
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if node.external_stop_requested:raise RuntimeError('operator stop during startup')
            if (node.navigation_ready() and node.readbacks_ready() and node.heartbeat_at is not None
                    and time.monotonic()-node.heartbeat_at<=.5 and node.zero_velocity_hold_ok()):break
        else:raise RuntimeError('Fresh operator heartbeat, active services and fixed parameters required')
        node.append({'kind':'actual_parameters','values':node.readback})
        node.start_legs(a.legs)
        while rclpy.ok() and node.navigation.state not in node.navigation.TERMINAL:
            rclpy.spin_once(node,timeout_sec=.02);node.poll_fixed()
        node.append({'kind':'terminal','state':node.navigation.state,'reason':node.navigation.last_error})
        if node.navigation.state!=node.navigation.SUCCEEDED:raise RuntimeError(node.navigation.last_error)
    finally:
        node.navigation.stop()
        until=time.monotonic()+5.
        while time.monotonic()<until and node.navigation.cleanup_pending:
            rclpy.spin_once(node,timeout_sec=.02);node.navigation.poll()
        node.append({'kind':'cleanup','cancellation_pending':node.navigation.cleanup_pending,'zero_hold':node.zero_velocity_hold_ok()})
        node._health_stop.set();node._health_thread.join(timeout=1.)
        node.destroy_node();rclpy.shutdown()
        for sig,handler in previous.items():signal.signal(sig,handler)


if __name__=='__main__':main()
