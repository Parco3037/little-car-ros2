#!/usr/bin/env python3
"""Real Humble BT feedback and workbench cancellation, fake actions, no base.

Private DDS 87; no controller, guard, driver, servo or velocity publisher is
started. Mock FollowPath fails and mock recovery services/actions succeed.
"""
import json,os,signal,subprocess,threading,time
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
assert os.environ.get('ROS_DOMAIN_ID')=='87' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
import rclpy,yaml
from rclpy.action import ActionClient,ActionServer,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from lifecycle_msgs.srv import ChangeState,GetState
from nav2_msgs.action import ComputePathToPose,ComputePathThroughPoses,FollowPath,NavigateToPose,Spin,Wait
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import PoseStamped,TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from raicom_control.field_tool_navigation_logic import NavigationController,NavigationTarget
from raicom_control.field_tool_ros import FieldToolNode
from raicom_control.field_tool_recovery import materialize_recovery_tree


def wait(future,seconds=10):
    deadline=time.monotonic()+seconds
    while not future.done() and time.monotonic()<deadline:time.sleep(.01)
    assert future.done(),'future timeout'
    return future.result()


def main():
    root=Path.cwd();rclpy.init();node=rclpy.create_node('recovery_isolated_probe')
    assert not node.get_publishers_info_by_topic('/controller/cmd_vel')
    group=ReentrantCallbackGroup();servers=[];services=[]
    def action(typ,name,success=True):
        def execute(handle):
            # Delay enough for the real navigator to publish each feedback.
            time.sleep(.18)
            result=typ.Result()
            if hasattr(result,'path'):
                result.path.header.frame_id='map'
                for x in (0.,.5):
                    p=PoseStamped();p.header=result.path.header;p.pose.position.x=x;p.pose.orientation.w=1.
                    result.path.poses.append(p)
            if handle.is_cancel_requested:handle.canceled()
            elif success:handle.succeed()
            else:handle.abort()
            return result
        servers.append(ActionServer(node,typ,name,execute_callback=execute,
            cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=group))
    for typ,name,success in [(ComputePathToPose,'compute_path_to_pose',True),
            (ComputePathThroughPoses,'compute_path_through_poses',True),(FollowPath,'follow_path',False),
            (Spin,'spin',True),(Wait,'wait',True)]:action(typ,name,success)
    def clear(request,response):time.sleep(.12);return response
    for name in ['local_costmap/clear_entirely_local_costmap','global_costmap/clear_entirely_global_costmap']:
        services.append(node.create_service(ClearEntireCostmap,name,clear,callback_group=group))
    tf=TransformBroadcaster(node);odom=node.create_publisher(Odometry,'/odom',10)
    def tick():
        t=TransformStamped();t.header.frame_id='map';t.child_frame_id='base_footprint'
        t.header.stamp=node.get_clock().now().to_msg();t.transform.rotation.w=1.;tf.sendTransform(t)
        o=Odometry();o.header=t.header;o.child_frame_id=t.child_frame_id;o.pose.pose.orientation.w=1.;odom.publish(o)
    node.create_timer(.04,tick,callback_group=group)
    executor=MultiThreadedExecutor(num_threads=6);executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    stream=(root/'recovery_bt.log').open('w');child=None
    try:
        params=yaml.safe_load((root/'config/nav2_params_workspace.yaml').read_text())
        path=root/'recovery_bt.yaml';path.write_text(yaml.safe_dump({'bt_navigator':params['bt_navigator']}))
        child=subprocess.Popen(['/opt/ros/humble/lib/nav2_bt_navigator/bt_navigator','--ros-args','--params-file',str(path)],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        client=node.create_client(ChangeState,'/bt_navigator/change_state')
        assert client.wait_for_service(timeout_sec=8.)
        for transition in (1,3):
            request=ChangeState.Request();request.transition.id=transition
            assert wait(client.call_async(request)).success
        state=node.create_client(GetState,'/bt_navigator/get_state')
        assert wait(state.call_async(GetState.Request())).current_state.id==3
        navclient=ActionClient(node,NavigateToPose,'/navigate_to_pose');assert navclient.wait_for_server(timeout_sec=3.)
        for limit in (0,10,17):
            leases=[]
            proxy=NS(navigate_client=navclient,external_stop_requested=False,
                fixed_tf_ok=lambda _:True,sensor_ready=lambda _:True,navigation_ready=lambda:True,
                set_motion_mission=lambda _:None,publish_goal_context=lambda *a,**k:None,
                publish_manual_suppressed=lambda _:None,publish_lease=lambda enabled:leases.append(enabled),
                publish_nav_stop=lambda:None,get_clock=node.get_clock,zero_velocity_hold_ok=lambda:True)
            def tolerances(*_):
                f=Future();f.set_result(NS(results=[NS(successful=True)]*4));return f
            proxy.set_controller_tolerances=tolerances
            controller=NavigationController(proxy);proxy.navigation=controller
            controller.max_recoveries=limit
            proxy.make_navigation_goal=lambda target:FieldToolNode.make_navigation_goal(proxy,target)
            target=NavigationTarget('same_target','test',.5,0.,0.,.02,10.)
            counts=[]
            with patch('raicom_control.field_tool_recovery.materialize_recovery_tree',
                       lambda value:materialize_recovery_tree(value,cache_dir=root/'trees')):
                controller.start([target],'recovery-isolated-'+str(limit))
                end=time.monotonic()+25
                while controller.state not in controller.TERMINAL and time.monotonic()<end:
                    controller.poll();counts.append(controller.feedback['recoveries']);time.sleep(.02)
                handle=controller.goal_handle
                answer=wait(controller.result_future) if controller.result_future else None
                controller.poll()
            result={'limit':limit,'max_feedback':max(counts),'state':controller.state,
                'error':controller.last_error,'action_status':answer.status if answer else None,
                'index':controller.index,'observed_counts':sorted(set(counts)),
                'last_lease':leases[-1],'hardware_motion':False}
            print(json.dumps(result),flush=True)
            assert controller.state==controller.FAILED and '超过上限' in controller.last_error,result
            assert max(counts)>limit and result['index']==0 and leases[-1] is False,result
            assert answer.status==5,result
            if limit>=10:assert 3 in counts and 6 in counts and 10 in counts,result
            assert not node.get_publishers_info_by_topic('/controller/cmd_vel')
            time.sleep(.4)
        print(json.dumps({'result':'PASS','scope':'real BT feedback and cancellation; mock actions; no physical motion'}),flush=True)
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:child.wait(timeout=6)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
        executor.shutdown();thread.join(timeout=3)
        for s in servers:s.destroy()
        stream.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
