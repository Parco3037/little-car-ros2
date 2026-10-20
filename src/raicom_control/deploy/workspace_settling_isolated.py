#!/usr/bin/env python3
"""Native settling and transient braking regressions in private DDS only."""
import inspect,json,time
import workspace_native_isolated as base
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from std_msgs.msg import Empty

class SettlingProbe(base.WorkspaceProbe):
    def __init__(self):
        super().__init__();self.reported_velocity=None
        publish=self.odom.publish
        def odometry(message):
            if self.reported_velocity is not None:
                message.twist.twist.linear.x,message.twist.twist.angular.z=self.reported_velocity
            publish(message)
        self.odom.publish=odometry

    def send(self,x):
        goal=FollowPath.Goal();goal.controller_id='FollowPath';goal.goal_checker_id='general_goal_checker'
        goal.path.header.frame_id='map'
        for i in range(11):
            p=PoseStamped();p.header=goal.path.header;p.pose.position.x=x*i/10;p.pose.orientation.w=1.
            goal.path.poses.append(p)
        self.goal_id='settling-'+str(time.monotonic_ns());self.samples=[]
        handle=self.wait(self.client.send_goal_async(goal));assert handle.accepted
        self.enabled=True
        return handle.get_result_async()

    def arrival_case(self):
        self.enabled=False;self.integrate=False;self.spin(.7)
        self.x=self.y=self.yaw=0.;self.v=[0.,0.];self.reported_velocity=[.04,.15];self.spin(.4)
        future=self.send(0.);self.spin(.8)
        assert not future.done(),'Must not succeed while still moving through the pose window'
        self.reported_velocity=[0.,0.];self.spin(.2)
        assert not future.done(),'A momentary zero is not a stationary arrival'
        # The localization correction occurs during the hold, as in the live trace.
        self.x=-.045;self.reported_velocity=None;self.integrate=True;self.spin(.15)
        assert not future.done(),'Leaving XY tolerance must restore correction within this goal'
        answer=self.wait(future,15);self.enabled=False;self.spin(.7)
        assert answer.status==4 and abs(self.x)<=.02 and abs(self.y)<=.02
        assert any(v[1]>.001 for v in self.samples),'Same action never resumed position correction'
        print(json.dumps({'case':'moving_then_localization_shift_same_goal','status':answer.status,'xy_error':abs(self.x)}),flush=True)

    def braking_case(self,grid):
        self.enabled=False;self.integrate=False;self.spin(.7)
        self.x=self.y=self.yaw=0.;self.v=[0.,0.]
        column=round((.18-grid.info.origin.position.x)/grid.info.resolution)
        for row in range(grid.info.height):grid.data[row*grid.info.width+column]=100
        self.map_pub.publish(grid);self.reported_velocity=[.25,0.];self.spin(.7)
        future=self.send(.18);self.spin(.55)
        assert not future.done(),'Braking was aborted by the 0.3-second patience instead of held in the goal'
        assert self.samples and all(abs(s[1])+abs(s[2])<1e-7 for s in self.samples),'Unsafe command during braking hold'
        self.reported_velocity=None;self.v=[0.,0.];self.spin(.35)
        assert not future.done() and any(s[1]>.001 for s in self.samples),'Clear braking state did not resume same goal'
        self.enabled=False;self.stop.publish(Empty());assert self.wait(future).status==5;self.spin(.7)
        assert all(abs(s[1])+abs(s[2])<1e-7 for s in self.samples[-8:])
        print(json.dumps({'case':'transient_braking_preserves_goal_then_cancel','result':'PASS'}),flush=True)
        self.reported_velocity=[.25,0.];self.v=[0.,0.];self.spin(.4)
        future=self.send(.18);start=time.monotonic();answer=self.wait(future,5.)
        self.enabled=False;self.reported_velocity=None;self.spin(.7)
        assert answer.status==6,'Persistent unsafe braking must still abort'
        assert all(abs(s[1])+abs(s[2])<1e-7 for s in self.samples),'Unsafe motion during persistent blockage'
        print(json.dumps({'case':'persistent_braking_bounded_abort','status':answer.status,'elapsed_s':time.monotonic()-start}),flush=True)
        grid.data=[0]*len(grid.data);self.map_pub.publish(grid);self.spin(.8)

source=inspect.getsource(base.main)
source=source.replace("        probe.run_case('within_10deg'", "        probe.arrival_case()\n        probe.braking_case(grid)\n        probe.run_case('within_10deg'")
source=source.replace("        probe.obstacle_cases(grid)","        probe.settings(turn=1.)\n        probe.run_case('operator_turn_1rad',x=0.,yaw=-2.67)\n        probe.settings()\n        probe.obstacle_cases(grid)")
base.WorkspaceProbe=SettlingProbe
exec(compile(source,__file__,'exec'),base.__dict__)
if __name__=='__main__':base.main()
