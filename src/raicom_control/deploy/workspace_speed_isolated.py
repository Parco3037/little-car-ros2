#!/usr/bin/env python3
"""Verify editable speed values on private DDS 87, with virtual pose only.

Run with workspace_native_isolated.py and fixed_route_isolated.py beside it.
The existing harness redirects every velocity output to /isolated/*.
"""
import json
import time
import workspace_native_isolated as baseline
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath
from std_msgs.msg import Empty


class SpeedProbe(baseline.WorkspaceProbe):
    checked_custom_speeds = False

    def custom_speed_case(self):
        self.settings(far=2., near=1., fine=.5, near_distance=.1, turn=1.)
        self.integrate=False; self.enabled=False; self.spin(.65)
        self.x=self.y=self.yaw=0.; self.v=[0.,0.]; self.spin(.4)
        raw=[]
        sub=self.node.create_subscription(Twist, '/isolated/raw_cmd',
            lambda m: raw.append([m.linear.x,m.angular.z]), 10)
        goal=FollowPath.Goal()
        goal.controller_id='FollowPath'; goal.goal_checker_id='general_goal_checker'
        goal.path.header.frame_id='map'
        for x in (0., .025, .05):
            p=PoseStamped(); p.header=goal.path.header
            p.pose.position.x=x; p.pose.orientation.w=1.
            goal.path.poses.append(p)
        self.goal_id='custom-fine-speed-'+str(time.monotonic_ns()); self.samples=[]
        handle=self.wait(self.client.send_goal_async(goal)); assert handle.accepted
        future=handle.get_result_async(); self.enabled=True; self.spin(1.4)
        result={'case':'fine_0.5_distance_0.1_real_controller',
            'active_before_cancel':not future.done(),
            'raw_peak':max((row[0] for row in raw),default=0.),
            'guard_peak':max((row[1] for row in self.samples),default=0.)}
        self.enabled=False; self.stop.publish(Empty())
        answer=self.wait(future); self.spin(.7)
        result.update(status=answer.status,
            zero_tail=bool(self.samples) and all(abs(s[1])+abs(s[2])<1e-7 for s in self.samples[-8:]))
        print(json.dumps(result),flush=True)
        self.node.destroy_subscription(sub)
        assert result['active_before_cancel'] and abs(result['raw_peak']-.5)<1e-8,result
        assert result['guard_peak']>.1 and answer.status==5 and result['zero_tail'],result
        # Independent stages, distance below 6 cm, disabled distance, and values
        # outside the old recommendation ranges must survive all three nodes.
        self.settings(far=.25,near=1.2,fine=.5,near_distance=.03)
        self.settings(far=.25,near=.12,fine=.5,near_distance=0.)
        self.settings(fine=.005,near_distance=3.,turn=.01)
        self.settings()

    def run_case(self,*args,**kwargs):
        if not self.checked_custom_speeds:
            self.checked_custom_speeds=True
            self.custom_speed_case()
        return super().run_case(*args,**kwargs)


if __name__=='__main__':
    baseline.WorkspaceProbe=SpeedProbe
    baseline.main()
