"""Existing FieldArmIO on its own ROS executor, independent of Tk latency."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--execute-init',action='store_true',required=True)
    parser.add_argument('--result',type=Path,required=True)
    args=parser.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Empty
    from .field_tool_arm_io import FieldArmIO

    class InitNode(Node):
        def __init__(self):
            super().__init__('raicom_competition_init',enable_rosout=False)
            self.external_stop_requested=False
            self.velocity_at=self.odom_at=self.zero_since=0.
            self.base_zero=self.odom_zero=False
            self.create_subscription(Twist,'/controller/cmd_vel',self.velocity,qos_profile_sensor_data)
            self.create_subscription(Odometry,'/odom',self.odometry,qos_profile_sensor_data)
            self.create_subscription(Empty,'/raicom/nav_stop',self.stop,10)

        def stop(self,_):self.external_stop_requested=True

        def velocity(self,msg):
            now=time.monotonic()
            zero=all(math.isfinite(v) and abs(v)<=1e-7 for v in
                (msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z))
            if not (zero and self.base_zero):self.zero_since=now
            self.base_zero=zero;self.velocity_at=now

        def odometry(self,msg):
            v=msg.twist.twist;values=(v.linear.x,v.linear.y,v.angular.z)
            self.odom_zero=all(math.isfinite(x) for x in values) and math.hypot(*values[:2])<=.01 and abs(values[2])<=.02
            self.odom_at=time.monotonic()

        def zero_velocity_hold_ok(self):
            now=time.monotonic()
            return (self.base_zero and self.odom_zero and now-self.zero_since>=.5
                and now-self.velocity_at<=.5 and now-self.odom_at<=.5)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=InitNode();parent=os.getppid();events=[];arm=None
    def cancel(*_):node.external_stop_requested=True
    signal.signal(signal.SIGINT,cancel);signal.signal(signal.SIGTERM,cancel)
    result=dict(completed=False,reason='init未完成')
    try:
        arm=FieldArmIO(node,events.append)
        started=time.monotonic();issued=False
        while True:
            rclpy.spin_once(node,timeout_sec=.02)
            if os.getppid()!=parent or node.external_stop_requested:
                raise RuntimeError('回init已取消或工作台退出')
            if time.monotonic()-started>25:raise RuntimeError('回init超时')
            if not issued:
                if not node.zero_velocity_hold_ok():continue
                owners=node.get_publishers_info_by_topic('/controller/cmd_vel')
                if len(owners)!=1 or owners[0].node_name!='cmd_vel_guard':
                    raise RuntimeError('底盘guard控制权不唯一')
                arm.start('init_pose');issued=True
            arm.poll()
            if arm.state=='failed':raise RuntimeError(arm.error)
            if arm.state=='complete':
                result=dict(completed=True,positions=dict(arm.positions));break
    except Exception as exc:
        result['reason']=str(exc)
        if arm:arm.cancel(str(exc))
    finally:
        if arm:arm.release_control()
        result.update(events=events,at_unix=time.time())
        temp=args.result.with_suffix('.next');temp.write_text(json.dumps(result,ensure_ascii=False),encoding='utf-8');temp.replace(args.result)
        node.destroy_node();rclpy.shutdown()
    return 0 if result['completed'] else 1


if __name__=='__main__':raise SystemExit(main())
