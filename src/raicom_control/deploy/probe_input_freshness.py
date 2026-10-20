"""Bounded subscription-only timing check; no application publishers/actions."""
import argparse
from datetime import datetime, timezone
import json
import math
import time


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan, Imu
    from nav_msgs.msg import Odometry

    parser=argparse.ArgumentParser()
    parser.add_argument('--seconds',type=float,default=20)
    args=parser.parse_args()
    if not 5<=args.seconds<=60: parser.error('seconds must be 5..60')
    rclpy.init()
    node=rclpy.create_node('input_freshness_readonly',enable_rosout=False,start_parameter_services=False)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    start=time.monotonic()
    data=dict(utc_start=datetime.now(timezone.utc).isoformat(),topics={},commands=[],errors=[])

    def sample(msg,topic):
        now=node.get_clock().now().nanoseconds*1e-9
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        item=dict(t=time.monotonic()-start,stamp=stamp,age=now-stamp,ros_now=now,wall_time=time.time())
        if isinstance(msg,LaserScan):
            item.update(frame=msg.header.frame_id,count=len(msg.ranges),scan_time=msg.scan_time,
                        finite=sum(math.isfinite(v) for v in msg.ranges),range_max=msg.range_max)
        data['topics'].setdefault(topic,[]).append(item)

    def speed(msg):
        data['commands'].append([time.monotonic()-start,msg.linear.x,msg.linear.y,msg.linear.z,
                                 msg.angular.x,msg.angular.y,msg.angular.z])

    topics=[('/scan_raw',LaserScan),('/scan_stable',LaserScan),('/imu',Imu),('/odom',Odometry)]
    subs=[node.create_subscription(kind,topic,lambda m,t=topic:sample(m,t),qos) for topic,kind in topics]
    subs.append(node.create_subscription(Twist,'/controller/cmd_vel',speed,qos))
    try:
        while time.monotonic()-start<args.seconds:
            rclpy.spin_once(node,timeout_sec=.02)
        data['publishers']={t:[i.node_namespace.rstrip('/')+'/'+i.node_name for i in
                              node.get_publishers_info_by_topic(t)] for t,_ in topics}
        data['publishers']['/controller/cmd_vel']=[i.node_namespace.rstrip('/')+'/'+i.node_name for i in
                                                 node.get_publishers_info_by_topic('/controller/cmd_vel')]
        data['summary']={t:dict(n=len(rows),first=rows[0],last=rows[-1],
            min_age=min(r['age'] for r in rows),max_age=max(r['age'] for r in rows),
            max_gap=max((b['t']-a['t'] for a,b in zip(rows,rows[1:])),default=None)) for t,rows in data['topics'].items()}
        data['command_n']=len(data['commands'])
        data['command_nonzero']=sum(any(row[1:]) for row in data['commands'])
    except BaseException as error:
        data['errors'].append(type(error).__name__+': '+str(error)); raise
    finally:
        data['utc_end']=datetime.now(timezone.utc).isoformat()
        print(json.dumps(data,indent=2,allow_nan=False),flush=True)
        node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
