#!/usr/bin/env python3
"""Synthetic scan/TF integration test, isolated from the real robot DDS domain."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

def main():
    if os.environ.get('ROS_DOMAIN_ID') != '87' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Requires private ROS_DOMAIN_ID=87 and ROS_LOCALHOST_ONLY=1')
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
    from sensor_msgs.msg import LaserScan
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=['legacy','patched'], required=True)
    parser.add_argument('--library-dir', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('synthetic_rotation_fixture', enable_rosout=False)
    pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
    tf = TransformBroadcaster(node)
    static_tf = StaticTransformBroadcaster(node)
    fixed = TransformStamped()
    fixed.header.stamp = node.get_clock().now().to_msg()
    fixed.header.frame_id = 'base_footprint'
    fixed.child_frame_id = 'test_laser'
    fixed.transform.rotation.w = 1.
    static_tf.sendTransform(fixed)
    received, truth = [], {}
    def pose(msg):
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        p,q = msg.pose.pose.position,msg.pose.pose.orientation
        y = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        received.append({'stamp':stamp,'xy':[p.x,p.y],'yaw':y,'truth_yaw':truth.get(stamp)})
    node.create_subscription(PoseWithCovarianceStamped,'/pose',pose,qos_profile_sensor_data)
    env = dict(os.environ)
    env['LD_LIBRARY_PATH'] = args.library_dir + ':' + env.get('LD_LIBRARY_PATH','')
    env['ROS_LOG_DIR'] = str(args.output/'ros_logs')
    cmd = ['/opt/ros/humble/lib/slam_toolbox/sync_slam_toolbox_node', '--ros-args',
       '--params-file','/home/ubuntu/ros2_ws/src/slam/config/slam.yaml',
       '-p','use_map_saver:=false','-p','base_frame:=base_footprint',
       '-p','scan_topic:=/scan','-p','minimum_time_interval:=0.05',
       '-p','map_update_interval:=0.5','-p','resolution:=0.02']
    child = None
    code = None
    start = time.monotonic()
    loaded = []
    sent = 0
    try:
        with (args.output/'slam.log').open('w') as log:
            child = subprocess.Popen(cmd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            next_scan = start
            while time.monotonic()-start < 15.:
                if child.poll() is not None:
                    raise RuntimeError('SLAM exited unexpectedly')
                elapsed = time.monotonic()-start
                if time.monotonic() >= next_scan:
                    y = min(.6, max(0., (elapsed-4.)*.1))
                    stamp = node.get_clock().now().to_msg()
                    stamp_ns=stamp.sec*10**9+stamp.nanosec
                    truth[stamp_ns]=y
                    transform=TransformStamped()
                    transform.header.stamp=stamp
                    transform.header.frame_id='odom'
                    transform.child_frame_id='base_footprint'
                    transform.transform.rotation.z=math.sin(y/2)
                    transform.transform.rotation.w=math.cos(y/2)
                    tf.sendTransform(transform)
                    scan=LaserScan()
                    scan.header.stamp=stamp
                    scan.header.frame_id='test_laser'
                    scan.angle_min=0.
                    scan.angle_increment=2*math.pi/500
                    scan.angle_max=scan.angle_increment*499
                    scan.range_min=.05
                    scan.range_max=2.
                    scan.scan_time=.1
                    ranges=[]
                    for i in range(500):
                        angle=y+i*scan.angle_increment
                        cx,sy=math.cos(angle),math.sin(angle)
                        rx=1.4/abs(cx) if abs(cx)>1e-12 else float('inf')
                        ry=1./abs(sy) if abs(sy)>1e-12 else float('inf')
                        r=min(rx,ry)
                        ranges.append(r if r<=2. else float('inf'))
                    scan.ranges=ranges
                    pub.publish(scan)
                    sent+=1
                    next_scan+=.1
                rclpy.spin_once(node,timeout_sec=.005)
                if elapsed>3 and not loaded:
                    loaded=[line for line in Path(f'/proc/{child.pid}/maps').read_text().splitlines()
                            if 'libtoolbox_common.so' in line]
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid,signal.SIGINT)
            try:
                code=child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM)
                try:
                    code=child.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL)
                    code=child.wait(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
    errors=[abs(math.atan2(math.sin(p['yaw']-p['truth_yaw']),math.cos(p['yaw']-p['truth_yaw'])))
            for p in received if p['truth_yaw'] is not None]
    passed=(len(received)>=15 and received[-1]['yaw']>.50 and max(errors,default=99)<.04
            if args.variant=='patched' else len(received)==1)
    result={'kind':'ISOLATED_SYNTHETIC_NOT_REAL_MOTION','variant':args.variant,
        'domain_id':87,'passed':passed,'scans_sent':sent,'poses_received':len(received),
        'final_pose':received[-1] if received else None,
        'maximum_yaw_error_rad':max(errors,default=None),'loaded_library_maps':loaded,
        'child_exit':code,'child_reaped':child is not None and child.poll() is not None,
        'poses':received}
    (args.output/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='poses'}),flush=True)
    if not passed:
        raise SystemExit(1)

if __name__=='__main__':
    main()
