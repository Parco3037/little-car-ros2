#!/usr/bin/env python3
"""Bounded real-sensor observation. No velocity, goal, servo, or TF publishers."""
import argparse
from collections import defaultdict
import gzip
import json
import math
import os
from pathlib import Path
import time

def process_snapshot():
    result = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            cmd = (entry / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            if not any(s in cmd for s in ('sync_slam_toolbox_node', 'ros_robot_controller --',
                    'ekf_node', 'oradar_scan --', 'lib/raicom_control/cmd_vel_guard')):
                continue
            parts = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            result[entry.name] = {'cmd': cmd, 'start_ticks': parts[19],
                'cpu_ticks': int(parts[11]) + int(parts[12]), 'rss_pages': int(parts[21])}
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            pass
    return result

def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import LaserScan, Imu
    from tf2_msgs.msg import TFMessage
    from tf2_ros import Buffer, TransformListener
    from ros_robot_controller_msgs.msg import MotorsState
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=120.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 10 <= args.seconds <= 180:
        raise ValueError('10..180 seconds only')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('raicom_slam_static_observer', enable_rosout=False,
                            start_parameter_services=False)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    streams, poses, geometry, authority = (defaultdict(list) for _ in range(4))
    counts = defaultdict(int)
    raw_scans = []
    tf_failures = []
    begin = time.monotonic()
    before = process_snapshot()
    def yaw(q):
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
    def record(key, header):
        stamp = header.stamp.sec + header.stamp.nanosec*1e-9
        now = node.get_clock().now().nanoseconds*1e-9
        streams[key].append([time.monotonic()-begin, stamp, now-stamp])
    def scan(msg):
        record('/scan', msg.header)
        geometry['/scan'].append([len(msg.ranges), msg.angle_min, msg.angle_max,
                                 msg.angle_increment, msg.range_min, msg.range_max])
        # Allow discovery to fill the TF buffer; never retimestamp measurements.
        if time.monotonic()-begin > 3:
            try:
                buffer.lookup_transform('odom', msg.header.frame_id,
                    rclpy.time.Time.from_msg(msg.header.stamp))
                counts['scan_stamp_tf_ok'] += 1
            except Exception as exc:
                counts['scan_stamp_tf_fail'] += 1
                if len(tf_failures) < 5:
                    tf_failures.append(str(exc))
        raw_scans.append([time.monotonic()-begin, list(msg.ranges)])
    def odom(topic, msg):
        record(topic, msg.header)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        poses[topic].append([time.monotonic()-begin, p.x, p.y, yaw(q)])
    def imu(msg):
        record('/imu', msg.header)
        poses['/imu'].append([time.monotonic()-begin, 0., 0., yaw(msg.orientation)])
    def tf(msg):
        for t in msg.transforms:
            edge = t.header.frame_id + '->' + t.child_frame_id
            authority[edge].append(1)
            if edge in ('map->odom', 'odom->base_footprint'):
                record(edge, t.header)
                p, q = t.transform.translation, t.transform.rotation
                poses[edge].append([time.monotonic()-begin, p.x, p.y, yaw(q)])
    def velocity(msg):
        values = [msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z]
        counts['velocity'] += 1
        counts['nonzero_velocity'] += int(any(not math.isfinite(x) or x != 0 for x in values))
    def motors(msg):
        counts['motors'] += 1
        counts['nonzero_motors'] += int(any(not math.isfinite(x.rps) or x.rps != 0 for x in msg.data))
    node.create_subscription(LaserScan, '/scan', scan, qos)
    for topic in ('/odom', '/odom_raw'):
        node.create_subscription(Odometry, topic, lambda m,t=topic: odom(t,m), qos)
    node.create_subscription(Imu, '/imu', imu, qos)
    node.create_subscription(Twist, '/controller/cmd_vel', velocity, qos)
    node.create_subscription(MotorsState, '/ros_robot_controller/set_motor', motors, qos)
    node.create_subscription(TFMessage, '/tf', tf, qos)
    node.create_subscription(TFMessage, '/tf_static', tf, map_qos)
    node.create_subscription(PoseWithCovarianceStamped, '/pose', lambda m: odom('/pose',m), qos)
    node.create_subscription(OccupancyGrid, '/map', lambda m: record('/map',m.header), map_qos)
    while time.monotonic()-begin < args.seconds:
        rclpy.spin_once(node, timeout_sec=.01)
    elapsed = time.monotonic()-begin
    after = process_snapshot()
    def percentile(values, fraction):
        return sorted(values)[int((len(values)-1)*fraction)] if values else None
    result = {'seconds': elapsed, 'counts': dict(counts), 'topics': {},
              'tf_edges': {e: len(v) for e,v in authority.items()},
              'tf_writer_attribution': 'Not measured: installed rclpy executor drops MessageInfo',
              'tf_failures': tf_failures, 'process_before': before, 'process_after': after}
    for topic, values in streams.items():
        gaps = [b[0]-a[0] for a,b in zip(values,values[1:])]
        ages = [v[2] for v in values]
        result['topics'][topic] = {'n': len(values),
          'hz': (len(values)-1)/(values[-1][0]-values[0][0]) if len(values)>1 else None,
          'gap_max_s': max(gaps) if gaps else None, 'age_p95_s': percentile(ages,.95),
          'age_max_s': max(ages), 'stamp_backwards': sum(b[1]<a[1] for a,b in zip(values,values[1:]))}
    for topic, values in poses.items():
        import numpy as np
        a = np.asarray(values)
        y = np.unwrap(a[:,3])
        result['topics'][topic].update(xy_span_m=np.ptp(a[:,1:3],axis=0).tolist(),
          yaw_net_deg=float(np.degrees(y[-1]-y[0])), yaw_span_deg=float(np.degrees(np.ptp(y))),
          yaw_trend_deg_per_min=float(np.degrees(np.polyfit(a[:,0],y,1)[0])*60) if len(y)>2 else None)
    result['scan_geometry'] = sorted(set(tuple(g) for g in geometry['/scan']))
    result['cpu_percent_one_core'] = {pid:100*(v['cpu_ticks']-before[pid]['cpu_ticks']) /
        (os.sysconf('SC_CLK_TCK')*elapsed) for pid,v in after.items()
        if pid in before and v['start_ticks']==before[pid]['start_ticks']}
    endpoints = {}
    for topic in ('/scan','/odom','/odom_raw','/map','/tf','/controller/cmd_vel',
                  '/ros_robot_controller/set_motor','/raicom/motion_enable','/raicom/teleop_cmd'):
        endpoints[topic] = {'publishers': [{'node': e.node_namespace+'/'+e.node_name,
            'gid': bytes(e.endpoint_gid).hex()} for e in node.get_publishers_info_by_topic(topic)],
            'subscriptions': [e.node_namespace+'/'+e.node_name for e in node.get_subscriptions_info_by_topic(topic)]}
    result['endpoints'] = endpoints
    result['caveat'] = 'Zero commands are not independent physical-stillness or navigation acceptance.'
    (args.output/'summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    with gzip.open(args.output/'numeric.json.gz','wt',encoding='utf-8') as stream:
        json.dump({'streams':streams,'poses':poses,'scans':raw_scans,'geometry':geometry},stream)
    print(json.dumps(result),flush=True)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
