"""Subscription-only, bounded localization verification; never commands the robot."""
import argparse
import json
import math
import time
from datetime import datetime, timezone

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
import tf2_ros


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=90)
    parser.add_argument('--full-samples', action='store_true',
                        help='Include every sampled TF for exact offline hold-window analysis')
    args = parser.parse_args()
    if not 10 <= args.seconds <= 300:
        parser.error('seconds must be 10..300')
    rclpy.init()
    node = rclpy.create_node('stationary_localization_verifier')
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, node)
    started = time.monotonic()
    report = {'utc_start': datetime.now(timezone.utc).isoformat(),
              'velocity_n': 0, 'velocity_nonzero': 0}
    series, amcl, base, subs = {}, [], [], []

    def velocity(msg):
        report['velocity_n'] += 1
        report['velocity_nonzero'] += int(any(v != 0 for v in (
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z)))

    def orientation(msg, key):
        q = msg.orientation if isinstance(msg, Imu) else msg.pose.pose.orientation
        series.setdefault(key, []).append([time.monotonic()-started, yaw(q)])

    def pose(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        now = node.get_clock().now().nanoseconds*1e-9
        amcl.append({'t': time.monotonic()-started, 'stamp': stamp, 'age': now-stamp,
                     'x': msg.pose.pose.position.x, 'y': msg.pose.pose.position.y,
                     'yaw_deg': math.degrees(yaw(msg.pose.pose.orientation)),
                     'covariance_diagonal': [msg.pose.covariance[i] for i in (0, 7, 35)]})

    subs.append(node.create_subscription(Twist, '/controller/cmd_vel', velocity, qos_profile_sensor_data))
    for topic, kind in [('/imu', Imu), ('/odom', Odometry), ('/odom_raw', Odometry)]:
        subs.append(node.create_subscription(kind, topic,
            lambda msg, key=topic: orientation(msg, key), qos_profile_sensor_data))
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    subs.append(node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', pose, qos))
    next_sample = 0.0
    while time.monotonic()-started < args.seconds:
        rclpy.spin_once(node, timeout_sec=.02)
        elapsed = time.monotonic()-started
        if elapsed >= next_sample:
            next_sample = elapsed + .1
            try:
                tf = buffer.lookup_transform('map', 'base_footprint', Time())
                age = node.get_clock().now().nanoseconds*1e-9 - (
                    tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9)
                base.append([elapsed, tf.transform.translation.x,
                             tf.transform.translation.y, yaw(tf.transform.rotation), age])
            except Exception:
                pass
    report['angles'] = {}
    for topic, values in series.items():
        m = np.asarray(values)
        angles = np.unwrap(m[:, 1])
        report['angles'][topic] = {'n': len(m),
            'delta_deg': float(np.degrees(angles[-1]-angles[0]))}
    if base:
        m = np.asarray(base)
        angles = np.unwrap(m[:, 3])
        report['map_base'] = {'n': len(m), 'duration_sec': float(m[-1, 0]-m[0, 0]),
            'first_xy_yaw_deg': [float(m[0, 1]), float(m[0, 2]), float(np.degrees(angles[0]))],
            'last_xy_yaw_deg': [float(m[-1, 1]), float(m[-1, 2]), float(np.degrees(angles[-1]))],
            'yaw_delta_deg': float(np.degrees(angles[-1]-angles[0])),
            'yaw_span_deg': float(np.degrees(np.ptp(angles))),
            'max_xy_from_first_m': float(np.linalg.norm(m[:, 1:3]-m[0, 1:3], axis=1).max()),
            'max_xy_step_m': float(np.linalg.norm(np.diff(m[:, 1:3], axis=0), axis=1).max()) if len(m)>1 else 0.,
            'max_tf_age_sec': float(m[:, 4].max()),
            'samples_every_5s': [r.tolist() for r in m[::50]]}
        if args.full_samples:
            report['map_base']['all_samples'] = [r.tolist() for r in m]
    report['amcl_messages'] = amcl
    fresh = [p for p in amcl if 0 <= p['age'] <= 1]
    report['fresh_amcl_n'] = len(fresh)
    report['publishers'] = {topic: [i.node_namespace.rstrip('/')+'/'+i.node_name
        for i in node.get_publishers_info_by_topic(topic)]
        for topic in ['/controller/cmd_vel', '/imu', '/odom', '/odom_raw', '/amcl_pose']}
    report['utc_end'] = datetime.now(timezone.utc).isoformat()
    print(json.dumps(report, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
