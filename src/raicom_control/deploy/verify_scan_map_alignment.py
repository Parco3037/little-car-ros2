"""Read-only, timestamp-matched scan/map overlap verification. No publishers.

Reports ALL finite in-range visible points and, separately, the points usable
by the current AMCL laser model. Never hides rejected points from the report.
"""
import argparse
import hashlib
import json
import math
import time
from datetime import datetime, timezone

import numpy as np


def overlap(points, occupied):
    if len(points) == 0 or len(occupied) == 0:
        return {'n': int(len(points)), 'valid': False}
    distance = np.sqrt(((points[:, None, :]-occupied[None, :, :])**2).sum(axis=2)).min(axis=1)
    return {'n': len(points), 'valid': True, 'median_m': float(np.median(distance)),
            'p90_m': float(np.percentile(distance, 90)),
            'within_5cm': float(np.mean(distance <= .05)),
            'within_10cm': float(np.mean(distance <= .10)),
            'max_m': float(distance.max())}


def transform_xy(xy, translation, quaternion):
    x, y, z, w = quaternion
    rotation = np.asarray([[1-2*(y*y+z*z), 2*(x*y-z*w)],
                           [2*(x*y+z*w), 1-2*(x*x+z*z)]])
    return xy@rotation.T + np.asarray(translation[:2])


def main():
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    import tf2_ros

    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=120.)
    args = parser.parse_args()
    if not 10 <= args.seconds <= 300:
        parser.error('seconds must be 10..300')
    rclpy.init()
    node = rclpy.create_node('scan_map_overlap_verifier')
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, node)
    data = {'utc_start': datetime.now(timezone.utc).isoformat(), 'velocity_n': 0,
            'velocity_nonzero': 0, 'frames': [], 'errors': [], 'map_changed': False,
            'metric': 'Distance from each measured laser endpoint to nearest occupied cell centre; not full raycast or physical ground truth.',
            'stationary_overlap_trial_criteria': {'median_m_max': .05, 'p90_m_max': .10,
                'within_10cm_min': .90, 'scope': 'All visible valid returns, per sampled frame; no automatic acceptance of navigation.'}}
    state = {'scan': None, 'occupied': None, 'hash': None}

    def velocity(msg):
        data['velocity_n'] += 1
        data['velocity_nonzero'] += int(any(v != 0 for v in (msg.linear.x, msg.linear.y,
            msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)))

    def grid(msg):
        digest = hashlib.sha256(np.asarray(msg.data, dtype=np.int8).tobytes()).hexdigest()
        if state['hash'] is not None and digest != state['hash']:
            data['map_changed'] = True
        state['hash'] = digest
        cells = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
        gy, gx = np.nonzero(cells >= 65)
        local = np.column_stack(((gx+.5)*msg.info.resolution, (gy+.5)*msg.info.resolution))
        p, q = msg.info.origin.position, msg.info.origin.orientation
        state['occupied'] = transform_xy(local, [p.x, p.y], [q.x, q.y, q.z, q.w])
        data['map'] = {'width': msg.info.width, 'height': msg.info.height,
                       'resolution': msg.info.resolution, 'cells_sha256': digest}

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    subs = [node.create_subscription(Twist, '/controller/cmd_vel', velocity, qos_profile_sensor_data),
            node.create_subscription(OccupancyGrid, '/map', grid, qos),
            node.create_subscription(LaserScan, '/scan_stable', lambda m: state.update(scan=m), qos_profile_sensor_data)]
    start, next_at, last_stamp = time.monotonic(), 0., None
    while time.monotonic()-start < args.seconds:
        rclpy.spin_once(node, timeout_sec=.02)
        elapsed = time.monotonic()-start
        msg = state['scan']
        if elapsed < next_at or msg is None or state['occupied'] is None:
            continue
        next_at = elapsed+1
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        try:
            age = node.get_clock().now().nanoseconds*1e-9-stamp
            if not 0 <= age <= .5 or stamp == last_stamp:
                raise ValueError('Stale or repeated scan')
            tf = buffer.lookup_transform('map', msg.header.frame_id, Time.from_msg(msg.header.stamp))
            r = np.asarray(msg.ranges)
            angle = msg.angle_min+np.arange(len(r))*msg.angle_increment
            valid = np.isfinite(r)&(r >= msg.range_min)&(r <= msg.range_max)
            amcl = valid&(r >= max(.2, msg.range_min))&(r < msg.range_max)
            xy = np.column_stack((r[valid]*np.cos(angle[valid]), r[valid]*np.sin(angle[valid])))
            p, q = tf.transform.translation, tf.transform.rotation
            points = transform_xy(xy, [p.x, p.y], [q.x, q.y, q.z, q.w])
            measured = overlap(points, state['occupied'])
            measured['trial_criteria_met'] = (measured.get('valid', False) and measured['median_m'] <= .05
                and measured['p90_m'] <= .10 and measured['within_10cm'] >= .90)
            data['frames'].append({'t': elapsed, 'scan_stamp': stamp, 'age_sec': age,
                'all_visible': measured, 'amcl_usable': overlap(points[amcl[valid]], state['occupied']),
                'at_max_range_n': int(np.sum(valid&(r >= msg.range_max))),
                'below_amcl_min_n': int(np.sum(valid&(r < .2)))})
            last_stamp = stamp
        except Exception as error:
            data['errors'].append({'t': elapsed, 'error': str(error)})
    data['publishers'] = {topic: [i.node_namespace.rstrip('/')+'/'+i.node_name for i in
        node.get_publishers_info_by_topic(topic)] for topic in ['/map', '/scan_stable', '/controller/cmd_vel']}
    data['utc_end'] = datetime.now(timezone.utc).isoformat()
    print(json.dumps(data, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
