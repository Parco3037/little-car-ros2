"""Capture a latched AMCL prior, or explicitly re-seed it after odom reset.

No velocity/action publishers. The captured pose is NOT physical ground truth.
--apply is an explicit initial estimate, never a re-stamped sensor measurement.
Both modes require a fresh map, scan, one guard, and observed zero commands.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def route_point_prior(route, binding, station, *, yaml_path: Path, image_path: Path):
    """Build a recorded route-point prior only after proving the fixed map assets.

    The route stores an older, documented occupancy-cell byte encoding.  The
    live binding therefore uses the observed OccupancyGrid digest while the
    route is tied to the exact saved YAML/PGM hashes and map geometry.
    """
    if route.get('schema') != 'raicom_manual_route/v1' or route.get('frame') != 'map':
        raise ValueError('Route schema/frame mismatch')
    saved = route.get('map', {}).get('saved', {})
    loaded = route.get('map', {}).get('loaded', {})
    actual_yaml, actual_image = _sha256(yaml_path), _sha256(image_path)
    if actual_yaml != saved.get('yaml_sha256') or actual_image != saved.get('image_sha256'):
        raise ValueError('Saved map asset hash mismatch')
    if (int(loaded.get('width', -1)) != int(binding['width'])
            or int(loaded.get('height', -1)) != int(binding['height'])
            or not math.isclose(float(loaded.get('resolution', math.nan)),
                                float(binding['resolution']), rel_tol=0.0, abs_tol=1e-9)):
        raise ValueError('Loaded map geometry mismatch')
    point = route.get('points', {}).get(station)
    if not isinstance(point, dict):
        raise ValueError(f'Route point missing: {station}')
    values = [point.get('x'), point.get('y'), point.get('yaw_rad')]
    if any(isinstance(value, bool) or not isinstance(value, (int, float))
           or not math.isfinite(value) for value in values):
        raise ValueError(f'Route point pose is invalid: {station}')
    covariance = [0.0] * 36
    covariance[0] = covariance[7] = .03 ** 2
    covariance[35] = .05 ** 2
    yaw = float(point['yaw_rad'])
    return {
        'kind': 'ROUTE_POINT_INITIAL_ESTIMATE_NOT_GROUND_TRUTH',
        'route_point': station,
        'map': dict(binding),
        'original_stamp': 0.0,
        'position': [float(point['x']), float(point['y']), 0.0],
        'quaternion': [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)],
        'covariance': covariance,
        'asset_sha256': {'yaml': actual_yaml, 'image': actual_image},
    }


def route_start_prior(route, binding, *, yaml_path: Path, image_path: Path):
    """Compatibility wrapper for the existing startup-area recovery path."""
    return route_point_prior(
        route, binding, 'start', yaml_path=yaml_path, image_path=image_path)


def main():
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    parser = argparse.ArgumentParser()
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--prior', type=Path)
    source.add_argument('--route-start', type=Path)
    parser.add_argument('--route-point', default='start',
                        choices=('start', 'recognition', 'pickup', 'park_1', 'park_2'))
    parser.add_argument('--map-yaml', type=Path)
    parser.add_argument('--map-image', type=Path)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    if args.route_start is not None and (not args.apply or args.map_yaml is None or args.map_image is None):
        parser.error('--route-start requires --apply, --map-yaml and --map-image')
    if args.prior is not None and args.route_point != 'start':
        parser.error('--route-point is only valid with --route-start')
    rclpy.init(); node = rclpy.create_node('map_prior_recovery')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    data = dict(speed_n=0, nonzero=0, speed_at=None, pose=None, map=None, scan=None)
    def speed(msg):
        data['speed_at'] = time.monotonic(); data['speed_n'] += 1
        data['nonzero'] += int(any([msg.linear.x,msg.linear.y,msg.linear.z,
                                   msg.angular.x,msg.angular.y,msg.angular.z]))
    subs = [node.create_subscription(Twist, '/controller/cmd_vel', speed, qos_profile_sensor_data),
            node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m:data.update(pose=m), qos),
            node.create_subscription(OccupancyGrid, '/map', lambda m:data.update(map=m), qos),
            node.create_subscription(LaserScan, '/scan_stable', lambda m:data.update(scan=m), qos_profile_sensor_data)]
    start = time.monotonic()
    while time.monotonic()-start < 8:
        rclpy.spin_once(node, timeout_sec=.02)
    assert data['speed_n'] > 80 and data['nonzero'] == 0 and time.monotonic()-data['speed_at'] < .3
    assert [i.node_name for i in node.get_publishers_info_by_topic('/controller/cmd_vel')] == ['cmd_vel_guard']
    grid, scan = data['map'], data['scan']
    assert grid is not None and scan is not None
    assert grid.header.frame_id == 'map' and grid.info.resolution < .021 and len(scan.ranges) == 254
    now = node.get_clock().now().nanoseconds*1e-9
    assert 0 <= now-scan.header.stamp.sec-scan.header.stamp.nanosec*1e-9 < .3
    binding = dict(width=grid.info.width, height=grid.info.height, resolution=grid.info.resolution,
                   cells_sha256=hashlib.sha256(bytes(v & 255 for v in grid.data)).hexdigest())
    if not args.apply:
        msg = data['pose']; assert msg is not None and msg.header.frame_id == 'map'
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        result = dict(kind='LATched_AMCL_PRIOR_NOT_GROUND_TRUTH', map=binding,
                      captured_ros_time=now, original_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                      position=[p.x,p.y,p.z], quaternion=[q.x,q.y,q.z,q.w], covariance=list(msg.pose.covariance),
                      zero_samples=data['speed_n'])
        with args.prior.open('x') as stream: json.dump(result, stream, indent=2, allow_nan=False)
        print(json.dumps(result), flush=True)
    else:
        if args.route_start is not None:
            prior = route_point_prior(
                json.loads(args.route_start.read_text()), binding, args.route_point,
                yaml_path=args.map_yaml, image_path=args.map_image,
            )
        else:
            prior = json.loads(args.prior.read_text())
        prior_map = prior['map']
        map_matches = (
            set(prior_map) == set(binding)
            and int(prior_map['width']) == int(binding['width'])
            and int(prior_map['height']) == int(binding['height'])
            and math.isclose(
                float(prior_map['resolution']), float(binding['resolution']),
                rel_tol=0.0, abs_tol=1e-9,
            )
            and str(prior_map['cells_sha256']) == str(binding['cells_sha256'])
        )
        if not map_matches:
            raise RuntimeError(
                'Map binding mismatch: prior='
                + json.dumps(prior['map'], sort_keys=True)
                + ' observed=' + json.dumps(binding, sort_keys=True)
            )
        assert [i.node_name for i in node.get_publishers_info_by_topic('/odom')] == ['local_odometry_supervisor']
        publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        end = time.monotonic()+3
        while time.monotonic() < end: rclpy.spin_once(node, timeout_sec=.02)
        assert data['nonzero'] == 0 and time.monotonic()-data['speed_at'] < .3
        msg = PoseWithCovarianceStamped(); msg.header.frame_id = 'map'
        msg.header.stamp = node.get_clock().now().to_msg()
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        p.x,p.y,p.z = prior['position']; q.x,q.y,q.z,q.w = prior['quaternion']
        msg.pose.covariance = prior['covariance']
        # This is a reinitialization prior, not a claim of calibrated precision.
        for i, floor in ((0,.03**2),(7,.03**2),(35,.05**2)):
            msg.pose.covariance[i] = max(msg.pose.covariance[i], floor)
        publisher.publish(msg)
        end = time.monotonic()+2
        while time.monotonic() < end: rclpy.spin_once(node, timeout_sec=.02)
        print(json.dumps(dict(applied_initial_estimate=True, prior_kind=prior.get('kind'),
                              original_stamp=prior['original_stamp'],
                              position=prior['position'], quaternion=prior['quaternion'], map=binding,
                              route_point=prior.get('route_point'),
                              asset_sha256=prior.get('asset_sha256'),
                              zero_samples=data['speed_n'], nonzero=data['nonzero'])), flush=True)
    node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
