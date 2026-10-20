"""Read-only lifecycle, TF, command isolation and costmap audit. No actions."""
import collections
import json
import math
import time
from datetime import datetime, timezone

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data


def main():
    rclpy.init()
    node = rclpy.create_node('navigation_staging_readonly_audit')
    report = dict(utc_start=datetime.now(timezone.utc).isoformat(), velocity_n=0, velocity_nonzero=0)
    counts, grids, lengths, edges = collections.Counter(), {}, set(), collections.Counter()
    subs, clients = [], {}

    def speed(msg):
        report['velocity_n'] += 1
        report['velocity_nonzero'] += int(any(v != 0 for v in (
            msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)))

    def scan(msg, topic):
        counts[topic] += 1
        if topic == '/scan_stable':
            lengths.add(len(msg.ranges))

    def grid(msg, topic):
        counts[topic] += 1
        grids[topic] = dict(width=msg.info.width, height=msg.info.height,
                           resolution=msg.info.resolution, frame=msg.header.frame_id)

    def tf(msg):
        for transform in msg.transforms:
            edges[transform.header.frame_id+'->'+transform.child_frame_id] += 1

    subs.append(node.create_subscription(Twist, '/controller/cmd_vel', speed, qos_profile_sensor_data))
    for topic in ('/scan_stable', '/scan_raw'):
        subs.append(node.create_subscription(LaserScan, topic, lambda m,t=topic: scan(m,t), qos_profile_sensor_data))
    for topic in ('/odom', '/odom_raw'):
        subs.append(node.create_subscription(Odometry, topic, lambda m,t=topic: counts.update([t]), qos_profile_sensor_data))
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    for topic in ('/map', '/local_costmap/costmap', '/global_costmap/costmap'):
        subs.append(node.create_subscription(OccupancyGrid, topic, lambda m,t=topic: grid(m,t), qos))
    subs.append(node.create_subscription(TFMessage, '/tf', tf, qos_profile_sensor_data))
    for name in ('amcl', 'map_server', 'controller_server', 'planner_server', 'smoother_server',
                 'behavior_server', 'bt_navigator', 'waypoint_follower', 'velocity_smoother'):
        clients[name] = node.create_client(GetState, '/'+name+'/get_state')
    parameter_names = {
        'controller_server': [
            'use_sim_time', 'controller_frequency', 'FollowPath.plugin',
            'FollowPath.max_vel_y', 'FollowPath.max_vel_theta',
            'FollowPath.max_speed_xy', 'FollowPath.vx_samples',
            'FollowPath.vy_samples', 'FollowPath.vtheta_samples',
            'FollowPath.sim_time'],
        'local_costmap/local_costmap': ['resolution', 'footprint_padding', 'footprint'],
        'global_costmap/global_costmap': ['resolution', 'footprint_padding', 'footprint'],
        'velocity_smoother': ['max_velocity', 'min_velocity', 'velocity_timeout'],
    }
    param_clients = {n: node.create_client(GetParameters, '/'+n+'/get_parameters') for n in parameter_names}
    service_attempts = {}

    def call_with_retry(client, request, label, *, attempts=3, timeout_sec=4.0):
        """Issue one DDS service request at a time and retry transport loss.

        Sending every lifecycle and parameter request concurrently produced
        real Fast DDS response-delivery timeouts on the Pi while every Nav2
        node remained active.  Sequential requests keep the audit strict
        without turning a middleware burst into a false lifecycle failure.
        """
        rows = []
        for attempt in range(1, attempts + 1):
            if not client.wait_for_service(timeout_sec=2.0):
                rows.append({'attempt': attempt, 'result': 'service_unavailable'})
                continue
            future = client.call_async(request)
            before = time.monotonic()
            while time.monotonic() - before < timeout_sec:
                rclpy.spin_once(node, timeout_sec=.02)
                if future.done():
                    try:
                        result = future.result()
                    except Exception as error:  # ROS transport failure path.
                        rows.append({
                            'attempt': attempt, 'result': 'exception',
                            'detail': str(error),
                        })
                        break
                    rows.append({
                        'attempt': attempt, 'result': 'response',
                        'latency_sec': time.monotonic() - before,
                    })
                    service_attempts[label] = rows
                    return result
            else:
                future.cancel()
                rows.append({
                    'attempt': attempt, 'result': 'timeout',
                    'latency_sec': time.monotonic() - before,
                })
        service_attempts[label] = rows
        return None

    try:
        start = time.monotonic()
        # First collect graph and sensor traffic without issuing a burst of 13
        # concurrent DDS service calls from one Python executor.
        while time.monotonic()-start < 10:
            rclpy.spin_once(node, timeout_sec=.02)
        lifecycle_results = {
            name: call_with_retry(client, GetState.Request(), 'lifecycle:'+name)
            for name, client in clients.items()
        }
        report['lifecycles'] = {
            name: result.current_state.label if result is not None else 'timeout'
            for name, result in lifecycle_results.items()
        }
        report['missing_lifecycles'] = sorted(
            name for name, result in lifecycle_results.items() if result is None)
        report['parameters'] = {}
        for n, client in param_clients.items():
            request = GetParameters.Request()
            request.names = parameter_names[n]
            result = call_with_retry(client, request, 'parameters:'+n)
            if result is not None:
                fields = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value', 8: 'double_array_value'}
                report['parameters'][n] = {name: getattr(v,fields.get(v.type,'string_value'))
                    if v.type != 8 else list(v.double_array_value)
                    for name,v in zip(parameter_names[n],result.values)}
        controller_parameters = report['parameters'].get('controller_server', {})
        sample_names = (
            'FollowPath.vx_samples', 'FollowPath.vy_samples',
            'FollowPath.vtheta_samples')
        try:
            samples = [int(controller_parameters[name]) for name in sample_names]
            candidate_count = math.prod(samples)
        except (KeyError, TypeError, ValueError):
            samples = None
            candidate_count = None
        report['dwb_candidate_budget'] = {
            'readback_samples': samples,
            'expected_samples': [5, 5, 7],
            'candidate_count': candidate_count,
            'maximum_allowed': 200,
            'passed': samples == [5, 5, 7] and candidate_count <= 200,
        }
        while time.monotonic()-start < 30:
            rclpy.spin_once(node, timeout_sec=.02)
        report['service_attempts'] = service_attempts
        topics = ('/controller/cmd_vel', '/raicom/teleop_cmd', '/map', '/scan_raw', '/scan_stable',
                  '/odom', '/odom_raw', '/raicom/nav2_staging/raw_cmd_vel', '/raicom/nav2_staging/cmd_vel')
        report['publishers'] = {t: [p.node_namespace.rstrip('/')+'/'+p.node_name for p in node.get_publishers_info_by_topic(t)] for t in topics}
        report['staging_output_subscribers'] = [p.node_namespace.rstrip('/')+'/'+p.node_name for p in
            node.get_subscriptions_info_by_topic('/raicom/nav2_staging/cmd_vel')]
        report['tf_edge_message_counts'] = dict(edges)
        report['tf_per_edge_authority_verified'] = False  # Installed rclpy lacks MessageInfo callbacks.
        report['receive_counts'] = dict(counts)
        report['scan_lengths'] = sorted(lengths)
        report['grids'] = grids
        report['utc_end'] = datetime.now(timezone.utc).isoformat()
        print(json.dumps(report, indent=2), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
