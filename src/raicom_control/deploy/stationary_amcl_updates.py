"""Request AMCL laser updates while stopped; NEVER set a pose or move the robot.

Humble request_nomotion_update only sets AMCL's force_update flag. Limit requests
to one per two seconds, only with initialized AMCL, matching fresh local-odometry
lease, fresh formal odometry, and continuously zero observed commands.
"""
import argparse
import json
import signal
import time

from continuous_fusion_runtime import identity
from raicom_control.guard_logic import LocalizationLease


def main():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import Empty
    parser = argparse.ArgumentParser()
    parser.add_argument('--session', required=True)
    parser.add_argument('--parent-pid', type=int, required=True)
    parser.add_argument('--parent-start', required=True)
    args = parser.parse_args()
    assert identity(args.parent_pid) == args.parent_start
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('stationary_amcl_updates', enable_rosout=False, start_parameter_services=False)
    lease = LocalizationLease(args.session)
    state = dict(stop=False, speed_at=None, zero_since=None, odom_stamp=None, pose_stamp=None,
                 calls=0, completed=0, nonzero=0, future=None, requested_at=None)
    def now(): return node.get_clock().now().nanoseconds*1e-9
    def stamp(m): return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    def speed(m):
        mono = time.monotonic()
        if state['speed_at'] is None or mono-state['speed_at'] > .3:
            state['zero_since'] = None
        state['speed_at'] = mono
        if any([m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z]):
            state['nonzero'] += 1; state['zero_since'] = None
        elif state['zero_since'] is None:
            state['zero_since'] = mono
    def health(m):
        try: message = json.loads(m.data)
        except ValueError: message = None
        lease.receive(message, time.monotonic(), now())
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subs = [node.create_subscription(Twist, '/controller/cmd_vel', speed, qos_profile_sensor_data),
            node.create_subscription(String, '/raicom/local_odometry/health', health, 1),
            node.create_subscription(Odometry, '/odom', lambda m:state.update(odom_stamp=stamp(m)), qos_profile_sensor_data),
            node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m:state.update(pose_stamp=stamp(m)), qos)]
    client = node.create_client(Empty, '/request_nomotion_update')
    signal.signal(signal.SIGINT, lambda s,f:state.update(stop=True))
    signal.signal(signal.SIGTERM, lambda s,f:state.update(stop=True))
    next_check = time.monotonic()+5
    try:
        while not state['stop'] and identity(args.parent_pid) == args.parent_start:
            rclpy.spin_once(node, timeout_sec=.02)
            mono = time.monotonic()
            if state['future'] is not None:
                if state['future'].done():
                    state['future'].result(); state['completed'] += 1
                    print(json.dumps(dict(event='nomotion_update_completed', calls=state['calls'],
                                          ros_time=now(), amcl_stamp=state['pose_stamp'])), flush=True)
                    state['future'] = None
                elif mono-state['requested_at'] > 3:
                    raise RuntimeError('AMCL update service timeout; stop requesting')
            if mono < next_check or state['future'] is not None: continue
            next_check = mono+2
            if (state['zero_since'] is None or mono-state['zero_since'] < 2
                    or mono-state['speed_at'] > .3 or not lease.valid_at(mono, now())
                    or state['odom_stamp'] is None or not 0 <= now()-state['odom_stamp'] <= .3
                    or state['pose_stamp'] is None or now()-state['pose_stamp'] < 1):
                continue
            if [i.node_name for i in node.get_publishers_info_by_topic('/odom')] != ['local_odometry_supervisor']:
                raise RuntimeError('Odometry publisher changed')
            if [i.node_name for i in node.get_publishers_info_by_topic('/amcl_pose')] != ['amcl']:
                raise RuntimeError('AMCL publisher changed')
            if not client.service_is_ready(): continue
            state['future'] = client.call_async(Empty.Request())
            state['requested_at'] = mono; state['calls'] += 1
    finally:
        print(json.dumps(dict(event='end', calls=state['calls'], completed=state['completed'],
                              observed_nonzero=state['nonzero'])), flush=True)
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
