"""Isolated guard integration test. Nonzero tests NEVER use production topics."""
import json
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from probe_continuous_fusion import stop_child

PREFIX = '/raicom/localization_guard_test'


def main():
    rclpy.init()
    node = rclpy.create_node('localization_guard_test_reader')
    speed_pub = node.create_publisher(Twist, PREFIX+'/input', 1)
    health_pub = node.create_publisher(String, PREFIX+'/health', 1)
    rows, production = [], []
    subscriptions = [
        node.create_subscription(Twist, PREFIX+'/output', lambda m: rows.append((time.monotonic(), m.linear.x)), 1),
        node.create_subscription(Twist, '/controller/cmd_vel', lambda m: production.append(
            [m.linear.x,m.linear.y,m.linear.z,m.angular.x,m.angular.y,m.angular.z]), qos_profile_sensor_data)]
    child = subprocess.Popen([sys.executable, '-B', '-m', 'raicom_control.cmd_vel_guard', '--ros-args',
        '-r', '__ns:='+PREFIX, '-p', 'input_topic:='+PREFIX+'/input',
        '-p', 'output_topic:='+PREFIX+'/output', '-p', 'localization_health_topic:='+PREFIX+'/health',
        '-p', 'localization_session:=isolated-test-session'])
    report = dict(application_output_topics=[PREFIX+'/input',PREFIX+'/health'], stages=[])
    sequence = 0

    def stage(name, seconds, mode, expected):
        nonlocal sequence
        begin = time.monotonic()
        while time.monotonic()-begin < seconds:
            msg = Twist(); msg.linear.x = .1
            speed_pub.publish(msg)
            if mode != 'absent':
                now = node.get_clock().now().nanoseconds*1e-9
                sequence += 1
                health = dict(session='isolated-test-session', sequence=sequence, state='ACTIVE', valid=True,
                              source_stamps=dict(laser=now, imu=now, ekf=now))
                if mode == 'stale': health['source_stamps']['laser'] -= 1
                if mode == 'foreign': health['session'] = 'foreign-session'
                health_pub.publish(String(data=json.dumps(health)))
            rclpy.spin_once(node, timeout_sec=.025)
            time.sleep(.01)
        observed = [v for t,v in rows if t > begin+.7]
        assert len(observed) >= 10, (name, len(observed))
        assert all(abs(v-expected) < 1e-8 for v in observed), (name, observed)
        report['stages'].append(dict(name=name, samples=len(observed), expected=expected, passed=True))

    try:
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        owners = [i.node_name for i in node.get_publishers_info_by_topic('/controller/cmd_vel')]
        assert owners == ['cmd_vel_guard'], owners
        out_subs = node.get_subscriptions_info_by_topic(PREFIX+'/output')
        assert len(out_subs) == 1 and out_subs[0].node_name == node.get_name()
        stage('no_health', 2, 'absent', 0)
        stage('fresh_sources_private_motion', 2, 'valid', .1)
        stage('stale_laser_fresh_heartbeat', 2, 'stale', 0)
        stage('foreign_session', 2, 'foreign', 0)
        stage('recover_requires_fresh_command', 2, 'valid', .1)
        stage('producer_disappears', 2, 'absent', 0)
        assert len(production) > 100 and not any(any(row) for row in production)
        report.update(passed=True, production_n=len(production), production_nonzero=0)
    finally:
        report['child'] = stop_child(child)
        print('GUARD_INTEGRATION '+json.dumps(report), flush=True)
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
