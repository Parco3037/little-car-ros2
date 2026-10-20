"""Teammate example. Publishes A/B only on explicit terminal input after READY.

Run inside ROS2 Humble: python3 -m raicom_control.pick_signal_example
Replace the terminal input with the finished grasp/classification callback.
This node does not move the robot, arm or gripper.
"""
import argparse
import queue
import threading


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request-topic', default='/raicom/pick/request')
    parser.add_argument('--result-topic', default='/raicom/pick/result')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node('raicom_pick_signal_example')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    result = node.create_publisher(String, args.result_topic, qos)
    inbox = queue.Queue()
    # Epoch here is internal only. Plain A/B cannot identify stale results that
    # were delayed on the network into a later PICK_READY window.
    state = {'waiting': False, 'epoch': 0}
    def received(msg):
        if msg.data == 'CANCEL':
            state['waiting'] = False; state['epoch'] += 1
            node.get_logger().warning('CANCEL: stop grasp work, discard pending classification, return arm control')
        elif msg.data == 'PICK_READY' and not state['waiting']:
            state['waiting'] = True; state['epoch'] += 1
            node.get_logger().info('PICK_READY: complete grasp and 1/2 classification; hold grip, return arm control, then type A or B')
    node.create_subscription(String, args.request_topic, received, qos)
    def terminal():
        while True:
            try: value = input().strip()
            except EOFError: return
            inbox.put((value, state['epoch'], state['waiting']))
    threading.Thread(target=terminal, daemon=True).start()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
            try: value, epoch, was_waiting = inbox.get_nowait()
            except queue.Empty: continue
            if value not in ('A', 'B', 'PICK_FAILED') or not was_waiting or not state['waiting'] or epoch != state['epoch']:
                node.get_logger().warning('Ignored: no current request, cancelled, duplicate, or unknown result')
                continue
            # Actual teammate node must release its /servo_controller publisher
            # BEFORE publishing the result. The field tool checks exclusive I/O.
            result.publish(String(data=value)); state['waiting'] = False
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
