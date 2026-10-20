"""ROS2 String adapter. Only records callbacks; UI owns state transitions."""
import json
import queue
import time


class PickSignalIO:
    def __init__(self, node, request_topic, result_topic):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from std_msgs.msg import String
        self.node, self.message_type = node, String
        self.inbox = queue.Queue(maxsize=16)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.request = node.create_publisher(String, request_topic, qos)
        self.status = node.create_publisher(String, '/raicom/task/status', qos)
        self.subscription = node.create_subscription(String, result_topic, self._receive, qos)

    def _receive(self, message):
        try: self.inbox.put_nowait((message.data, time.monotonic()))
        except queue.Full: pass  # Never block the ROS executor or resurrect old results.

    def drain(self):
        result = []
        while True:
            try: result.append(self.inbox.get_nowait())
            except queue.Empty: return result

    def send(self, signal):
        self.request.publish(self.message_type(data=signal))

    def publish_status(self, status):
        self.status.publish(self.message_type(data=json.dumps(status, ensure_ascii=False)))

    def close(self):
        self.node.destroy_subscription(self.subscription)
        self.node.destroy_publisher(self.request)
        self.node.destroy_publisher(self.status)
