"""Read-only sensor executor, independent of preview and task callbacks."""
import threading

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
import tf2_ros


class FieldToolSensors:
    def __init__(self, owner):
        self.node = Node(owner.get_name() + '_sensors', context=owner.context,
                         namespace=owner.get_namespace(), parameter_overrides=[
                             Parameter('use_sim_time', value=owner.get_parameter('use_sim_time').value)])
        self.buffer = tf2_ros.Buffer()
        # Latest dynamic transforms matter more than replaying a long DDS queue
        # after a scheduling delay. Static transforms keep their latched QoS.
        dynamic_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.listener = tf2_ros.TransformListener(self.buffer, self.node, qos=dynamic_qos)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                            reliability=ReliabilityPolicy.RELIABLE)
        self.subscriptions = [
            self.node.create_subscription(OccupancyGrid, '/map', owner._on_map, latched),
            self.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', owner._on_amcl, latched),
            self.node.create_subscription(LaserScan, '/scan', owner._on_scan, qos_profile_sensor_data),
            self.node.create_subscription(Twist, '/controller/cmd_vel', owner._on_velocity, qos_profile_sensor_data),
        ]
        self.executor = SingleThreadedExecutor(context=owner.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True, name='field-sensors-ros')

    def start(self):
        self.thread.start()

    def close(self):
        self.executor.shutdown(timeout_sec=2.)
        self.thread.join(timeout=2.)
        self.node.destroy_node()
