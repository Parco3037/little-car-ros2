"""Read-only command-chain observation; motor commands are not encoder feedback."""
import math
import threading
import time


class CommandChain:
    def __init__(self):
        self.samples = {}
        self.lock = threading.Lock()

    def observe(self, kind, values, now):
        if not all(math.isfinite(v) for v in values): return
        with self.lock: self.samples[kind] = {'at': now, 'values': list(values)}

    def snapshot(self, now):
        with self.lock: result = dict(self.samples)
        return {name: dict(value, fresh=0 <= now-value['at'] <= .6) for name,value in result.items()}

    def describe(self, now):
        values = self.snapshot(now)
        def fresh(k): return k in values and values[k]['fresh']
        def moving(k): return any(abs(v)>.005 for v in values[k]['values'])
        chain = [('nav2', 'Nav2'), ('smooth', '平滑器'), ('bridge', 'bridge'), ('guard', 'guard'), ('motors', 'STM32 电机命令')]
        for key, title in chain:
            if not fresh(key): return title + ' · 暂无新鲜输出'
        for (before, name1), (after, name2) in zip(chain, chain[1:]):
            if moving(before) and not moving(after): return name1+' 有命令，'+name2+' 当前为零'
        if not moving('nav2'): return 'Nav2 当前输出零；核对目标、规划及到点状态'
        if not fresh('odom'): return '动力链有命令；里程计回报过期，执行情况未知'
        if not moving('odom'): return '电机命令非零，里程计未见运动；检查 STM32 / 四轮'
        return '动力链有命令，里程计有运动回报'


class CommandChainObserver(CommandChain):
    def __init__(self, node):
        super().__init__()
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from ros_robot_controller_msgs.msg import MotorsState
        from rclpy.qos import qos_profile_sensor_data
        self.subscriptions = []
        for topic, key in [('/raicom/nav2_staging/raw_cmd_vel', 'nav2'),
                           ('/raicom/nav2_staging/cmd_vel', 'smooth'),
                           ('/raicom/teleop_cmd', 'bridge'), ('/controller/cmd_vel', 'guard')]:
            self.subscriptions.append(node.create_subscription(Twist, topic,
                lambda m,k=key: self.observe(k, [m.linear.x,m.linear.y,m.angular.z], time.monotonic()), qos_profile_sensor_data))
        self.subscriptions.append(node.create_subscription(MotorsState, '/ros_robot_controller/set_motor',
            lambda m: self.observe('motors', [s.rps for s in m.data], time.monotonic()), qos_profile_sensor_data))
        self.subscriptions.append(node.create_subscription(Odometry, '/odom',
            lambda m: self.observe('odom', [m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.angular.z],time.monotonic()), qos_profile_sensor_data))
