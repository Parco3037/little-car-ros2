#!/usr/bin/env python3
"""One bounded arm pose over existing ROS, before mapping gets a motion lease.

Default is read-only. --apply requires onsite clearance by the caller. No direct
serial access, no torque/PID edits, no wheel commands, no silent physical PASS.
"""
import argparse
import inspect
import json
import math
import signal
import time
from pathlib import Path

IDS = (1, 2, 3, 4, 5, 10)


def validate_profile(profile):
    positions = {int(k): v for k, v in profile['positions'].items()}
    if set(positions) != set(IDS) or len(profile['positions']) != len(IDS):
        raise ValueError('Pose must contain exactly IDs 1,2,3,4,5,10')
    if any(type(v) is not int or not 0 <= v <= 1000 for v in positions.values()):
        raise ValueError('Servo pulses must be integers in 0..1000')
    duration = float(profile['duration_s'])
    tolerance = profile['tolerance_pulse']
    if not math.isfinite(duration) or not 3.0 <= duration <= 6.0:
        raise ValueError('Slow movement duration must be 3..6 seconds')
    if type(tolerance) is not int or not 1 <= tolerance <= 15:
        raise ValueError('Readback tolerance must be 1..15 pulses')
    return positions, duration, tolerance


def decode_position(response):
    # Vendor response.success is True even for a missing physical reply.
    if response is None or not response.success or len(response.state) != 1:
        raise RuntimeError('Missing physical servo response')
    values = response.state[0].position
    if len(values) != 1 or not 0 <= values[0] <= 1000:
        raise RuntimeError('Missing/invalid physical servo position; not a cached joint state')
    return int(values[0])


def at_target(actual, target, tolerance):
    return set(actual) == set(target) and all(
        abs(actual[i] - target[i]) <= tolerance for i in target)


class ZeroMonitor:
    def __init__(self):
        self.count = 0
        self.last = None
        self.bad = False

    def add(self, values, now):
        self.bad |= not values or any(not math.isfinite(v) or abs(v) > 1e-7 for v in values)
        self.count += 1
        self.last = now

    def ready(self, now):
        return (not self.bad and self.count >= 10 and self.last is not None
                and now - self.last <= 0.5)


def serial_owner():
    # Minimal vendor container has no fuser. Inspect descriptors without opening
    # the device; host wrapper additionally checks all users via sudo fuser.
    device = Path('/dev/rrc').resolve(strict=True)
    pids = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdecimal():
            continue
        try:
            if any(fd.resolve() == device for fd in (proc / 'fd').iterdir()):
                pids.append(proc.name)
        except (PermissionError, FileNotFoundError, ProcessLookupError):
            continue
    if len(pids) != 1:
        raise RuntimeError('底盘串口必须只有一个持有者；请关闭厂家 arm 调试窗口')
    cmd = Path('/proc', pids[0], 'cmdline').read_bytes().replace(b'\0', b' ')
    if b'/ros_robot_controller/ros_robot_controller ' not in cmd:
        raise RuntimeError('串口持有者不是已核验的 ROS 底盘驱动')
    return int(pids[0])


def run(profile, apply):
    target, duration, tolerance = validate_profile(profile)
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.qos import qos_profile_sensor_data
    from ros_robot_controller_msgs.msg import GetBusServoCmd, MotorsState, SetBusServoState, BusServoState
    from ros_robot_controller_msgs.srv import GetBusServoState
    from servo_controller_msgs.msg import ServosPosition, ServoPosition
    from std_msgs.msg import Empty
    from ros_robot_controller.ros_robot_controller_sdk import Board

    # Without a server-side timeout even read-only queries can block the driver.
    if 'RAICOM_BOUNDED_SERVO_READ_V1' not in inspect.getsource(Board.bus_servo_read_and_unpack):
        raise RuntimeError('先部署并重新加载有超时的舵机读取补丁；禁止调用旧阻塞接口')
    serial_owner()
    rclpy.init()
    node = rclpy.create_node('mapping_arm_rest')
    vel, motors = ZeroMonitor(), ZeroMonitor()
    interrupted = False
    sent = False
    verified = False
    report = {'target': target, 'duration_s': duration, 'apply': apply}

    def stop_signal(*_):
        nonlocal interrupted
        interrupted = True

    previous = {sig: signal.signal(sig, stop_signal) for sig in (signal.SIGINT, signal.SIGTERM)}
    node.create_subscription(Twist, '/controller/cmd_vel', lambda m: vel.add(
        [m.linear.x, m.linear.y, m.linear.z, m.angular.x, m.angular.y, m.angular.z], time.monotonic()), qos_profile_sensor_data)
    node.create_subscription(MotorsState, '/ros_robot_controller/set_motor', lambda m: motors.add(
        [s.rps for s in m.data] if {s.id for s in m.data} == {1, 2, 3, 4} else [], time.monotonic()), qos_profile_sensor_data)
    node.create_subscription(Empty, '/raicom/nav_stop', lambda _m: stop_signal(), 10)
    command = node.create_publisher(ServosPosition, '/servo_controller', 1)
    stopper = node.create_publisher(SetBusServoState, '/ros_robot_controller/bus_servo/set_state', 1)
    client = node.create_client(GetBusServoState, '/ros_robot_controller/bus_servo/get_state')

    def check():
        if interrupted:
            raise RuntimeError('停止请求：已中断休息姿态')
        now = time.monotonic()
        if not vel.ready(now) or not motors.ready(now):
            raise RuntimeError('底盘零速/电机零指令不连续，拒绝机械臂动作')
        owners = node.get_publishers_info_by_topic('/controller/cmd_vel')
        if len(owners) != 1 or owners[0].node_name != 'cmd_vel_guard':
            raise RuntimeError('底盘速度不是唯一 guard 发布')
        if node.count_publishers('/raicom/motion_enable') or node.count_publishers('/raicom/teleop_cmd'):
            raise RuntimeError('请先退出键盘/遥控会话及运动许可，再执行休息姿态')

    def wait(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.03)
            check()

    def read_all():
        actual = {}
        for sid in IDS:
            check()
            req = GetBusServoState.Request()
            req.cmd = [GetBusServoCmd(id=sid, get_position=1)]
            future = client.call_async(req)
            end = time.monotonic() + 0.8
            while not future.done() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.03)
                check()
            if not future.done():
                client.remove_pending_request(future)
                raise RuntimeError(f'舵机 {sid} 读取超时；不发送/不重试姿态')
            try:
                actual[sid] = decode_position(future.result())
            except Exception as exc:
                raise RuntimeError(f'舵机 {sid} 无有效位置回读: {exc}') from exc
        return actual

    try:
        end = time.monotonic() + 6
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            if vel.bad or motors.bad or interrupted:
                raise RuntimeError('启动观察期间出现非零指令或停止请求')
            if vel.ready(time.monotonic()) and motors.ready(time.monotonic()) and client.service_is_ready() and command.get_subscription_count() == 1 and stopper.get_subscription_count() == 1:
                break
        check()
        if not client.service_is_ready() or command.get_subscription_count() != 1 or stopper.get_subscription_count() != 1:
            raise RuntimeError('现有 ROS 舵机接口未就绪')
        # Discovery settled: reject other pose clients, never contend with them.
        owners = node.get_publishers_info_by_topic('/servo_controller')
        if len(owners) != 1 or owners[0].node_name != node.get_name():
            raise RuntimeError('已有其他机械臂姿态发布者')
        before = read_all()
        report['before'] = before
        print(json.dumps(report, ensure_ascii=False), flush=True)
        if not apply:
            return
        serial_owner()
        check()
        if not at_target(before, target, tolerance):
            msg = ServosPosition(duration=duration, position_unit='pulse')
            msg.position = [ServoPosition(id=i, position=float(target[i])) for i in IDS]
            sent = True
            command.publish(msg)
            wait(duration + 0.6)
        # Two independent physical readbacks; cached /joint_states not used.
        for _ in range(2):
            actual = read_all()
            report['after'] = actual
            if not at_target(actual, target, tolerance):
                raise RuntimeError(f'机械臂未到位：{actual}；不加力、不反复重发')
            wait(0.35)
        verified = True
        report['result'] = 'PHYSICAL_READBACK_PASS'
    except Exception as exc:
        report['result'] = 'FAIL'
        report['error'] = str(exc)
        raise
    finally:
        if sent and not verified:
            # Stop interpolation only; never unload torque (the arm could fall).
            msg = SetBusServoState()
            msg.state = [BusServoState(present_id=[1, i], stop=[1]) for i in IDS]
            for _ in range(3):
                stopper.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.05)
        report.update(command_sent=sent, zero_velocity_samples=vel.count, zero_motor_samples=motors.count)
        print(json.dumps(report, ensure_ascii=False), flush=True)
        node.destroy_node()
        rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, default=Path(__file__).with_name('mapping_rest_pose.json'))
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    try:
        run(json.loads(args.profile.read_text(encoding='utf-8')), args.apply)
    except Exception as exc:
        print('ARM_REST_FAILED:', exc, flush=True)
        raise SystemExit(4)
