"""Scoped continuous local odometry with explicit stationary production handoff.

No velocity/action publishers. STANDBY only publishes private EKF inputs and
health=false. ACTIVE requires the old EKF to be gone, an explicit session token,
fresh sources, unique owners and eight seconds of observed zero commands.
``--standby-only`` additionally forbids activation and requires the observed
production odometry process to remain unchanged for bounded diagnostics.
The sole /odom and odom->base_footprint output retains the EKF source stamp.
Fatal failure is latched. A bounded source pause closes output and the motion
lease until multiple genuinely new measurements confirm recovery; it never
restarts a tracker origin, synthesizes fresh frozen TF, or silently falls back
to command-derived odometry. AMCL still owns map->odom.
"""
import copy
import json
import math
import multiprocessing
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import numpy as np

from odom_continuity import ContinuityAnchor
from probe_sensor_only_ekf import parameters as base_parameters, planar_transform
from scan_keyframe_experiment import sensor_pose_to_base
from runtime_fusion_gate import RuntimeFusionGate, source_sample_is_fresh
from probe_continuous_fusion import stop_child
from scan_match_worker import scan_match_process
from fusion_runtime_contract import (
    activation_requested, build_argument_parser, enforce_standby_contract,
    derive_affinity_roles, initialization_timed_out, validate_arguments,
)

PREFIX = '/raicom/local_odometry'
WORLD = 'raicom_local_odometry_origin'
NODE = 'local_odometry_supervisor'


def observe_match_result(scan, observed_now, response, end_to_end_ms):
    """Read-only profiling seam; production behavior deliberately does nothing."""


def identity(pid):
    try:
        parts = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return None if parts[0] == 'Z' else parts[19]
    except FileNotFoundError:
        return None


def pose_of(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
              *msg.pose.covariance, *msg.twist.covariance,
              msg.twist.twist.linear.x, msg.twist.twist.linear.y,
              msg.twist.twist.angular.z]
    if not np.isfinite(values).all() or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .001:
        raise ValueError('Invalid odometry values')
    return [p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))]


def main():
    parser = build_argument_parser()
    args = validate_arguments(parser.parse_args(), parser)

    try:
        receiver_cpu, diagnostic_cpu = derive_affinity_roles(
            os.cpu_count(), args.main_cpu, args.child_cpu)
    except ValueError as error:
        parser.error(str(error))
    # FastDDS and rclpy worker threads inherit the calling thread's mask. Keep
    # them off the scan matcher before ROS is imported or any node is created.
    try:
        os.sched_setaffinity(0, {receiver_cpu})
        bootstrap_affinity = sorted(os.sched_getaffinity(0))
    except (AttributeError, OSError) as error:
        parser.error('Unable to apply DDS/input CPU affinity: '+str(error))
    if bootstrap_affinity != [receiver_cpu]:
        parser.error('DDS/input CPU affinity verification failed')

    expected_old = None if args.reset_origin else args.old_start
    if identity(args.parent_pid) != args.parent_start or identity(args.old_pid) != expected_old:
        raise RuntimeError('Original process identity mismatch')
    args.directory.mkdir(exist_ok=False)
    events = (args.directory/'events.jsonl').open('x', buffering=1)
    params = base_parameters()['/**']['ros__parameters']
    params.update(world_frame=WORLD, odom_frame=WORLD,
                  pose0=PREFIX+'/laser_pose', imu0=PREFIX+'/imu')
    parameter_path = args.directory/'ekf.yaml'
    parameter_path.write_text(json.dumps({'/**': {'ros__parameters': params}}, indent=2))
    command = ['/usr/bin/taskset', '-c', str(args.child_cpu),
               '/opt/ros/humble/lib/robot_localization/ekf_node', '--ros-args',
               '-r', '__node:=local_sensor_ekf', '-r', '__ns:='+PREFIX,
               '--params-file', str(parameter_path),
               '-r', 'odometry/filtered:='+PREFIX+'/odom_unchecked',
               '-r', '/tf:='+PREFIX+'/tf_disabled', '-r', '/tf_static:='+PREFIX+'/tf_static',
               '-r', '/diagnostics:='+PREFIX+'/diagnostics',
               '-r', 'cmd_vel:='+PREFIX+'/unused_control', '-r', 'set_pose:='+PREFIX+'/unused_set_pose']
    gate = RuntimeFusionGate()
    context = multiprocessing.get_context('fork')
    matcher_connection, matcher_child_connection = context.Pipe(duplex=True)
    matcher = context.Process(target=scan_match_process,
                              args=(matcher_child_connection, args.main_cpu),
                              name='scan_match_worker', daemon=True)
    matcher.start()
    matcher_child_connection.close()
    if not matcher_connection.poll(3):
        matcher.terminate(); matcher.join(timeout=2)
        raise RuntimeError('Scan matcher readiness timed out')
    matcher_ready = matcher_connection.recv()
    if matcher_ready.get('kind') != 'ready' or matcher_ready.get('pid') != matcher.pid:
        matcher.terminate(); matcher.join(timeout=2)
        raise RuntimeError('Scan matcher readiness verification failed: '+str(matcher_ready))

    # Fork happened before ROS/FastDDS created any worker thread.  All ROS and
    # DDS state remains exclusively in this supervisor process.
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.time import Time
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, TransformStamped
    from sensor_msgs.msg import Imu, LaserScan
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    import tf2_ros

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(NODE, enable_rosout=False, start_parameter_services=False)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    gate_lock = threading.RLock()
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, node)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    static_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    imu_pub = node.create_publisher(Imu, PREFIX+'/imu', 5)
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, PREFIX+'/laser_pose', 5)
    static_pub = node.create_publisher(TFMessage, PREFIX+'/tf_static', static_qos)
    health_pub = node.create_publisher(String, PREFIX+'/health', 1)
    state = dict(mode='STANDBY', scan=None, imu=None, forward=False, imu_frame=None,
                 speed_at=None, zero_since=None, speed_nonzero=0, speed_count=0,
                 old_odom=None, private_odom=None, anchor=None, odom_pub=None, tf_pub=None,
                 processed=0, accepted=0, published=0, private_count=0, sequence=0,
                 processing_ms_max=0., stop=False, mount=None)
    state.update(tracker_status='UNINITIALIZED', async_error=None,
                 receiver_affinity=None, matcher_sequence=0,
                 imu_stale_dropped=0)
    state.update(phase='startup', phase_mono=time.monotonic(),
                 phase_thread_cpu=time.thread_time(), scan_received=None)
    child = child_log = None
    start = time.monotonic()

    def now():
        return node.get_clock().now().nanoseconds*1e-9

    def stamp(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def log(kind, **data):
        record = dict(event=kind, elapsed=time.monotonic()-start, ros_time=now(), **data)
        events.write(json.dumps(record, allow_nan=False)+'\n')
        if kind not in ('sample',):
            print(json.dumps(record, allow_nan=False), flush=True)

    def matcher_call(kind, timeout, **payload):
        if not matcher.is_alive():
            raise RuntimeError('Scan matcher process exited')
        state['matcher_sequence'] += 1
        sequence = state['matcher_sequence']
        matcher_connection.send(dict(kind=kind, sequence=sequence, **payload))
        if not matcher_connection.poll(timeout):
            raise RuntimeError('Scan matcher response timed out: '+kind)
        response = matcher_connection.recv()
        if response.get('kind') == 'error':
            raise RuntimeError('Scan matcher worker failure: '+response.get('error','unknown'))
        if response.get('kind') != kind or response.get('sequence') != sequence:
            raise RuntimeError('Scan matcher protocol mismatch')
        return response

    def phase(name):
        mono = time.monotonic()
        thread_cpu = time.thread_time()
        duration = mono-state['phase_mono']
        cpu_duration = thread_cpu-state['phase_thread_cpu']
        if duration > .08 and state['phase'] != 'startup':
            log('slow_main_stage', phase=state['phase'], duration_sec=duration,
                thread_cpu_sec=cpu_duration,
                scheduling_wait_sec=max(0., duration-cpu_duration))
        state['phase'], state['phase_mono'], state['phase_thread_cpu'] = (
            name, mono, thread_cpu)

    def receive_scan(message):
        with gate_lock:
            gate.observe_scan_input(stamp(message))
        state['scan'] = message
        state['scan_received'] = time.monotonic()

    def status():
        with gate_lock:
            return gate.evaluate(now(), state['tracker_status'])

    def publish_health():
        current = status()
        state['sequence'] += 1
        data = dict(session=args.session, sequence=state['sequence'],
                    valid=current['valid'] and state['mode'] == 'ACTIVE',
                    state=state['mode'], source_stamps=current['source_stamps'],
                    reason=current['reason'])
        health_pub.publish(String(data=json.dumps(data, allow_nan=False)))

    def speed(msg):
        vals = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
        if not np.isfinite(vals).all():
            raise RuntimeError('Nonfinite observed command')
        mono = time.monotonic()
        if state['speed_at'] is not None and mono-state['speed_at'] > .3:
            state['zero_since'] = None
        state['speed_count'] += 1
        state['speed_at'] = mono
        if any(vals):
            state['speed_nonzero'] += 1
            state['zero_since'] = None
            if state['mode'] != 'ACTIVE':
                raise RuntimeError('Motion during handoff preparation')
        elif state['zero_since'] is None:
            state['zero_since'] = mono

    def imu(msg):
        state['imu'] = msg
        if not state['forward']:
            return
        values = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                  *msg.angular_velocity_covariance]
        if (msg.header.frame_id != state['imu_frame'] or not np.isfinite(values).all()
                or msg.angular_velocity_covariance[0] < 0):
            raise RuntimeError('Invalid or changed IMU input')
        try:
            fresh = source_sample_is_fresh(now()-stamp(msg), gate.LIMITS['imu'])
        except ValueError as error:
            raise RuntimeError('Invalid IMU source time') from error
        if not fresh:
            # A delayed sample is never forwarded or restamped. The gate closes
            # the lease immediately, waits for genuinely fresh IMU messages,
            # and latches LOST if the bounded one-second recovery expires.
            state['imu_stale_dropped'] += 1
            return
        if gate.observe('imu', stamp(msg)):
            imu_pub.publish(msg)

    def old_odom(msg):
        if state['mode'] == 'STANDBY':
            if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_footprint':
                raise RuntimeError('Old odometry frame mismatch')
            pose_of(msg)
            state['old_odom'] = msg

    def output(msg):
        pose = pose_of(msg)
        if msg.header.frame_id != WORLD or msg.child_frame_id != 'base_footprint':
            raise RuntimeError('Private EKF frame mismatch')
        new = gate.observe('ekf', stamp(msg))
        current = status()
        state['private_count'] += 1
        if not new or not current['valid']:
            return
        state['private_odom'] = msg
        if state['mode'] != 'ACTIVE':
            return
        converted = copy.deepcopy(msg)
        transformed = state['anchor'].transform(pose)
        converted.header.frame_id = 'odom'
        converted.pose.pose.position.x = float(transformed[0])
        converted.pose.pose.position.y = float(transformed[1])
        converted.pose.pose.orientation.x = converted.pose.pose.orientation.y = 0.
        converted.pose.pose.orientation.z = math.sin(transformed[2]/2)
        converted.pose.pose.orientation.w = math.cos(transformed[2]/2)
        converted.pose.covariance = state['anchor'].covariance(msg.pose.covariance)
        # Twist is expressed in child base_footprint, not world: leave unchanged.
        transform = TransformStamped()
        transform.header = copy.deepcopy(converted.header)
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = converted.pose.pose.position.x
        transform.transform.translation.y = converted.pose.pose.position.y
        transform.transform.translation.z = converted.pose.pose.position.z
        transform.transform.rotation = copy.deepcopy(converted.pose.pose.orientation)
        state['odom_pub'].publish(converted)
        state['tf_pub'].publish(TFMessage(transforms=[transform]))
        state['published'] += 1
        if state['published'] == 1:
            log('first_production_output', pose=transformed.tolist(), source_stamp=stamp(msg),
                child_frame=converted.child_frame_id)

    def unexpected_tf(msg):
        raise RuntimeError('Private EKF broadcast TF with publish_tf=false')

    def locked(callback):
        def receive(message):
            with gate_lock:
                return callback(message)
        return receive

    subs = [node.create_subscription(Twist, '/controller/cmd_vel', locked(speed), qos),
            node.create_subscription(Imu, '/imu', locked(imu), qos),
            node.create_subscription(LaserScan, '/scan_stable', receive_scan, qos),
            node.create_subscription(Odometry, '/odom', locked(old_odom), qos),
            node.create_subscription(Odometry, PREFIX+'/odom_unchecked', locked(output), qos),
            node.create_subscription(TFMessage, PREFIX+'/tf_disabled', unexpected_tf, qos)]

    # ROS/DDS workers were created while the calling thread was confined to the
    # input CPU.  This parent is only a light safety/IPC supervisor; the scan
    # matcher already owns its dedicated CPU in a separate process/GIL.
    try:
        os.sched_setaffinity(0, {diagnostic_cpu})
        supervisor_affinity = sorted(os.sched_getaffinity(0))
    except (AttributeError, OSError) as error:
        parser.error('Unable to apply supervisor CPU affinity: '+str(error))
    if supervisor_affinity != [diagnostic_cpu]:
        parser.error('Supervisor CPU affinity verification failed')

    def owners(topic):
        return [i.node_namespace.rstrip('/')+'/'+i.node_name for i in node.get_publishers_info_by_topic(topic)]

    def graph():
        token_path = args.directory/'activate.json'
        enforce_standby_contract(
            standby_only=args.standby_only, directory=args.directory,
            observed_old_start=identity(args.old_pid), expected_old_start=expected_old)
        topic_owners = {t: owners(t) for t in ('/controller/cmd_vel', '/scan_stable', '/imu', '/odom',
                                             PREFIX+'/odom_unchecked', PREFIX+'/health')}
        if topic_owners['/controller/cmd_vel'] != ['/cmd_vel_guard']:
            raise RuntimeError('Velocity output not uniquely owned by guard')
        if any(len(topic_owners[t]) != 1 for t in ('/scan_stable', '/imu')):
            raise RuntimeError('Sensor ownership changed')
        if topic_owners[PREFIX+'/health'] != ['/'+NODE]:
            raise RuntimeError('Health ownership conflict')
        expected = [] if child is None else [PREFIX+'/local_sensor_ekf']
        if topic_owners[PREFIX+'/odom_unchecked'] != expected:
            raise RuntimeError('Private EKF ownership conflict')
        expected = ['/'+NODE] if state['mode'] == 'ACTIVE' else ['/ekf_filter_node']
        if state['mode'] == 'STANDBY' and identity(args.old_pid) is None:
            expected = []
        if topic_owners['/odom'] != expected:
            # DDS participant removal and /proc exit are not atomic, in EITHER
            # order. Only the explicitly requested old-node handoff may bridge
            # this transition; a foreign/duplicate owner remains a hard failure.
            if not (not args.standby_only and state['mode'] == 'STANDBY' and token_path.exists()
                    and topic_owners['/odom'] in ([], ['/ekf_filter_node'])
                    and identity(args.old_pid) in (None, args.old_start)):
                raise RuntimeError('Formal odometry ownership conflict')
        return topic_owners

    def safety(startup=False):
        if state['async_error'] is not None:
            raise RuntimeError('ROS callback failure: '+state['async_error'])
        if identity(args.parent_pid) != args.parent_start:
            raise RuntimeError('Original bringup ended; explicit new handoff required')
        enforce_standby_contract(
            standby_only=args.standby_only, directory=args.directory,
            observed_old_start=identity(args.old_pid), expected_old_start=expected_old)
        # Child-library loading can briefly stall the observer before fusion is
        # started. This phase cannot produce odometry or a valid guard lease.
        # ACTIVE and STANDBY readiness keep the original 300 ms requirement.
        limit = 2. if startup else .3
        if state['speed_at'] is None or time.monotonic()-state['speed_at'] > limit:
            raise RuntimeError('Command observation stale')
        if child is not None and child.poll() is not None:
            raise RuntimeError('Fusion EKF exited')

    def stop(signum, frame):
        state['stop'] = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    receiver_ready = threading.Event()
    def receive_loop():
        try:
            os.sched_setaffinity(0, {receiver_cpu})
            state['receiver_affinity'] = sorted(os.sched_getaffinity(0))
            if state['receiver_affinity'] != [receiver_cpu]:
                raise RuntimeError('ROS input thread CPU affinity verification failed')
        except BaseException as error:
            state['async_error'] = type(error).__name__+': '+str(error)
        finally:
            receiver_ready.set()
        if state['async_error'] is not None:
            return
        try:
            while not state['stop']:
                executor.spin_once(timeout_sec=.02)
        except BaseException as error:
            state['async_error'] = type(error).__name__+': '+str(error)
    receiver = threading.Thread(target=receive_loop, name='ros_input_receiver')
    receiver.start()
    if not receiver_ready.wait(timeout=2):
        state['async_error'] = 'TimeoutError: ROS input thread affinity check timed out'
    try:
        log('start', session=args.session, parent=[args.parent_pid, args.parent_start],
            old=[args.old_pid, args.old_start], reset_origin=args.reset_origin,
            standby_only=args.standby_only, supervisor_affinity=supervisor_affinity,
            receiver_affinity=state['receiver_affinity'],
            matcher_ready=matcher_ready,
            dds_bootstrap_affinity=bootstrap_affinity,
            diagnostic_cpu=diagnostic_cpu, child_cpu=args.child_cpu,
            command=command, parameters=params)
        deadline = time.monotonic()+8
        while time.monotonic() < deadline and not state['stop']:
            time.sleep(.02)
        safety()
        log('preflight_owners', topics=graph())
        first = state['scan']
        if first is None or len(first.ranges) != 254 or not 0 <= now()-stamp(first) <= .3:
            raise RuntimeError('Missing fresh fixed-bin scan')
        if state['imu'] is None or not 0 <= now()-stamp(state['imu']) <= .3:
            raise RuntimeError('Missing fresh IMU')
        t = buffer.lookup_transform('base_footprint', first.header.frame_id, Time()).transform
        q, p = t.rotation, t.translation
        mount = [q.x, q.y, q.z, q.w, p.x, p.y, p.z]
        rotation, offset = planar_transform(mount[:4], mount[4:])
        state['imu_frame'] = state['imu'].header.frame_id
        imu_tf = buffer.lookup_transform('base_footprint', state['imu_frame'], Time())
        static_pub.publish(TFMessage(transforms=[imu_tf]))
        log('mount', values=mount, lidar_frame=first.header.frame_id, imu_frame=state['imu_frame'])
        child_log = (args.directory/'ekf.log').open('x')
        child = subprocess.Popen(command, stdout=child_log, stderr=subprocess.STDOUT)
        affinity_deadline = time.monotonic()+1
        while True:
            if child.poll() is not None:
                raise RuntimeError('Private EKF exited before CPU-affinity verification')
            child_affinity = sorted(os.sched_getaffinity(child.pid))
            if child_affinity == [args.child_cpu]:
                break
            if time.monotonic() >= affinity_deadline:
                raise RuntimeError('Private EKF CPU affinity verification failed')
            time.sleep(.01)
        child_scheduler = os.sched_getscheduler(child.pid)
        child_priority = os.sched_getparam(child.pid).sched_priority
        if child_scheduler != os.SCHED_OTHER or child_priority != 0:
            raise RuntimeError('Private EKF unexpectedly inherited realtime scheduling')
        log('child_affinity', pid=child.pid, affinity=child_affinity,
            scheduler=child_scheduler, priority=child_priority)
        matcher_affinity = sorted(os.sched_getaffinity(matcher.pid))
        matcher_scheduler = os.sched_getscheduler(matcher.pid)
        matcher_priority = os.sched_getparam(matcher.pid).sched_priority
        if (matcher_affinity != [args.main_cpu]
                or matcher_scheduler != os.SCHED_RR or matcher_priority != 1):
            raise RuntimeError('Scan matcher process scheduling changed')
        log('matcher_process', pid=matcher.pid, affinity=matcher_affinity,
            scheduler=matcher_scheduler, priority=matcher_priority)
        deadline = time.monotonic()+5
        while time.monotonic() < deadline and not state['stop']:
            time.sleep(.02)
            safety(startup=True)
        safety()
        graph()
        state['forward'] = True
        ready = time.monotonic()
        next_graph = next_health = next_log = next_tracker_tick = ready
        last_stamp = None
        last_gate_state = None
        while not state['stop']:
            # A dedicated executor receives sensors/outputs continuously while
            # NumPy matching runs here. Never make IMU/EKF receipt wait for ICP.
            time.sleep(.005)
            phase('safety')
            safety()
            mono = time.monotonic()
            msg = state['scan']
            if stamp(msg) != last_stamp:
                phase('scan_match')
                last_stamp = stamp(msg)
                scan = dict(stamp=last_stamp, frame_id=msg.header.frame_id,
                            angle_min=msg.angle_min, angle_increment=msg.angle_increment,
                            range_min=msg.range_min, range_max=msg.range_max,
                            ranges=[float(v) if math.isfinite(v) else None for v in msg.ranges])
                before = time.monotonic()
                observed_now = now()
                response = matcher_call('update', .35, scan=scan, now=observed_now)
                result = response['result']
                match_wall_ms = (time.monotonic()-before)*1000
                observe_match_result(scan, observed_now, response, match_wall_ms)
                worker_wall_ms = response['worker_wall_ms']
                worker_thread_cpu_ms = response['worker_thread_cpu_ms']
                if not result['valid']:
                    log('scan_rejected', stamp=last_stamp, reason=result.get('reason'),
                        tracker_status=result.get('status'), processing_ms=match_wall_ms,
                        processing_thread_cpu_ms=worker_thread_cpu_ms,
                        scheduling_wait_ms=max(0., worker_wall_ms-worker_thread_cpu_ms),
                        worker_wall_ms=worker_wall_ms,
                        ipc_supervisor_overhead_ms=max(0., match_wall_ms-worker_wall_ms),
                        tracker_result=result)
                elif result.get('keyframe_changed') or result.get('reference_reused_from') is not None:
                    log('reference_changed', stamp=last_stamp, tracker_result=result)
                phase('laser_publish')
                with gate_lock:
                    state['tracker_status'] = response['tracker_status']
                ms = (time.monotonic()-before)*1000
                state['processing_ms_max'] = max(state['processing_ms_max'], ms)
                state['processed'] += 1
                with gate_lock:
                    laser_accepted = result['valid'] and gate.observe('laser', result['stamp'])
                if laser_accepted:
                    base_pose = sensor_pose_to_base(result['pose'], rotation, offset)
                    measurement = PoseWithCovarianceStamped()
                    measurement.header.frame_id = WORLD
                    measurement.header.stamp = copy.deepcopy(msg.header.stamp)
                    measurement.pose.pose.position.x = float(base_pose[0])
                    measurement.pose.pose.position.y = float(base_pose[1])
                    measurement.pose.pose.orientation.z = math.sin(base_pose[2]/2)
                    measurement.pose.pose.orientation.w = math.cos(base_pose[2]/2)
                    for index, value in zip((0,7,14,21,28,35), (.02**2,.02**2,1e6,1e6,1e6,math.radians(1)**2)):
                        measurement.pose.covariance[index] = value
                    pose_pub.publish(measurement)
                    state['accepted'] += 1
            phase('gate')
            tick_now = now()
            if mono >= next_tracker_tick:
                tick_response = matcher_call('tick', .1, now=tick_now)
                with gate_lock:
                    state['tracker_status'] = tick_response['tracker_status']
                next_tracker_tick = mono+.05
            current = status()
            if current['state'] != last_gate_state:
                log('gate_transition', **current)
                last_gate_state = current['state']
            if current['state'] == 'LOST':
                raise RuntimeError(current['reason'])
            # ``current`` is the lock-protected evaluation returned above.
            # Reading gate.established here raced the receiver thread's own
            # evaluation, whose bounded-pause implementation toggles that bit
            # internally for only a few microseconds.
            if initialization_timed_out(
                    ready_elapsed=mono-ready, gate_state=current['state']):
                raise RuntimeError('Fusion failed to initialize')
            if mono >= next_graph:
                phase('graph')
                graph()
                phase('mount_lookup')
                t = buffer.lookup_transform('base_footprint', first.header.frame_id, Time()).transform
                q, p = t.rotation, t.translation
                if not np.allclose([q.x,q.y,q.z,q.w,p.x,p.y,p.z], mount, atol=1e-6, rtol=0):
                    raise RuntimeError('Lidar mounting changed')
                next_graph = mono+.5
            phase('activation')
            token_path = args.directory/'activate.json'
            if activation_requested(
                    standby_only=args.standby_only, mode=state['mode'],
                    ready_elapsed=mono-ready, token_path=token_path):
                token = json.loads(token_path.read_text())
                if token != {'session': args.session, 'old_pid': args.old_pid, 'old_start': args.old_start}:
                    raise RuntimeError('Activation token mismatch')
                if identity(args.old_pid) is None and owners('/odom') == []:
                    old, private = state['old_odom'], state['private_odom']
                    old_valid = args.reset_origin or (old is not None and 0 <= now()-stamp(old) <= 20)
                    if (not current['valid'] or not old_valid or private is None
                            or state['zero_since'] is None or mono-state['zero_since'] < 8):
                        raise RuntimeError('Stationary continuity handoff requirements failed')
                    previous = [0., 0., 0.] if args.reset_origin else pose_of(old)
                    state['anchor'] = ContinuityAnchor(previous, pose_of(private))
                    log('handoff', reset_origin=args.reset_origin, previous_pose=previous,
                        previous_stamp=None if args.reset_origin else stamp(old),
                        candidate_pose=pose_of(private), candidate_stamp=stamp(private),
                        offset_xy=state['anchor'].xy.tolist(), offset_yaw=state['anchor'].yaw,
                        owners_before=owners('/odom'))
                    state['odom_pub'] = node.create_publisher(Odometry, '/odom', 10)
                    state['tf_pub'] = node.create_publisher(TFMessage, '/tf', 10)
                    state['mode'] = 'ACTIVE'
                    next_graph = mono+2
                    # Wait for the next genuinely new EKF stamp; do not republish
                    # the sample used as anchor or fabricate an activation pose.
            if mono >= next_health:
                phase('health_publish')
                publish_health()
                next_health = mono+.1
            if mono >= next_log:
                phase('sample_log')
                private = state['private_odom']
                log('sample', mode=state['mode'], gate=current,
                    processed=state['processed'], accepted=state['accepted'],
                    published=state['published'], private_count=state['private_count'],
                    processing_ms_max=state['processing_ms_max'],
                    ekf_pauses=gate.pauses, ekf_resumptions=gate.resumptions,
                    imu_stale_dropped=state['imu_stale_dropped'],
                    speed_count=state['speed_count'], speed_nonzero=state['speed_nonzero'],
                    private_pose=None if private is None else pose_of(private))
                next_log = mono+1
            phase('idle')
    except BaseException as error:
        gate.fail(type(error).__name__+': '+str(error))
        # Revoke before potentially slow evidence serialization. A trace never
        # prolongs a valid lease or supplies a replacement odometry sample.
        publish_health()
        log('failure', reason=gate.failure, phase=state['phase'],
            phase_elapsed=time.monotonic()-state['phase_mono'],
            phase_thread_cpu=time.thread_time()-state['phase_thread_cpu'],
            phase_scheduling_wait=max(
                0., (time.monotonic()-state['phase_mono'])
                -(time.thread_time()-state['phase_thread_cpu'])),
            scan_receive_age=None if state['scan_received'] is None else time.monotonic()-state['scan_received'],
            latest_scan_age=None if state['scan'] is None else now()-stamp(state['scan']),
            source_stamps=gate.stamps)
        if state['old_odom'] is not None:
            log('last_old_odom_recovery', pose=pose_of(state['old_odom']), stamp=stamp(state['old_odom']))
        try:
            trace_response = matcher_call('trace', 1., reason=gate.failure)
            trace_payload = trace_response['payload']
            raw = json.dumps(trace_payload, allow_nan=False, separators=(',',':')).encode()
            trace_path = args.directory/'failure_replay.json'
            with trace_path.open('xb') as trace_output:
                trace_output.write(raw)
            log('failure_trace', path=str(trace_path), bytes=len(raw),
                frames=len(trace_payload['records']))
        except (RuntimeError,EOFError,BrokenPipeError,OSError,ValueError,TypeError) as trace_error:
            log('failure_trace_unavailable', reason=str(trace_error))
        raise
    finally:
        state['mode'] = 'STOPPED'
        state['forward'] = False
        state['stop'] = True
        receiver.join(timeout=3)
        publish_health()
        matcher_stopped = matcher_forced = False
        try:
            if matcher.is_alive():
                matcher_stopped = bool(matcher_call('stop', .5).get('stopped'))
        except (RuntimeError,EOFError,BrokenPipeError,OSError) as matcher_error:
            log('matcher_stop_failed', reason=str(matcher_error))
        matcher.join(timeout=2)
        if matcher.is_alive():
            matcher.terminate()
            matcher.join(timeout=2)
            matcher_forced = True
        log('matcher_exit', exitcode=matcher.exitcode, reaped=not matcher.is_alive(),
            forced=matcher_forced, protocol_stopped=matcher_stopped)
        matcher_connection.close()
        if child is not None:
            log('child_exit', **stop_child(child))
        if child_log is not None:
            child_log.close()
        log('end', processed=state['processed'], accepted=state['accepted'], published=state['published'])
        events.close()
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
