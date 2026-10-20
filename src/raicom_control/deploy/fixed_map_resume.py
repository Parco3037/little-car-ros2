"""Fail-closed one-command recovery for the preserved RAICOM fixed map.

Run on the Raspberry Pi host.  The default mode is read-only.  The explicit
``--prepare-startup`` mode never publishes velocity, navigation, arm or gripper
commands: it rotates the guard session, replaces the vendor EKF with the
verified local odometry runtime, loads the fixed map, applies the saved start
area as an initial estimate, and proves stationary scan/map overlap.

Any failed step leaves cmd_vel_guard without a healthy lease, hence stopped.
It never restarts Docker, recreates a container, rescans, saves or edits a map.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import secrets
import shlex
import subprocess
import time


CONTAINER = 'MentorPi'
MAP_YAML = '/home/ubuntu/ros2_ws/src/slam/maps/map_01_2cm_20260901_0240.yaml'
MAP_IMAGE = '/home/ubuntu/ros2_ws/src/slam/maps/map_01_2cm_20260901_0240.pgm'
ROUTE = '/home/ubuntu/ros2_ws/src/raicom_control/config/route_map_01_2cm_20260901_0240.json'
MAP_YAML_SHA256 = '1ba1da589de00e7fa421eab16e3232903050e1aebb3c3195c2a02250f8ead828'
MAP_IMAGE_SHA256 = '67bf9f48d8e61e26b0ebd686e1390f5006b0409a4d918ceb210f9672dcb6017a'
BUNDLE = '/home/ubuntu/raicom_runtime/current/deploy'
HOST_EVIDENCE = Path('/home/pi/raicom_backups')
GUARD_DROPIN = Path('/etc/systemd/system/raicom_cmd_vel_guard.service.d/60-local-odometry.conf')
GUARD_ZERO_OBSERVE_SEC = 10
GUARD_ZERO_MIN_ROWS = 40
ALIGNMENT_OBSERVE_SEC = 12
BASE_STAMP_TOPICS = ('/odom', '/odom_raw', '/scan', '/scan_raw')
BASE_STAMP_MAX_AGE_SEC = 1.0
BASE_STAMP_FUTURE_TOLERANCE_SEC = 0.05
BASE_STAMP_OBSERVE_TIMEOUT_SEC = 8
ROS_ENV = (
    'source /opt/ros/humble/setup.bash; '
    'source /home/ubuntu/third_party_ros2/third_party_ws/install/setup.bash; '
    'source /home/ubuntu/ros2_ws/install/setup.bash; '
    'export FASTDDS_BUILTIN_TRANSPORTS=UDPv4; '
)


class RecoveryError(RuntimeError):
    pass


class Runner:
    def run(self, argv, *, allowed=(0,), timeout=60, input_text=None):
        result = subprocess.run(
            [str(value) for value in argv], input=input_text, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout,
        )
        if result.returncode not in allowed:
            raise RecoveryError(
                f'command failed rc={result.returncode}: {argv!r}; '
                f'stdout={result.stdout[-2000:]!r}; stderr={result.stderr[-2000:]!r}'
            )
        return result

    def docker_shell(self, command, *, allowed=(0,), timeout=60, detached=False,
                     user=None, workdir=None, env=()):
        argv = ['docker', 'exec']
        if detached:
            argv.append('-d')
        if user:
            argv += ['-u', user]
        if workdir:
            argv += ['-w', workdir]
        for name, value in env:
            argv += ['-e', f'{name}={value}']
        argv += [CONTAINER, '/bin/bash', '-lc', command]
        return self.run(argv, allowed=allowed, timeout=timeout)


def make_session(now=None, suffix=None):
    now = now or datetime.now()
    suffix = suffix or secrets.token_hex(3)
    if not re.fullmatch(r'[0-9a-f]{6}', suffix):
        raise ValueError('Session suffix must be six lowercase hex characters')
    return 'local-odom-' + now.strftime('%Y%m%d-%H%M%S-') + suffix


def parse_throttled(text):
    match = re.fullmatch(r'\s*throttled=0x([0-9a-fA-F]+)\s*', text)
    if not match:
        raise RecoveryError('Unrecognised vcgencmd output')
    value = int(match.group(1), 16)
    return {'raw': value, 'current_fault_bits': value & 0xF,
            'historical_fault_bits': (value >> 16) & 0xF}


def parse_cmd_vel_csv(text):
    rows, nonzero = 0, 0
    for line in text.splitlines():
        fields = line.strip().split(',')
        if len(fields) != 6:
            continue
        try:
            values = [float(value) for value in fields]
        except ValueError:
            continue
        rows += 1
        nonzero += int(any(value != 0.0 for value in values))
    return {'rows': rows, 'nonzero': nonzero}


def parse_ros_stamp(text):
    """Parse the exact ``ros2 topic echo --field header.stamp`` payload."""
    match = re.fullmatch(
        r'\s*sec:\s*([0-9]+)\s+nanosec:\s*([0-9]+)\s+---\s*', text)
    if not match:
        raise RecoveryError('Unrecognised ROS header stamp output')
    sec, nanosec = map(int, match.groups())
    if nanosec >= 1_000_000_000:
        raise RecoveryError('Invalid ROS header nanosecond field')
    return sec + nanosec * 1e-9


def stamp_age_is_fresh(age_sec):
    return (-BASE_STAMP_FUTURE_TOLERANCE_SEC <= age_sec
            <= BASE_STAMP_MAX_AGE_SEC)


def guard_zero_result_is_safe(result):
    """Require enough real samples and reject any nonzero component."""
    return (result['rows'] >= GUARD_ZERO_MIN_ROWS
            and result['nonzero'] == 0)


def update_guard_dropin(text, session):
    if not re.fullmatch(r'local-odom-[0-9]{8}-[0-9]{6}-[0-9a-f]{6}', session):
        raise RecoveryError('Invalid generated session')
    pattern = r'(?<=localization_session:=)[A-Za-z0-9._-]+'
    updated, count = re.subn(pattern, session, text)
    if count != 1:
        raise RecoveryError(f'Expected exactly one guard session, found {count}')
    return updated


def container_pids(runner, pattern):
    # Do not use ``pgrep -f PATTERN`` through a shell: PATTERN then appears in
    # that shell's own command line and can be counted as a false process.
    result = runner.run(
        ['docker', 'exec', CONTAINER, 'ps', '-eo', 'pid=,ppid=,args='], timeout=10)
    expression = re.compile(pattern)
    found = []
    for line in result.stdout.splitlines():
        fields = line.strip().split(None, 2)
        if (len(fields) == 3 and fields[0].isdigit() and fields[1].isdigit()
                and expression.search(fields[2])):
            found.append((int(fields[0]), int(fields[1])))
    # A multiprocessing fork inherits the supervisor's command line.  Count
    # matching process-tree roots so one runtime plus its matcher is not
    # reported as two independent runtime owners.
    matching = {pid for pid, _ in found}
    return [pid for pid, parent in found if parent not in matching]


def pid_start(runner, pid):
    result = runner.docker_shell(f'cat /proc/{pid}/stat', timeout=10)
    try:
        return result.stdout.rsplit(')', 1)[1].split()[19]
    except (IndexError, ValueError) as error:
        raise RecoveryError(f'Cannot parse process identity for {pid}') from error


def wait_for(predicate, *, timeout, description):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        try:
            last = predicate()
            if last:
                return last
        except (RecoveryError, json.JSONDecodeError) as error:
            last = str(error)
        time.sleep(.5)
    raise RecoveryError(f'Timed out waiting for {description}; last={last!r}')


def container_hashes(runner):
    result = runner.run(['docker', 'exec', CONTAINER, 'sha256sum', MAP_YAML, MAP_IMAGE], timeout=20)
    found = {line.split()[1]: line.split()[0] for line in result.stdout.splitlines() if len(line.split()) == 2}
    return found


def collect_status(runner):
    services = {}
    for name in ('start_app_node.service', 'raicom_cmd_vel_guard.service'):
        result = runner.run(['systemctl', 'is-active', name], allowed=(0, 3), timeout=10)
        services[name] = result.stdout.strip()
    container = runner.run(
        ['docker', 'inspect', '-f', '{{.State.Running}}', CONTAINER], allowed=(0, 1), timeout=10)
    container_running = container.stdout.strip() == 'true'
    throttled = parse_throttled(runner.run(['vcgencmd', 'get_throttled'], timeout=10).stdout)
    ntp = runner.run(
        ['timedatectl', 'show', '-p', 'NTPSynchronized', '--value'], timeout=10)
    ntp_value = ntp.stdout.strip()
    if ntp_value not in ('yes', 'no'):
        raise RecoveryError(f'Unrecognised NTPSynchronized value: {ntp_value!r}')
    patterns = {
        'controller': r'ros_robot_controller/lib/ros_robot_controller/ros_robot_controller ',
        'lidar': r'oradar_lidar/oradar_scan ',
        'bringup': r'^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch bringup bringup.launch.py$',
        'vendor_ekf': r'^/opt/ros/humble/lib/robot_localization/ekf_node .*__node:=ekf_filter_node ',
        'runtime': r'continuous_fusion_runtime.py --directory ',
        'scan_normalizer': r'/raicom_control/scan_normalizer ',
        'map_server': r'/nav2_map_server/map_server ',
        'amcl': r'/nav2_amcl/amcl ',
    }
    processes = ({name: container_pids(runner, pattern) for name, pattern in patterns.items()}
                 if container_running else {name: [] for name in patterns})
    return {
        'services': services,
        'container_running': container_running,
        'clock': {'ntp_synchronized': ntp_value == 'yes'},
        'throttled': throttled,
        'map_sha256': container_hashes(runner) if container_running else {},
        'processes': processes,
    }


def assert_prepare_preflight(runner, status):
    if status['services'] != {
            'start_app_node.service': 'active', 'raicom_cmd_vel_guard.service': 'active'}:
        raise RecoveryError('Base or guard service is not active; do not auto-restart it')
    if not status['container_running']:
        raise RecoveryError('MentorPi is not running; container mutation is forbidden')
    if status.get('clock') != {'ntp_synchronized': True}:
        raise RecoveryError('Host clock is not NTP-synchronised; wait before localization recovery')
    if status['throttled']['current_fault_bits']:
        raise RecoveryError('Current power/throttle fault is active')
    expected = {MAP_YAML: MAP_YAML_SHA256, MAP_IMAGE: MAP_IMAGE_SHA256}
    if status['map_sha256'] != expected:
        raise RecoveryError('Fixed map YAML/PGM hashes changed')
    for name in ('controller', 'lidar', 'bringup'):
        if len(status['processes'][name]) != 1:
            raise RecoveryError(f'Expected one {name}, found {status["processes"][name]}')
    # A failed attempt can already have stopped the vendor EKF before the local
    # runtime reaches STANDBY.  With no runtime/map/AMCL below, zero vendor EKFs
    # is a safe, stopped retry state; the new runtime still verifies that /odom
    # has no foreign DDS owner before it can activate.  Duplicates remain fatal.
    if len(status['processes']['vendor_ekf']) > 1:
        raise RecoveryError(
            f'Expected zero or one vendor_ekf, found {status["processes"]["vendor_ekf"]}')
    if len(status['processes']['scan_normalizer']) > 1:
        raise RecoveryError(
            'Expected zero or one scan_normalizer, found '
            f'{status["processes"]["scan_normalizer"]}')
    for name in ('runtime', 'map_server', 'amcl'):
        if status['processes'][name]:
            raise RecoveryError(f'{name} already exists; use --status, never stack a second copy')
    required = ('continuous_fusion_runtime.py', 'scan_match_worker.py',
                'fusion_runtime_contract.py', 'odom_continuity.py',
                'probe_sensor_only_ekf.py', 'scan_keyframe_experiment.py',
                'runtime_fusion_gate.py', 'probe_continuous_fusion.py',
                'map_localization_recovery.py', 'verify_scan_map_alignment.py')
    command = ' && '.join('test -f ' + shlex.quote(BUNDLE + '/' + name) for name in required)
    runner.docker_shell(command, timeout=10)


def observe_base_stamp_freshness(runner):
    """Reject stale/future ROS streams before rotating the production guard."""
    observed = {}
    for topic in BASE_STAMP_TOPICS:
        result = runner.docker_shell(
            ROS_ENV + f'timeout {BASE_STAMP_OBSERVE_TIMEOUT_SEC} '
            + 'ros2 topic echo ' + shlex.quote(topic)
            + ' --once --field header.stamp',
            timeout=BASE_STAMP_OBSERVE_TIMEOUT_SEC + 5,
        )
        stamp = parse_ros_stamp(result.stdout)
        age = time.time() - stamp
        observed[topic] = {'stamp': stamp, 'age_sec': age}
        if not stamp_age_is_fresh(age):
            raise RecoveryError(
                f'Stale or future ROS stamp on {topic}: age_sec={age:.6f}')
    return observed


def scan_normalizer_launch_value(status):
    """Start after a clean boot, or preserve the sole already-running owner."""
    count = len(status['processes']['scan_normalizer'])
    if count > 1:
        raise RecoveryError(f'Expected zero or one scan_normalizer, found {count}')
    return 'true' if count == 0 else 'false'


def write_guard_session(session, evidence):
    original = GUARD_DROPIN.read_text(encoding='utf-8')
    (evidence / '60-local-odometry.conf.before').write_text(original, encoding='utf-8')
    updated = update_guard_dropin(original, session)
    temporary = GUARD_DROPIN.with_suffix('.conf.raicom-new')
    temporary.write_text(updated, encoding='utf-8')
    os.chmod(temporary, 0o644)
    os.replace(temporary, GUARD_DROPIN)


def read_events(runner, directory):
    result = runner.docker_shell(
        'test -f ' + shlex.quote(directory + '/events.jsonl') + ' && cat '
        + shlex.quote(directory + '/events.jsonl'), allowed=(0, 1), timeout=10)
    events = []
    for line in result.stdout.splitlines():
        if line.strip():
            events.append(json.loads(line))
    return events


def open_ui(runner, evidence_dir):
    rviz_config = BUNDLE + '/../config/localization_watch_1fps.rviz'
    rviz_log = evidence_dir + '/rviz.log'
    if not container_pids(runner, r'^rviz2 .*localization_watch_1fps.rviz'):
        runner.docker_shell(
            ROS_ENV + 'exec rviz2 -d ' + shlex.quote(rviz_config) + ' >' + shlex.quote(rviz_log) + ' 2>&1',
            detached=True, user='ubuntu', env=(('DISPLAY', ':0'), ('QT_X11_NO_MITSHM', '1')),
        )
    if not container_pids(runner, r'xterm -title RAICOM-Teleop '):
        command = (
            'exec xterm -title RAICOM-Teleop -geometry 96x26+12+12 -fn fixed -e bash -lc '
            + shlex.quote(ROS_ENV + 'ros2 launch peripherals teleop_key_control.launch.py; exec bash')
        )
        runner.docker_shell(command, detached=True, user='ubuntu', env=(('DISPLAY', ':0'),))


def prepare_startup(runner, *, open_windows):
    if os.geteuid() != 0:
        raise RecoveryError('Run with sudo -n; root is required only for the guard drop-in')
    status = collect_status(runner)
    assert_prepare_preflight(runner, status)
    session = make_session()
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    evidence = HOST_EVIDENCE / f'{stamp}_fixed_map_resume'
    evidence.mkdir(mode=0o700, parents=False, exist_ok=False)
    (evidence / 'preflight.json').write_text(json.dumps(status, indent=2), encoding='utf-8')
    stamp_freshness = observe_base_stamp_freshness(runner)
    (evidence / 'base_stamp_freshness.json').write_text(
        json.dumps(stamp_freshness, indent=2), encoding='utf-8')
    write_guard_session(session, evidence)
    runner.run(['systemctl', 'daemon-reload'], timeout=20)
    runner.run(['systemctl', 'restart', 'raicom_cmd_vel_guard.service'], timeout=30)

    # The guard was just restarted with a new session.  Give a cold Fast DDS
    # participant enough discovery time while retaining the same minimum of 40
    # real six-axis samples.  A 5 s wall window once yielded only 35 all-zero
    # rows, even though the already-discovered follow-up produced 100/100 zero.
    zero = runner.docker_shell(
        ROS_ENV + f'timeout {GUARD_ZERO_OBSERVE_SEC} '
        'ros2 topic echo /controller/cmd_vel --csv',
        allowed=(0, 124), timeout=GUARD_ZERO_OBSERVE_SEC + 5,
    )
    zero_result = parse_cmd_vel_csv(zero.stdout)
    (evidence / 'guard_zero.csv').write_text(zero.stdout, encoding='utf-8')
    if not guard_zero_result_is_safe(zero_result):
        raise RecoveryError(f'Guard zero-speed gate failed: {zero_result}')

    container_evidence = f'/home/ubuntu/raicom_backups/{stamp}_fixed_map_resume'
    runner.docker_shell('mkdir -p ' + shlex.quote(container_evidence), user='ubuntu')
    localization_log = container_evidence + '/localization.log'
    start_scan_normalizer = scan_normalizer_launch_value(status)
    runner.docker_shell(
        ROS_ENV + 'exec ros2 launch raicom_control home_localization.launch.py map_file:='
        + shlex.quote(MAP_YAML) + ' start_scan_normalizer:=' + start_scan_normalizer + ' >'
        + shlex.quote(localization_log) + ' 2>&1',
        detached=True, user='ubuntu', workdir='/home/ubuntu',
    )
    wait_for(lambda: len(container_pids(runner, r'/nav2_map_server/map_server ')) == 1
             and len(container_pids(runner, r'/nav2_amcl/amcl ')) == 1
             and len(container_pids(runner, r'/raicom_control/scan_normalizer ')) == 1,
             timeout=20, description='one map_server, one AMCL and one scan_normalizer')

    parent = status['processes']['bringup'][0]
    parent_start = pid_start(runner, parent)
    old = status['processes']['vendor_ekf']
    if old:
        runner.docker_shell(f'kill -TERM {old[0]}')
        wait_for(lambda: not container_pids(runner, r'^/opt/ros/humble/lib/robot_localization/ekf_node .*__node:=ekf_filter_node '),
                 timeout=10, description='vendor EKF exit')
    sentinel = int(runner.docker_shell('cat /proc/sys/kernel/pid_max').stdout.strip())
    if runner.docker_shell(f'test ! -e /proc/{sentinel}/stat', allowed=(0, 1)).returncode != 0:
        raise RecoveryError('PID sentinel unexpectedly exists')

    runtime_dir = container_evidence + '/runtime'
    runtime_log = container_evidence + '/runtime.log'
    runtime_command = (
        ROS_ENV + 'exec python3 -B ' + shlex.quote(BUNDLE + '/continuous_fusion_runtime.py')
        + ' --directory ' + shlex.quote(runtime_dir)
        + ' --session ' + shlex.quote(session)
        + f' --parent-pid {parent} --parent-start {shlex.quote(parent_start)}'
        + f' --old-pid {sentinel} --old-start reset-sentinel'
        + ' --main-cpu 2 --child-cpu 3 --reset-origin'
        + ' >' + shlex.quote(runtime_log) + ' 2>&1'
    )
    runner.docker_shell(
        runtime_command, detached=True, workdir=BUNDLE,
        env=(('OPENBLAS_NUM_THREADS', '1'), ('OMP_NUM_THREADS', '1'),
             ('MKL_NUM_THREADS', '1'), ('NUMEXPR_NUM_THREADS', '1')),
    )
    wait_for(
        lambda: next((event for event in read_events(runner, runtime_dir)
                      if event.get('event') == 'sample'
                      and event.get('mode') == 'STANDBY'
                      and event.get('gate', {}).get('valid') is True), None),
        timeout=25, description='valid STANDBY sample',
    )
    token = {'session': session, 'old_pid': sentinel, 'old_start': 'reset-sentinel'}
    writer = (
        "import json,os,pathlib,sys; p=pathlib.Path(sys.argv[1]); "
        "t=p.with_suffix('.tmp'); t.write_text(sys.stdin.read()); os.replace(t,p)"
    )
    runner.run(
        ['docker', 'exec', '-i', CONTAINER, 'python3', '-c', writer,
         runtime_dir + '/activate.json'], input_text=json.dumps(token), timeout=10,
    )
    handoff = wait_for(
        lambda: next((event for event in read_events(runner, runtime_dir)
                      if event.get('event') == 'handoff'), None),
        timeout=30, description='production odometry handoff',
    )

    prior_result = runner.docker_shell(
        ROS_ENV + 'python3 -B ' + shlex.quote(BUNDLE + '/map_localization_recovery.py')
        + ' --route-start ' + shlex.quote(ROUTE)
        + ' --map-yaml ' + shlex.quote(MAP_YAML)
        + ' --map-image ' + shlex.quote(MAP_IMAGE) + ' --apply',
        user='ubuntu', workdir=BUNDLE, timeout=30,
    )
    (evidence / 'initialpose.json').write_text(prior_result.stdout, encoding='utf-8')

    alignment_path = container_evidence + '/startup_alignment.json'
    runner.docker_shell(
        ROS_ENV + 'python3 -B ' + shlex.quote(BUNDLE + '/verify_scan_map_alignment.py')
        + f' --seconds {ALIGNMENT_OBSERVE_SEC} >' + shlex.quote(alignment_path),
        user='ubuntu', workdir=BUNDLE, timeout=ALIGNMENT_OBSERVE_SEC + 10,
    )
    runner.run(['docker', 'cp', f'{CONTAINER}:{alignment_path}', str(evidence / 'startup_alignment.json')])
    alignment = json.loads((evidence / 'startup_alignment.json').read_text(encoding='utf-8'))
    frames = alignment.get('frames', [])
    if (len(frames) < 8 or not all(frame['all_visible'].get('trial_criteria_met') for frame in frames)
            or alignment.get('velocity_nonzero') != 0 or alignment.get('map_changed') is not False
            or alignment.get('publishers') != {
                '/map': ['/map_server'], '/scan_stable': ['/scan_normalizer'],
                '/controller/cmd_vel': ['/cmd_vel_guard']}):
        raise RecoveryError('Startup scan/map alignment gate failed')
    if open_windows:
        open_ui(runner, container_evidence)
    result = {
        'result': 'READY_AT_STARTUP', 'session': session,
        'evidence': str(evidence), 'zero_speed': zero_result,
        'handoff': handoff, 'alignment_frames': len(frames),
        'alignment_p90_max_m': max(frame['all_visible']['p90_m'] for frame in frames),
        'actions_sent': {'velocity': False, 'navigation': False, 'arm': False, 'gripper': False},
    }
    (evidence / 'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    return result


def main():
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--status', action='store_true', help='Read-only; this is the default')
    mode.add_argument('--prepare-startup', action='store_true',
                      help='Requires the robot to be physically placed at the saved startup area')
    parser.add_argument('--startup-area-confirmed', action='store_true',
                        help='Explicit physical placement confirmation for --prepare-startup')
    parser.add_argument('--open-ui', action='store_true')
    args = parser.parse_args()
    runner = Runner()
    if not args.prepare_startup:
        print(json.dumps(collect_status(runner), indent=2), flush=True)
        return
    if not args.startup_area_confirmed:
        parser.error('--prepare-startup requires --startup-area-confirmed')
    print(json.dumps(prepare_startup(runner, open_windows=args.open_ui), indent=2), flush=True)


if __name__ == '__main__':
    main()
