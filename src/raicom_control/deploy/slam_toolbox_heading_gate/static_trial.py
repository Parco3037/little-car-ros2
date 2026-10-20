#!/usr/bin/env python3
"""One bounded, headless SLAM trial on real stationary sensors; saves no map."""
import argparse
import ctypes
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

EXPECTED_LIBRARY = '1788d199ad57e4b868a245b37e5353ceabb34333370eefe3e52754b9aeba83be'
LIBRARY = Path('/home/ubuntu/raicom_tools/slam_toolbox_2_6_8_heading_v1/lib/libtoolbox_common.so')

def owned_processes(pgid):
    result = []
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            if os.getpgid(int(path.name)) == pgid:
                result.append(int(path.name))
        except ProcessLookupError:
            pass
    return result

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=int,default=120)
    args=parser.parse_args()
    if not 30 <= args.seconds <= 180:
        raise ValueError('30..180 seconds only')
    if os.environ.get('ROS_DOMAIN_ID','0') != '0':
        raise ValueError('Expected explicitly inspected robot domain 0')
    args.output.mkdir(parents=True,exist_ok=False)
    ctypes.CDLL(None).prctl(36,1,0,0,0)
    lock=open('/tmp/raicom_operator_session.lock','a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    env=dict(os.environ,ROS_LOG_DIR=str(args.output/'ros_logs'))
    observer=Path(__file__).with_name('observe_static.py')
    def observe(label,seconds):
        out=args.output/label
        with (args.output/(label+'.log')).open('w') as log:
            subprocess.run([sys.executable,str(observer),'--seconds',str(seconds),
                            '--output',str(out)],env=env,stdout=log,stderr=subprocess.STDOUT,
                           timeout=seconds+30,check=True)
        return json.loads((out/'summary.json').read_text())
    def zero(result):
        c=result['counts']
        if c.get('velocity',0)<100 or c.get('motors',0)<100 or c.get('nonzero_velocity',0) or c.get('nonzero_motors',0):
            raise RuntimeError('Fresh zero command verification failed')
        for topic in ('/controller/cmd_vel','/ros_robot_controller/set_motor','/scan','/odom'):
            if len(result['endpoints'][topic]['publishers'])!=1:
                raise RuntimeError('Publisher ownership failure: '+topic)
        for topic in ('/raicom/motion_enable','/raicom/teleop_cmd'):
            if result['endpoints'][topic]['publishers']:
                raise RuntimeError('Unexpected operator activity: '+topic)
    preflight=observe('preflight',10)
    zero(preflight)
    if preflight['endpoints']['/map']['publishers']:
        raise RuntimeError('Existing map owner; refusing a second SLAM')
    if hashlib.sha256(LIBRARY.read_bytes()).hexdigest()!=EXPECTED_LIBRARY:
        raise RuntimeError('Unexpected deployed library')
    child=None
    result={'scope':'REAL_SENSORS_STATIC_ONLY_NO_MAP_SAVE','motion_permission_created':False}
    try:
        with (args.output/'slam.log').open('w') as log:
            child=subprocess.Popen(['ros2','launch','raicom_control','home_slam.launch.py'],
              stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True)
            deadline=time.monotonic()+15
            slam_pid=None
            while time.monotonic()<deadline and child.poll() is None:
                for pid in owned_processes(child.pid):
                    cmd=Path(f'/proc/{pid}/cmdline').read_bytes()
                    if b'/lib/slam_toolbox/sync_slam_toolbox_node' in cmd:
                        slam_pid=pid
                if slam_pid:
                    break
                time.sleep(.2)
            if not slam_pid:
                raise RuntimeError('SLAM did not start')
            time.sleep(2)
            maps=Path(f'/proc/{slam_pid}/maps').read_text()
            (args.output/'loaded_maps.txt').write_text(maps)
            if str(LIBRARY) not in maps:
                raise RuntimeError('Expected patched library is not actually loaded')
            result.update(slam_pid=slam_pid,library=str(LIBRARY),library_sha256=EXPECTED_LIBRARY,
                          exe=os.readlink(f'/proc/{slam_pid}/exe'))
            params=subprocess.run(['ros2','param','dump','/slam_toolbox'],env=env,
                stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=12,check=True)
            (args.output/'runtime_parameters.yaml').write_bytes(params.stdout)
            sample=observe('observation',args.seconds)
            zero(sample)
            if child.poll() is not None or not sample['endpoints']['/map']['publishers']:
                raise RuntimeError('SLAM did not remain active')
            result['observation_complete']=True
    finally:
        if child is not None:
            result['cleanup_signals']=[]
            for sig,seconds in ((signal.SIGINT,10),(signal.SIGTERM,3),(signal.SIGKILL,3)):
                members=owned_processes(child.pid)
                if not members:
                    break
                os.killpg(child.pid,sig)
                result['cleanup_signals'].append(sig.name)
                limit=time.monotonic()+seconds
                while time.monotonic()<limit:
                    child.poll()
                    if not owned_processes(child.pid):
                        break
                    # Reap only our adopted, already-exited descendants.
                    for pid in owned_processes(child.pid):
                        if pid == child.pid:
                            continue
                        try:
                            os.waitpid(pid,os.WNOHANG)
                        except ChildProcessError:
                            pass
                    time.sleep(.1)
            result['launch_exit']=child.poll()
            result['remaining_owned_processes']=owned_processes(child.pid)
        (args.output/'trial_result.json').write_text(json.dumps(result,indent=2))
    stopped=observe('stopped',10)
    zero(stopped)
    if stopped['endpoints']['/map']['publishers']:
        raise RuntimeError('Map owner remained after trial')
    result['stopped_verified']=True
    (args.output/'trial_result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result),flush=True)

if __name__=='__main__':
    main()
