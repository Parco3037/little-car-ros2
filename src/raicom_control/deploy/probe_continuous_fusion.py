"""Bounded scan-keyframe + gyro fusion in an isolated namespace.

No production odometry/TF, commands, actions or services are written. The only
accepted output is PRIVATE /odom_valid, gated by original sensor timestamps.
An optional laser-dropout injection skips this probe's scan processing only;
it never stops the real sensor. Fresh unchecked EKF predictions remain visible
for two seconds after the gate closes, demonstrating why freshness is separate.
This is local relative odometry, NOT map alignment or navigation acceptance.
Default is stationary. --allow-manual only observes human-operated single-axis
commands through the existing guard; this probe cannot stop the real vehicle.
"""
import argparse
import copy
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import signal
import subprocess
import time

import numpy as np

from probe_sensor_only_ekf import parameters as fixed_parameters, planar_transform
from scan_keyframe_experiment import ScanTracker, TrackingConfig, sensor_pose_to_base
from scan_fusion_gate import FusionGate

PREFIX = '/raicom/continuous_fusion_shadow'
WORLD = 'raicom_continuous_fusion_shadow_odom'
NODE = 'continuous_fusion_supervisor'


def parameters():
    # Fresh dict; do not mutate the earlier fixed-reference experiment defaults.
    params = fixed_parameters()['/**']['ros__parameters']
    params.update(world_frame=WORLD, odom_frame=WORLD,
                  pose0=PREFIX+'/laser_pose', imu0=PREFIX+'/imu')
    return {'/**': {'ros__parameters': params}}


def command(path):
    return ['/opt/ros/humble/lib/robot_localization/ekf_node', '--ros-args',
            '-r', '__node:=continuous_sensor_ekf', '-r', '__ns:='+PREFIX,
            '--params-file', str(path),
            '-r', 'odometry/filtered:='+PREFIX+'/odom_unchecked',
            '-r', '/tf:='+PREFIX+'/tf_disabled',
            '-r', '/tf_static:='+PREFIX+'/tf_static',
            '-r', '/diagnostics:='+PREFIX+'/diagnostics',
            '-r', 'cmd_vel:='+PREFIX+'/unused_control',
            '-r', 'set_pose:='+PREFIX+'/unused_set_pose']


def stop_child(child):
    if child.poll() is None:
        child.send_signal(signal.SIGINT)
    forced = False
    try:
        child.wait(timeout=8)
    except subprocess.TimeoutExpired:
        forced = True
        child.kill()
        child.wait(timeout=3)
    return dict(returncode=child.returncode, reaped=child.poll() is not None, forced=forced)


def summary(rows, since):
    a = np.asarray([row for row in rows if row[0] >= since])
    if len(a) < 2:
        return dict(n=len(a))
    yaw = np.unwrap(a[:,3])
    return dict(n=len(a), duration_sec=float(a[-1,0]-a[0,0]),
                yaw_delta_deg=float(np.degrees(yaw[-1]-yaw[0])),
                yaw_span_deg=float(np.degrees(np.ptp(yaw))),
                max_xy_from_first_m=float(np.linalg.norm(a[:,1:3]-a[0,1:3],axis=1).max()),
                max_gap_sec=float(np.diff(a[:,0]).max()), max_age_sec=float(a[:,5].max()))


def check_observed_command(values, allow_manual=False, fusion_ready=False):
    """Validate a recorded command, not authorize or publish a motion command."""
    values=np.asarray(values,dtype=float)
    if values.shape!=(6,) or not np.isfinite(values).all():
        raise RuntimeError('Malformed/nonfinite command; human must stop')
    if not np.any(values):
        return False
    if not allow_manual:
        raise RuntimeError('Nonzero command in stationary test; human must stop')
    if not fusion_ready:
        raise RuntimeError('Human motion before fusion ready or after failure; human must stop')
    if any(values[i]!=0 for i in (2,3,4)) or abs(values[0])>.26 or abs(values[1])>.26 or abs(values[5])>.36:
        raise RuntimeError('Command outside manual-test limits; human must stop')
    if np.count_nonzero(values)!=1:
        raise RuntimeError('Only one manual axis at a time; human must stop')
    return True


def main():
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.time import Time
    import tf2_ros
    from tf2_msgs.msg import TFMessage
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from sensor_msgs.msg import Imu, LaserScan
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=120.)
    parser.add_argument('--inject-laser-dropout-at', type=float)
    parser.add_argument('--allow-manual', action='store_true',
                        help='Observe existing human-operated guarded controls; never drive the car')
    args = parser.parse_args()
    if not 60 <= args.seconds <= 300:
        parser.error('seconds must be 60..300')
    if args.inject_laser_dropout_at is not None and not 30 <= args.inject_laser_dropout_at <= args.seconds-5:
        parser.error('dropout injection must be >=30 and at least 5 seconds before end')
    if args.allow_manual and args.inject_laser_dropout_at is not None:
        parser.error('Do not inject faults during a physical movement test')
    out = args.output.open('x', encoding='utf-8')
    param_path = args.output.with_suffix('.params.yaml')
    with param_path.open('x', encoding='utf-8') as stream:
        json.dump(parameters(), stream, indent=2, allow_nan=False)
    config = TrackingConfig(); tracker = ScanTracker(config); gate = FusionGate()
    report = dict(utc_start=datetime.now(timezone.utc).isoformat(), production_modified=False,
                  application_publish_topics=[PREFIX+s for s in ('/imu','/laser_pose','/tf_static','/odom_valid','/health')],
                  scope='Human-operated isolated fusion recording' if args.allow_manual else 'Stationary isolated continuous sensor fusion',
                  allow_manual=args.allow_manual, motion_verified=False, navigation_verified=False,
                  physical_motion_truth=None, config=asdict(config), parameters=parameters(),
                  command=command(param_path), freshness_limits=gate.LIMITS,
                  injection_at=args.inject_laser_dropout_at, frames=[], series={}, speeds=[],
                  owners=[], gate_events=[], input_examples={}, errors=[], output_decisions=[],
                  scan_receipts=[], scan_processing_policy='Each new stamp, latest-only, no timer decimation',
                  laser_published=0, imu_forwarded=0, valid_published=0, private_tf_messages=0)
    rclpy.init()
    node = rclpy.create_node(NODE, enable_rosout=False, start_parameter_services=False)
    buffer = tf2_ros.Buffer(); listener = tf2_ros.TransformListener(buffer, node)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    static_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    # All application publishers are explicitly confined to this namespace.
    imu_pub = node.create_publisher(Imu, PREFIX+'/imu', 5)
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, PREFIX+'/laser_pose', 5)
    tf_pub = node.create_publisher(TFMessage, PREFIX+'/tf_static', static_qos)
    valid_pub = node.create_publisher(Odometry, PREFIX+'/odom_valid', 1)
    health_pub = node.create_publisher(String, PREFIX+'/health', 1)
    state = dict(scan=None, imu=None, speed_at=None, forward=False, imu_frame=None,
                 last_nonzero_at=None, latest_speed=[0.]*6)
    start = time.monotonic(); ready = None; child = None; log = None; last_event = None

    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

    def ros_now():
        return node.get_clock().now().nanoseconds*1e-9

    def elapsed():
        return time.monotonic()-start

    def speed(msg):
        state['speed_at'] = time.monotonic()
        values = [msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z]
        report['speeds'].append([elapsed(), *values])
        state['latest_speed']=values
        if check_observed_command(values,args.allow_manual,gate.established and gate.failure is None):
            state['last_nonzero_at']=state['speed_at']

    def receive_scan(msg):
        state['scan']=msg
        report['scan_receipts'].append(dict(t=elapsed(),stamp=stamp(msg),age=ros_now()-stamp(msg)))

    def record(msg, topic):
        is_imu = isinstance(msg, Imu)
        q = msg.orientation if is_imu else msg.pose.pose.orientation
        vals = [q.x,q.y,q.z,q.w]
        if not np.isfinite(vals).all() or abs(sum(v*v for v in vals)-1) > .001:
            raise RuntimeError('Invalid orientation '+topic)
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        x=y=0.
        if not is_imu:
            p=msg.pose.pose.position; v=msg.twist.twist
            x,y=p.x,p.y
            if not np.isfinite([x,y,p.z,v.linear.x,v.linear.y,v.linear.z,
                                v.angular.x,v.angular.y,v.angular.z,*msg.pose.covariance,*msg.twist.covariance]).all():
                raise RuntimeError('Nonfinite odometry '+topic)
        source_stamp=stamp(msg); age=ros_now()-source_stamp
        row=[elapsed(),x,y,yaw,source_stamp,age]
        report['series'].setdefault(topic,[]).append(row)
        if topic not in report['input_examples']:
            report['input_examples'][topic] = dict(frame=msg.header.frame_id)
            if is_imu:
                report['input_examples'][topic].update(angular_velocity_covariance=list(msg.angular_velocity_covariance))
            else:
                report['input_examples'][topic].update(child=msg.child_frame_id, pose_covariance=list(msg.pose.covariance))
        return row

    def imu(msg):
        state['imu']=msg
        record(msg, '/imu')
        if not state['forward']:
            return
        if msg.header.frame_id != state['imu_frame']:
            raise RuntimeError('IMU frame changed')
        if not np.isfinite([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z,
                            *msg.angular_velocity_covariance]).all() or msg.angular_velocity_covariance[0]<0:
            raise RuntimeError('Invalid angular velocity/covariance')
        if not 0 <= ros_now()-stamp(msg) <= .3:
            raise RuntimeError('Stale or future IMU')
        # Continue raw private prediction during intentional dropout only to
        # demonstrate that the accepted stream stays closed. Never fuse yaw.
        if gate.observe('imu',stamp(msg)) or (report.get('injection_started') and gate.failure):
            imu_pub.publish(msg); report['imu_forwarded']+=1

    def decision():
        nonlocal last_event
        now=ros_now(); tracker.tick(now)
        status=gate.evaluate(now,tracker.status)
        identity=(status['state'],status['reason'])
        if identity != last_event:
            report['gate_events'].append(dict(t=elapsed(),**status))
            last_event=identity
            print('GATE '+json.dumps(dict(t=round(elapsed(),3),**status)),flush=True)
        return status

    def raw_odom(msg):
        row=record(msg, PREFIX+'/odom_unchecked')
        if msg.header.frame_id != WORLD or msg.child_frame_id != 'base_footprint':
            raise RuntimeError('Private EKF frame mismatch')
        new=gate.observe('ekf',stamp(msg))
        status=decision()
        accepted=new and status['valid']
        report['output_decisions'].append(dict(t=row[0],stamp=stamp(msg),accepted=accepted,**status))
        if accepted:
            # No replacement stamp and no relabeling as map/production odom.
            valid_pub.publish(msg); report['valid_published']+=1
            report['series'].setdefault(PREFIX+'/odom_valid',[]).append(row)

    def unexpected_tf(msg):
        report['private_tf_messages']+=1
        raise RuntimeError('Private EKF emitted TF despite publish_tf=False')

    subs=[node.create_subscription(Twist,'/controller/cmd_vel',speed,qos),
          node.create_subscription(LaserScan,'/scan_stable',receive_scan,qos),
          node.create_subscription(Imu,'/imu',imu,qos),
          node.create_subscription(Odometry,PREFIX+'/odom_unchecked',raw_odom,qos),
          node.create_subscription(TFMessage,PREFIX+'/tf_disabled',unexpected_tf,qos),
          node.create_subscription(Odometry,PREFIX+'/odom_valid',
                                   lambda m:record(m,PREFIX+'/odom_valid_received'),qos)]
    for topic in ('/odom','/odom_raw'):
        subs.append(node.create_subscription(Odometry,topic,lambda m,t=topic:record(m,t),qos))

    def safety():
        if state['speed_at'] is None or time.monotonic()-state['speed_at']>.3:
            raise RuntimeError('Controller command observation stale; human must stop')
        if child is not None and child.poll() is not None:
            raise RuntimeError('Private EKF exited early')

    def graph_check():
        topics=['/controller/cmd_vel','/odom','/odom_raw','/imu','/scan_stable',
                PREFIX+'/odom_unchecked',PREFIX+'/odom_valid',PREFIX+'/laser_pose',PREFIX+'/imu',PREFIX+'/tf_disabled']
        owners={t:[i.node_namespace.rstrip('/')+'/'+i.node_name for i in node.get_publishers_info_by_topic(t)] for t in topics}
        report['owners'].append(dict(t=elapsed(),topics=owners))
        if owners['/controller/cmd_vel'] != ['/cmd_vel_guard'] or any(len(owners[t])!=1 for t in topics[:5]):
            raise RuntimeError('Production unique publisher gate failed')
        for suffix in ('/odom_valid','/laser_pose','/imu'):
            if owners[PREFIX+suffix]!=['/'+NODE]:
                raise RuntimeError('Private supervisor output has another owner: '+suffix)
        expected=[] if child is None else [PREFIX+'/continuous_sensor_ekf']
        if owners[PREFIX+'/odom_unchecked'] != expected:
            raise RuntimeError('Private EKF output ownership mismatch')
        # 3.5.3 constructs its broadcaster even with publish_tf=False. Ownership
        # is not publication: separately subscribe and require zero messages.
        if owners[PREFIX+'/tf_disabled'] != expected:
            raise RuntimeError('Unexpected private TF endpoint owner')

    def scan_data(msg):
        return dict(stamp=stamp(msg),frame_id=msg.header.frame_id,angle_min=msg.angle_min,
                    angle_increment=msg.angle_increment,range_min=msg.range_min,range_max=msg.range_max,
                    ranges=[float(v) if math.isfinite(v) else None for v in msg.ranges])

    try:
        until=time.monotonic()+8
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.02)
        safety(); graph_check()
        first=state['scan']
        if first is None or len(first.ranges)!=254 or not 0<=ros_now()-stamp(first)<=.3:
            raise RuntimeError('Missing fresh fixed 254-bin scan')
        if state['imu'] is None or not 0<=ros_now()-stamp(state['imu'])<=.3:
            raise RuntimeError('Missing fresh IMU')
        transform=buffer.lookup_transform('base_footprint',first.header.frame_id,Time()).transform
        q,t=transform.rotation,transform.translation
        mount=[q.x,q.y,q.z,q.w,t.x,t.y,t.z]
        mounting,offset=planar_transform(mount[:4],mount[4:])
        report['mount']=dict(source=first.header.frame_id,target='base_footprint',values=mount)
        state['imu_frame']=state['imu'].header.frame_id
        imu_tf=copy.deepcopy(buffer.lookup_transform('base_footprint',state['imu_frame'],Time()))
        tf_pub.publish(TFMessage(transforms=[imu_tf]))
        log=args.output.with_suffix('.child.log').open('x')
        child=subprocess.Popen(command(param_path),stdout=log,stderr=subprocess.STDOUT)
        until=time.monotonic()+5
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.02); safety()
        graph_check()
        if node.count_subscribers(PREFIX+'/imu')!=1 or node.count_subscribers(PREFIX+'/laser_pose')!=1:
            raise RuntimeError('Private input discovery mismatch')
        ready=time.monotonic(); report['ready_t']=elapsed(); state['forward']=True
        next_graph=ready+1; next_health=ready; next_status=ready
        last_stamp=None; loss_at=None
        print('PRIVATE_FUSION_STARTED: wait for VALID and operator instruction; do not move yet',flush=True)
        while time.monotonic()-ready<args.seconds:
            rclpy.spin_once(node,timeout_sec=.005); safety()
            mono=time.monotonic(); active=mono-ready
            if mono>=next_graph:
                graph_check(); next_graph=mono+1
                current=buffer.lookup_transform('base_footprint',first.header.frame_id,Time()).transform
                a,b=current.rotation,current.translation
                if not np.allclose([a.x,a.y,a.z,a.w,b.x,b.y,b.z],mount,atol=1e-6,rtol=0):
                    raise RuntimeError('Lidar mounting changed')
            injecting=args.inject_laser_dropout_at is not None and active>=args.inject_laser_dropout_at
            if injecting and not report.get('injection_started'):
                if not gate.established or gate.failure:
                    raise RuntimeError('Injection requires previously healthy fusion')
                report['injection_started']=dict(t=elapsed(),ros_time=ros_now(),kind='Skip private scan processing only')
            msg=state['scan']; source_stamp=stamp(msg)
            # The sensor already runs near 10 Hz. Additional 150 ms decimation
            # delayed the next usable frame after one rejected fit, exhausting
            # the unchanged 450 ms validity budget. Process each new source
            # stamp once; depth-one QoS bounds backlog. Never renew a bad fit.
            if not injecting and gate.failure is None and source_stamp!=last_stamp:
                last_stamp=source_stamp
                scan=scan_data(msg); now=ros_now(); before=time.monotonic()
                result=tracker.update(scan,now)
                item=dict(t=elapsed(),now=now,scan=scan,result=result,processing_ms=(time.monotonic()-before)*1000)
                if result['valid']:
                    pose=sensor_pose_to_base(result['pose'],mounting,offset)
                    item['base_pose']=pose.tolist()
                    if gate.observe('laser',result['stamp']):
                        measurement=PoseWithCovarianceStamped()
                        measurement.header.frame_id=WORLD; measurement.header.stamp=copy.deepcopy(msg.header.stamp)
                        measurement.pose.pose.position.x=float(pose[0]); measurement.pose.pose.position.y=float(pose[1])
                        measurement.pose.pose.orientation.z=math.sin(pose[2]/2)
                        measurement.pose.pose.orientation.w=math.cos(pose[2]/2)
                        # Diagnostic uncertainty only, not calibrated accuracy.
                        for index,value in zip((0,7,14,21,28,35),(.02**2,.02**2,1e6,1e6,1e6,math.radians(1)**2)):
                            measurement.pose.covariance[index]=value
                        pose_pub.publish(measurement); report['laser_published']+=1
                report['frames'].append(item)
            status=decision()
            if mono>=next_health:
                health_pub.publish(String(data=json.dumps(dict(t=elapsed(),**status),allow_nan=False)))
                next_health=mono+.1
            if status['state']=='LOST':
                if not injecting:
                    raise RuntimeError('Fusion validity lost: '+status['reason'])
                loss_at=loss_at or mono
                if mono-loss_at>=2:
                    break
            if active>5 and not gate.established:
                raise RuntimeError('No valid private fusion after startup')
            if mono>=next_status:
                print(json.dumps(dict(t=round(elapsed(),1),gate=status['state'],frames=len(report['frames']),
                    laser_published=report['laser_published'],valid_published=report['valid_published'],
                    nonzero_commands=sum(any(row[1:]) for row in report['speeds']),
                    base_pose=report['frames'][-1].get('base_pose') if report['frames'] else None)),flush=True)
                next_status=mono+10
        if args.inject_laser_dropout_at is not None:
            if loss_at is None:
                raise RuntimeError('Expected dropout did not close gate')
            lost_event=next(e for e in report['gate_events'] if e['state']=='LOST')
            after=[d for d in report['output_decisions'] if d['t']>=lost_event['t']]
            report['dropout_result']=dict(loss_t=lost_event['t'],reason=lost_event['reason'],
                raw_outputs_after_loss=len(after),accepted_after_loss=sum(d['accepted'] for d in after),
                fresh_raw_after_loss=sum(0<=row[5]<=.3 for row in report['series'].get(PREFIX+'/odom_unchecked',[]) if row[0]>=lost_event['t']))
            if len(after)<10 or any(d['accepted'] for d in after):
                raise RuntimeError('Dropout evidence incomplete or accepted output leaked')
        else:
            rows=report['series'].get(PREFIX+'/odom_valid',[])
            if len(rows)<args.seconds*10:
                raise RuntimeError('Insufficient valid output coverage')
        if args.allow_manual:
            last=state['last_nonzero_at']
            report['manual_observation']=dict(nonzero_seen=last is not None,
                last_nonzero_t=None if last is None else last-start,
                final_zero_duration=None if last is None else time.monotonic()-last,
                stopped_command_at_end=not any(state['latest_speed']))
            if any(state['latest_speed']) or (last is not None and time.monotonic()-last<5):
                raise RuntimeError('No five-second zero-command tail; human must stop')
        key='trajectory_after_warmup' if args.allow_manual else 'held_after_warmup'
        report[key]={topic:summary(rows,report['ready_t']+10) for topic,rows in report['series'].items()}
        report['completed']=True
    except BaseException as error:
        report['completed']=False; report['errors'].append(type(error).__name__+': '+str(error))
        print('OBSERVATION_STOPPED: '+str(error)+'; human must stop; this probe cannot command a stop',flush=True)
        raise
    finally:
        state['forward']=False
        if child is not None:
            report['child']=stop_child(child)
        report['utc_end']=datetime.now(timezone.utc).isoformat()
        json.dump(report,out,indent=2,allow_nan=False); out.close()
        if log is not None:
            log.close()
        node.destroy_node(); rclpy.shutdown()


if __name__=='__main__':
    main()
