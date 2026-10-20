"""Private laser-reference + gyro EKF experiment, not a production navigator.

No command-derived odometry is fused. No production sensor/TF/command output.
Fixed-reference range is bounded: this tests static fusion only, not long routes.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import signal
import subprocess
import time

import numpy as np

from scan_anchor_experiment import reference, scan_points, match

PREFIX = '/raicom/ekf_sensor_shadow'
WORLD = 'raicom_ekf_sensor_shadow_odom'


def parameters():
    return {'/**': {'ros__parameters': dict(
        use_sim_time=False, frequency=30.0, sensor_timeout=0.3,
        two_d_mode=True, publish_tf=False, publish_acceleration=False,
        print_diagnostics=False, debug=False, use_control=False,
        world_frame=WORLD, odom_frame=WORLD, base_link_frame='base_footprint',
        transform_timeout=0.05,
        pose0=PREFIX+'/laser_pose', pose0_queue_size=5,
        pose0_config=[True,True,False,False,False,True]+[False]*9,
        pose0_differential=False, pose0_relative=False,
        imu0=PREFIX+'/imu', imu0_queue_size=10,
        imu0_config=[False]*11+[True]+[False]*3,
        imu0_differential=False, imu0_relative=False,
        imu0_remove_gravitational_acceleration=False)}}


def command(parameter_path):
    return ['/opt/ros/humble/lib/robot_localization/ekf_node', '--ros-args',
        '-r','__node:=sensor_only_ekf','-r','__ns:='+PREFIX,
        '--params-file',str(parameter_path),
        '-r','odometry/filtered:='+PREFIX+'/odom',
        '-r','/tf:='+PREFIX+'/tf_disabled',
        '-r','/tf_static:='+PREFIX+'/tf_static',
        '-r','/diagnostics:='+PREFIX+'/diagnostics',
        '-r','cmd_vel:='+PREFIX+'/unused_control',
        '-r','set_pose:='+PREFIX+'/unused_set_pose']


def planar_transform(xyzw, translation):
    """Allow an upright or inverted horizontal lidar, never ignore its mounting."""
    x,y,z,w=np.asarray(xyzw,dtype=float)
    t=np.asarray(translation,dtype=float)
    if not np.isfinite([x,y,z,w,*t]).all() or abs(x*x+y*y+z*z+w*w-1)>1e-5:
        raise ValueError('Invalid lidar mounting transform')
    matrix=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    # Projection would distort scan geometry for a tilted sensor: reject instead.
    if abs(matrix[2,2])<.9999:
        raise ValueError('Lidar plane is not horizontal enough for this experiment')
    return matrix[:2,:2],t[:2]


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.time import Time
    import tf2_ros
    from tf2_msgs.msg import TFMessage
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from sensor_msgs.msg import Imu, LaserScan
    from nav_msgs.msg import Odometry

    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=180)
    args=parser.parse_args()
    if not 120<=args.seconds<=300:
        parser.error('seconds must be 120..300')
    out=args.output.open('x')
    parameter_path=args.output.with_suffix('.params.yaml')
    with parameter_path.open('x') as stream:
        json.dump(parameters(),stream,indent=2,allow_nan=False)
    report=dict(production_modified=False,motion_verified=False,
                scope='Fixed-reference static fusion experiment; not navigation acceptance',
                command=command(parameter_path),parameters=parameters(),
                velocity_n=0,velocity_nonzero=0,errors=[],reference_scans=[],fits=[],series={},input_examples={})
    rclpy.init(); node=rclpy.create_node('sensor_only_ekf_supervisor')
    buffer=tf2_ros.Buffer(); listener=tf2_ros.TransformListener(buffer,node)
    state=dict(speed_at=None,scan=None,imu=None,forward=False,imu_frame=None)
    start=time.monotonic(); child=None; log=None
    imu_pub=node.create_publisher(Imu,PREFIX+'/imu',10)
    pose_pub=node.create_publisher(PoseWithCovarianceStamped,PREFIX+'/laser_pose',10)
    static_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    tf_pub=node.create_publisher(TFMessage,PREFIX+'/tf_static',static_qos)

    def stamp_age(msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        age=node.get_clock().now().nanoseconds*1e-9-stamp
        if not 0<=age<=.3: raise ValueError('Stale or future sensor stamp')
        return stamp,age

    def speed(msg):
        state['speed_at']=time.monotonic(); report['velocity_n']+=1
        report['velocity_nonzero']+=int(any(v!=0 for v in (msg.linear.x,msg.linear.y,msg.linear.z,
                                                          msg.angular.x,msg.angular.y,msg.angular.z)))

    def record(msg,topic):
        q=msg.orientation if isinstance(msg,Imu) else msg.pose.pose.orientation
        if not np.isfinite([q.x,q.y,q.z,q.w]).all() or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>1e-3:
            raise ValueError('Invalid orientation '+topic)
        stamp,age=stamp_age(msg)
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        x=y=0.
        if isinstance(msg,Odometry): x,y=msg.pose.pose.position.x,msg.pose.pose.position.y
        if not np.isfinite([x,y]).all(): raise ValueError('Invalid position '+topic)
        report['series'].setdefault(topic,[]).append([time.monotonic()-start,x,y,yaw,stamp,age])
        if topic not in report['input_examples']:
            item=dict(frame=msg.header.frame_id)
            if isinstance(msg,Imu):
                item.update(orientation_covariance=list(msg.orientation_covariance),
                    angular_velocity_covariance=list(msg.angular_velocity_covariance))
            else:
                item.update(child_frame=msg.child_frame_id,pose_covariance=list(msg.pose.covariance),
                    twist_covariance=list(msg.twist.covariance))
            report['input_examples'][topic]=item

    def imu(msg):
        stamp_age(msg)
        if not np.isfinite([msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]).all():
            raise ValueError('Nonfinite gyro')
        record(msg,'/imu'); state['imu']=msg
        if state['forward']:
            if msg.header.frame_id!=state['imu_frame']: raise ValueError('IMU frame changed')
            imu_pub.publish(msg)

    subs=[node.create_subscription(Twist,'/controller/cmd_vel',speed,qos_profile_sensor_data),
          node.create_subscription(LaserScan,'/scan_stable',lambda m:state.update(scan=m),qos_profile_sensor_data),
          node.create_subscription(Imu,'/imu',imu,qos_profile_sensor_data)]
    for topic in ('/odom','/odom_raw',PREFIX+'/odom'):
        subs.append(node.create_subscription(Odometry,topic,lambda m,t=topic:record(m,t),qos_profile_sensor_data))

    def safety():
        owners=node.get_publishers_info_by_topic('/controller/cmd_vel')
        if state['speed_at'] is None or time.monotonic()-state['speed_at']>.3 or report['velocity_nonzero']:
            raise RuntimeError('Zero-command observation gate failed')
        if len(owners)!=1 or owners[0].node_name!='cmd_vel_guard': raise RuntimeError('Unique guard gate failed')
        if child is not None and child.poll() is not None: raise RuntimeError('EKF exited early')

    def scan_data():
        msg=state['scan']
        if msg is None or len(msg.ranges)!=254: raise ValueError('Missing fixed 254-bin scan')
        stamp,age=stamp_age(msg)
        return dict(stamp=stamp,age=age,t=time.monotonic()-start,frame_id=msg.header.frame_id,
            angle_min=msg.angle_min,angle_increment=msg.angle_increment,
            range_min=msg.range_min,range_max=msg.range_max,
            ranges=[float(v) if math.isfinite(v) else None for v in msg.ranges])

    try:
        until=time.monotonic()+10
        while time.monotonic()<until: rclpy.spin_once(node,timeout_sec=.02)
        safety(); first=scan_data()
        if state['imu'] is None: raise ValueError('No fresh IMU')
        if node.get_publishers_info_by_topic(PREFIX+'/odom'): raise ValueError('Private output already owned')
        extrinsic=buffer.lookup_transform('base_footprint',first['frame_id'],Time()).transform
        q,t=extrinsic.rotation,extrinsic.translation
        planar,translation=planar_transform([q.x,q.y,q.z,q.w],[t.x,t.y,t.z])
        report['lidar_to_base']=dict(xyzw=[q.x,q.y,q.z,q.w],translation=[t.x,t.y,t.z])
        state['imu_frame']=state['imu'].header.frame_id
        imu_tf=copy.deepcopy(buffer.lookup_transform('base_footprint',state['imu_frame'],Time()))
        tf_pub.publish(TFMessage(transforms=[imu_tf]))
        log=args.output.with_suffix('.child.log').open('x')
        child=subprocess.Popen(command(parameter_path),stdout=log,stderr=subprocess.STDOUT)
        until=time.monotonic()+5
        while time.monotonic()<until: rclpy.spin_once(node,timeout_sec=.02); safety()
        if node.count_subscribers(PREFIX+'/imu')!=1 or node.count_subscribers(PREFIX+'/laser_pose')!=1:
            raise RuntimeError('Private input subscriber discovery failed')
        state['forward']=True; start=time.monotonic(); report['series'].clear()
        next_reference=0.; next_fit=10.; ref=None; last_stamp=None; last_pose_at=None
        while time.monotonic()-start<args.seconds:
            rclpy.spin_once(node,timeout_sec=.02); safety(); elapsed=time.monotonic()-start
            if elapsed<10 and elapsed>=next_reference:
                report['reference_scans'].append(scan_data()); next_reference=elapsed+1
            if elapsed>=10 and ref is None:
                points,normals=reference(report['reference_scans'])
                ref=(points@planar.T+translation,normals@planar.T)
                ref=(ref[0],ref[1]/np.linalg.norm(ref[1],axis=1)[:,None])
            if ref is not None and elapsed>=next_fit:
                next_fit=elapsed+.2; scan=scan_data()
                if any(scan[k]!=first[k] for k in ('frame_id','angle_min','angle_increment','range_min','range_max')):
                    raise ValueError('Scan geometry changed')
                if scan['stamp']==last_stamp: continue
                last_stamp=scan['stamp']; points,valid=scan_points(scan)
                before=time.monotonic()
                try: result=match(points[valid]@planar.T+translation,*ref)
                except ValueError as error: result=dict(valid=False,reason=str(error))
                report['fits'].append(dict(t=elapsed,stamp=scan['stamp'],processing_ms=(time.monotonic()-before)*1000,**result))
                if result['valid']:
                    msg=PoseWithCovarianceStamped(); msg.header.frame_id=WORLD
                    msg.header.stamp=state['scan'].header.stamp
                    msg.pose.pose.position.x=result['x_m']; msg.pose.pose.position.y=result['y_m']
                    msg.pose.pose.orientation.z=math.sin(result['yaw_rad']/2)
                    msg.pose.pose.orientation.w=math.cos(result['yaw_rad']/2)
                    # Diagnostic uncertainty, NOT a sensor calibration or precision promise.
                    for i,value in zip((0,7,14,21,28,35),(.02**2,.02**2,1e6,1e6,1e6,math.radians(1)**2)):
                        msg.pose.covariance[i]=value
                    pose_pub.publish(msg); last_pose_at=time.monotonic()
            if elapsed>15 and (last_pose_at is None or time.monotonic()-last_pose_at>1):
                raise RuntimeError('No recent accepted independent laser pose')
            if elapsed>20 and not report['series'].get(PREFIX+'/odom'): raise RuntimeError('No private EKF output')
        report['held_after_30s']={}
        for topic,rows in report['series'].items():
            a=np.asarray([row for row in rows if row[0]>=30])
            if len(a)<(args.seconds-30)*10: raise RuntimeError('Insufficient output coverage '+topic)
            yaw=np.unwrap(a[:,3])
            report['held_after_30s'][topic]=dict(n=len(a),duration_sec=float(a[-1,0]-a[0,0]),
                yaw_delta_deg=float(np.degrees(yaw[-1]-yaw[0])),yaw_span_deg=float(np.degrees(np.ptp(yaw))),
                max_xy_from_first_m=float(np.linalg.norm(a[:,1:3]-a[0,1:3],axis=1).max()),
                max_gap_sec=float(np.diff(a[:,0]).max()),max_age_sec=float(a[:,5].max()))
        report['publishers']={topic:[i.node_namespace.rstrip('/')+'/'+i.node_name for i in node.get_publishers_info_by_topic(topic)]
            for topic in ('/odom','/odom_rf2o','/imu','/controller/cmd_vel',PREFIX+'/odom',PREFIX+'/laser_pose',PREFIX+'/imu')}
    except Exception as error:
        report['errors'].append(str(error)); raise
    finally:
        state['forward']=False
        if child is not None:
            if child.poll() is None: child.send_signal(signal.SIGINT)
            try: child.wait(timeout=8)
            except subprocess.TimeoutExpired: child.kill(); child.wait(timeout=3)
            report['child_returncode']=child.returncode; report['child_reaped']=child.poll() is not None
        json.dump(report,out,indent=2,allow_nan=False); out.close()
        if log is not None: log.close()
        node.destroy_node(); rclpy.shutdown()


if __name__=='__main__': main()
