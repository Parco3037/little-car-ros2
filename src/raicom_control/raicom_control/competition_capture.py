"""Read-only measurement of a manually positioned arm. Never sends motion."""
import argparse
import inspect
import json
import math
from pathlib import Path
import time


def capture(name, output):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from ros_robot_controller_msgs.msg import GetBusServoCmd
    from ros_robot_controller_msgs.srv import GetBusServoState
    from ros_robot_controller.ros_robot_controller_sdk import Board
    if 'RAICOM_BOUNDED_SERVO_READ_V1' not in inspect.getsource(Board.bus_servo_read_and_unpack):
        raise RuntimeError('缺少已验证的舵机物理读回超时保护')
    output=Path(output)
    if output.exists(): raise FileExistsError(output)
    rclpy.init(); node=rclpy.create_node('raicom_competition_pose_capture')
    state=dict(velocity_at=0.,zero_since=None,odom_at=0.,odom_zero=False)
    def velocity(msg):
        now=time.monotonic();state['velocity_at']=now
        zero=all(math.isfinite(v) and abs(v)<=1e-7 for v in (msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z))
        state['zero_since']=(state['zero_since'] or now) if zero else None
    def odom(msg):
        v=msg.twist.twist;state['odom_at']=time.monotonic()
        state['odom_zero']=all(math.isfinite(x) for x in (v.linear.x,v.linear.y,v.angular.z)) and math.hypot(v.linear.x,v.linear.y)<=.01 and abs(v.angular.z)<=.02
    node.create_subscription(Twist,'/controller/cmd_vel',velocity,qos_profile_sensor_data)
    node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
    client=node.create_client(GetBusServoState,'/ros_robot_controller/bus_servo/get_state')
    def check():
        now=time.monotonic();owners=node.get_publishers_info_by_topic('/controller/cmd_vel')
        if (state['zero_since'] is None or now-state['zero_since']<.5 or now-state['velocity_at']>.5
                or now-state['odom_at']>.5 or not state['odom_zero'] or len(owners)!=1 or owners[0].node_name!='cmd_vel_guard'):
            raise RuntimeError('采集期间底盘须持续停稳，guard与里程计须有新鲜数据')
    try:
        end=time.monotonic()+2
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.03)
        check()
        if not client.service_is_ready():raise RuntimeError('舵机物理读回接口未就绪')
        reads=[]
        for _ in range(2):
            positions={}
            for sid in (1,2,3,4,5,10):
                check();future=client.call_async(GetBusServoState.Request(cmd=[GetBusServoCmd(id=sid,get_position=1)]))
                end=time.monotonic()+.8
                while not future.done() and time.monotonic()<end:
                    rclpy.spin_once(node,timeout_sec=.02);check()
                if not future.done():client.remove_pending_request(future);raise RuntimeError('舵机物理读回超时')
                reply=future.result()
                if reply is None or not reply.success or len(reply.state)!=1 or len(reply.state[0].position)!=1:
                    raise RuntimeError(f'舵机{sid}没有有效物理读回')
                pulse=reply.state[0].position[0]
                if type(pulse) is not int or not 0<=pulse<=1000:raise RuntimeError('舵机读回值无效')
                positions[str(sid)]=pulse
            reads.append(positions)
        if any(abs(reads[0][k]-reads[1][k])>3 for k in reads[0]):raise RuntimeError('采集期间姿态仍在变化')
        value=dict(name=name,positions=reads[-1],physical_reads=reads,command_sent=False,physical_readback_verified=True,
            boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),recorded_at_unix=time.time(),
            duration_s=4.,tolerance_pulse=10,capture_evidence=str(output.resolve()),trajectory_verified=False)
        output.parent.mkdir(parents=True,exist_ok=True)
        with output.open('x',encoding='utf-8') as stream:json.dump(value,stream,ensure_ascii=False,indent=2)
        print(json.dumps(value,ensure_ascii=False))
    finally:node.destroy_node();rclpy.shutdown()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name',required=True);parser.add_argument('--output',required=True)
    args=parser.parse_args();capture(args.name,args.output)


if __name__=='__main__':main()
