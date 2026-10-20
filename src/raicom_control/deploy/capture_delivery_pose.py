#!/usr/bin/env python3
"""Record an operator-positioned arm through two physical ROS servo reads.

Never publishes a servo, gripper or chassis command. Capture the requested
transport/placement phase only after the operator has positioned it onsite.
"""
import argparse
import inspect
import json
from pathlib import Path
import time
from raicom_control.delivery_arm import SERVO_IDS


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--name',required=True);p.add_argument('--output',required=True,type=Path)
    a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from ros_robot_controller_msgs.msg import GetBusServoCmd
    from ros_robot_controller_msgs.srv import GetBusServoState
    from ros_robot_controller.ros_robot_controller_sdk import Board
    if 'RAICOM_BOUNDED_SERVO_READ_V1' not in inspect.getsource(Board.bus_servo_read_and_unpack):
        raise RuntimeError('Bounded physical read patch is not loaded')
    rclpy.init();node=rclpy.create_node('capture_delivery_pose');zero={'since':None,'at':None}
    def velocity(msg):
        now=time.monotonic();zero['at']=now
        if any(abs(v)>1e-7 for v in [msg.linear.x,msg.linear.y,msg.angular.z]):zero['since']=None
        elif zero['since'] is None:zero['since']=now
    node.create_subscription(Twist,'/controller/cmd_vel',velocity,qos_profile_sensor_data)
    client=node.create_client(GetBusServoState,'/ros_robot_controller/bus_servo/get_state')
    def check():
        now=time.monotonic()
        owners=node.get_publishers_info_by_topic('/controller/cmd_vel')
        if (zero['since'] is None or now-zero['since']<.5 or zero['at'] is None or now-zero['at']>.5
                or len(owners)!=1 or owners[0].node_name!='cmd_vel_guard'):
            raise RuntimeError('Fresh unique-guard zero hold required for physical capture')
    try:
        end=time.monotonic()+2.
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.03)
        check()
        if not client.service_is_ready():raise RuntimeError('Physical read service unavailable')
        reads=[]
        for repeat in range(2):
            actual={}
            for sid in SERVO_IDS:
                check();future=client.call_async(GetBusServoState.Request(cmd=[GetBusServoCmd(id=sid,get_position=1)]))
                end=time.monotonic()+.8
                while not future.done() and time.monotonic()<end:
                    rclpy.spin_once(node,timeout_sec=.03);check()
                if not future.done():client.remove_pending_request(future);raise RuntimeError('Servo read timeout')
                reply=future.result()
                if reply is None or not reply.success or len(reply.state)!=1 or len(reply.state[0].position)!=1:
                    raise RuntimeError('Missing physical read, cached joint states are not accepted')
                value=reply.state[0].position[0]
                if not 0<=value<=1000:raise RuntimeError('Physical position outside range')
                actual[str(sid)]=value
            reads.append(actual)
        if any(abs(reads[0][k]-reads[1][k])>3 for k in reads[0]):raise RuntimeError('Arm moved during capture')
        result={'name':a.name,'positions':reads[-1],'duration_s':4.0,'tolerance_pulse':10,
                'physical_readback_verified':True,'capture_evidence':str(a.output.resolve()),
                'physical_reads':reads,'boot_id':Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                'recorded_at_unix':time.time(),'command_sent':False}
        with a.output.open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
        print(json.dumps(result))
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
