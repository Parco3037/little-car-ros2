#!/usr/bin/env python3
"""Exercise the installed launch against synthetic RGB-D and virtual servos.

Always runs in an isolated, localhost-only DDS domain. No hardware or camera
driver is launched. This verifies software transitions, not a physical grip.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

# Refuse the production domain before importing or creating any ROS entity.
assert os.environ.get('ROS_DOMAIN_ID') == '126'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
os.environ['MACHINE_TYPE'] = 'LanderPi_Mecanum'

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from kinematics.forward_kinematics import ForwardKinematics
from kinematics.transform import pulse2angle
from rclpy.node import Node
from ros_robot_controller_msgs.msg import BusServoState
from ros_robot_controller_msgs.srv import GetBusServoState
from sensor_msgs.msg import CameraInfo, Image
from servo_controller_msgs.msg import ServoState, ServoStateList, ServosPosition
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rgbd_grasp.target_to_ik_node import pose_matrix


class Fixture(Node):
    def __init__(self, empty=False):
        super().__init__('rgbd_isolated_fixture')
        self.empty = empty
        self.positions = {1: 450., 2: 700., 3: 150., 4: 320., 5: 875., 10: 390.}
        self.motions = {}
        self.commands = []
        self.stages = []
        self.expose_cube = False
        self.create_service(Trigger, '/controller_manager/init_finish', self.ready)
        self.create_service(GetBusServoState, '/ros_robot_controller/bus_servo/get_state', self.read)
        self.create_subscription(ServosPosition, '/servo_controller', self.command, 10)
        self.create_subscription(String, '/grasp_once_node/status', self.status, 20)
        self.states_pub = self.create_publisher(ServoStateList, '/controller_manager/servo_states', 10)
        self.rgb_pub = self.create_publisher(Image, '/aurora/rgb/image_raw', 1)
        self.depth_pub = self.create_publisher(Image, '/aurora/depth/image_raw', 1)
        self.info_pub = self.create_publisher(CameraInfo, '/aurora/rgb/camera_info', 1)
        self.stop_client = self.create_client(Trigger, '/grasp_once_node/stop')
        self.fk = ForwardKinematics(debug=False)
        self.hand_eye = np.asarray([[0., 0., 1., -.101], [-1., 0., 0., 0.],
                                    [0., -1., 0., .037], [0., 0., 0., 1.]])
        # A cube initially to the right of the image centre, within reach.
        ray = np.array([(370.-320.787)*.225/418.879,
                        (270.-186.311)*.225/419.721, .225, 1.])
        self.cube = self.endpoint([450,800,50,300,500]) @ self.hand_eye @ ray
        self.lower_cube = self.cube.copy()
        self.lower_cube[2] -= .05
        self.create_timer(.02, self.tick)
        self.create_timer(.1, self.camera)

    def endpoint(self, joints):
        xyz, q = self.fk.get_fk(pulse2angle(joints))
        p = Pose(); p.position.x,p.position.y,p.position.z=map(float,xyz);p.orientation=q
        return pose_matrix(p)

    def ready(self, req, res):
        res.success = True
        return res

    def update_positions(self):
        now = time.monotonic()
        for sid,(start,end,at,duration) in self.motions.items():
            alpha=min(1.,max(0.,(now-at)/duration))
            self.positions[sid]=start+(end-start)*alpha

    def tick(self):
        self.update_positions()
        msg=ServoStateList()
        for sid,value in self.positions.items():
            item=ServoState();item.id=sid;item.position=int(round(value))
            msg.servo_state.append(item)
        self.states_pub.publish(msg)

    def read(self, req, res):
        self.update_positions()
        for cmd in req.cmd:
            state=BusServoState();state.position=[int(round(self.positions[cmd.id]))]
            res.state.append(state)
        res.success=True
        return res

    def command(self, msg):
        assert msg.position_unit=='pulse'
        self.update_positions()
        values={int(item.id):int(item.position) for item in msg.position}
        if 5 in values:
            assert values == {1:450, 2:500, 3:250, 4:400, 5:860}, values
            assert {10:50} in self.commands, 'wrist moved before the grip closed'
            assert not self.empty, 'empty grip must not move to hold'
        self.commands.append(values)
        for sid,end in values.items():
            if sid==10 and end==50 and not self.empty:
                end=180  # Synthetic mechanical obstruction, not real grip proof.
            self.motions[sid]=(self.positions[sid],end,time.monotonic(),msg.duration)

    def status(self, msg):
        item=json.loads(msg.data);self.stages.append(item)
        if item['stage']=='search':
            self.expose_cube=True
        print(json.dumps(item),flush=True)

    def camera(self):
        self.update_positions()
        joints=[self.positions[i] for i in (1,2,3,4)]+[500.]
        rgb=np.zeros((400,640,3),np.uint8)
        depth=np.zeros((400,640),np.uint16)
        inverse=np.linalg.inv(self.endpoint(joints) @ self.hand_eye)
        for cube in [self.lower_cube,self.cube]:
            camera=inverse @ cube
            # Invert the production 25mm surface-to-centre compensation exactly.
            camera[:3] *= (camera[2]-.025)/camera[2]
            u=int(round(camera[0]/camera[2]*418.879+320.787))
            v=int(round(camera[1]/camera[2]*419.721+186.311))
            if self.expose_cube and .1<camera[2]<1 and 25<u<615 and 25<v<375:
                cv2.rectangle(rgb,(u-20,v-20),(u+20,v+20),(255,0,0),-1)
                depth[v-20:v+21,u-20:u+21]=int(round(camera[2]*1000))
        stamp=self.get_clock().now().to_msg()
        info=CameraInfo();info.header.stamp=stamp;info.header.frame_id='rgb_camera_link'
        info.width=640;info.height=400;info.k=[418.879,0.,320.787,0.,419.721,186.311,0.,0.,1.]
        self.info_pub.publish(info)
        for array,encoding,pub in [(rgb,'bgr8',self.rgb_pub),(depth,'16UC1',self.depth_pub)]:
            msg=CvBridge().cv2_to_imgmsg(array,encoding);msg.header=info.header;pub.publish(msg)


def main():
    rclpy.init();fixture=Fixture(empty=os.environ.get('TEST_EMPTY_CLOSE')=='1')
    path=Path('/home/ubuntu/raicom_candidates/20260922_id4_comp')/('isolated_'+str(time.time_ns()))
    path.mkdir();print('EVIDENCE',path,flush=True)
    end=time.monotonic()+1
    while time.monotonic()<end:rclpy.spin_once(fixture,timeout_sec=.05)
    names=[name for name,ns in fixture.get_node_names_and_namespaces()]
    assert all(name.startswith(('rgbd_isolated_fixture','_ros2cli')) for name in names),names
    child=None
    try:
        with (path/'launch.log').open('w') as stream:
            launch_file=os.environ.get('RGBD_TEST_LAUNCH_FILE')
            launch_args=[launch_file] if launch_file else ['rgbd_grasp','rgbd_grasp.launch.py']
            child=subprocess.Popen(['ros2','launch',*launch_args,
                'start_camera:=false','start_hardware:=false','start_servo_controller:=false',
                'start_description:=false'],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            end=time.monotonic()+100
            while time.monotonic()<end:
                rclpy.spin_once(fixture,timeout_sec=.03)
                if child.poll() is not None:break
                if fixture.stages and fixture.stages[-1]['stage'] in ('auto_grasp_complete','rejected','failed'):
                    break
            completed_command_count = len(fixture.commands)
            idle_until = time.monotonic() + 2.
            while time.monotonic() < idle_until:
                rclpy.spin_once(fixture, timeout_sec=.03)
            assert len(fixture.commands) == completed_command_count, 'new commands after terminal state'
            stages=[x['stage'] for x in fixture.stages]
            result={'isolated_domain':126,'synthetic':True,'cube_world':fixture.cube.tolist(),
                    'stages':fixture.stages,'commands':fixture.commands}
            (path/'result.json').write_text(json.dumps(result,indent=2))
            print('RESULT',json.dumps(result),flush=True)
            required=['prepare','prepared','auto_grasp','search','centering','centered','precheck','ready','approach','close_gripper']
            for stage in required:assert stage in stages,stage
            ready=next(x for x in fixture.stages if x['stage']=='ready')
            assert ready['candidate_count']==2, ready
            assert np.linalg.norm(np.array(ready['target'])-fixture.cube[:3])<.003, ready
            assert abs(ready['grasp_target'][2]-ready['target'][2]-.015)<.00011, ready
            assert ready['grasp_height_offset_m']==.015
            assert ready['grasp_id4_offset_pulse']==60
            nominal = np.asarray(ready['nominal_final_pulses'])
            commanded = np.asarray(ready['commanded_final_pulses'])
            assert (commanded-nominal).tolist() == [0,0,0,60,0]
            records = sorted((Path.home()/'raicom_runtime/rgbd_grasp/selection/domain_126').glob('*/selection.json'))
            assert records, 'no passive selection record'
            selection = json.loads(records[-1].read_text())
            assert selection['ready_status'] == ready, selection
            assert selection['raw_candidate_count'] == 2, selection
            assert selection['valid_candidate_count'] == 2, selection
            assert selection['selected_uvz'][1] == min(d['v'] for d in selection['detections']), selection
            assert selection['frame_age_at_ready_s'] < 1., selection
            if fixture.empty:
                assert stages[-1]=='rejected'
                assert 'empty-close' in fixture.stages[-1]['detail']
                assert 'move_to_hold_pose' not in stages
            else:
                assert stages[-1]=='auto_grasp_complete'
                assert stages.count('close_gripper')==1
                assert stages.index('close_gripper')<stages.index('move_to_hold_pose')<stages.index('auto_grasp_complete')
                expected = {1:450, 2:500, 3:250, 4:400, 5:860}
                closed_at = fixture.commands.index({10:50})
                assert fixture.commands[closed_at-1] == {
                    sid:int(commanded[sid-1]) for sid in (1,2,3,4)}
                assert fixture.commands[closed_at+1:] == [expected], fixture.commands
                fixture.update_positions()
                assert all(abs(fixture.positions[sid]-value)<1 for sid,value in expected.items())
                assert fixture.positions[10] == 180, 'claw changed after the grip'
                held = next(item for item in fixture.stages if item['stage']=='holding_unconfirmed')
                assert held['hold_pose_reached'] and held['gripper_command_preserved']
                assert held['measured'] == {str(sid):value for sid,value in expected.items()}
            print('PASS: isolated synthetic flow',flush=True)
    finally:
        if fixture.stop_client.service_is_ready():
            future=fixture.stop_client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(fixture,future,timeout_sec=2)
        if child is not None and child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
        fixture.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
