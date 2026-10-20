"""Host the installed grasp algorithm; publishers exist only during one request."""
from rgbd_grasp.grasp_once_node import GraspOnceNode
from rgbd_grasp.grasp_core import Rejected
from rgbd_grasp.grasp_core import select_candidate_uvz, stable_target


class WarmGraspHost(GraspOnceNode):
    def __init__(self):
        self.owner = None
        super().__init__()  # Native defaults disable motion and automatic preparation.
        if self.parameters['motion_enabled'] or self.auto_prepare_timer is not None:
            raise RuntimeError('常驻抓取节点必须以禁用动作模式初始化')
        # User's explicit gripper settings; keep the repaired trajectory unchanged.
        self.parameters['gripper_close'] = 50
        self.parameters['gripper_open'] = 350
        self.parameters['initial_pose'][-1] = 350
        self.parameters['auto_grasp'] = True
        # The grasp ends at the user-taught YOLO observation pose. There is no
        # intermediate trip through the former 450/500/250/400/860 hold pose.
        self.parameters['carry_pose'] = [450, 620, 180, 260, 875]
        self.parameters['carry_pose_duration_s'] = 1.0
        # One stable comparison (two physical reads), as requested. Native
        # goal tolerance, <=2-pulse stability and command duration still apply.
        self.parameters['stable_joint_samples'] = 1
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter(k, value=self.parameters[k]) for k in
                             ('gripper_close', 'gripper_open', 'initial_pose', 'auto_grasp',
                              'stable_joint_samples', 'carry_pose', 'carry_pose_duration_s')])
        original_guard = self.hardware.guard
        def guarded():
            original_guard()
            if not self.owner or not self.owner.stationary():
                raise Rejected('底盘未停稳或停车反馈失联')
            self.assert_single_command_source()
        self.hardware.guard = guarded

    def blocked(self, request, response):
        return self.finish(response, False, '请通过带请求编号的 /raicom/depth_pick/request 执行')

    prepare_callback = grasp_callback = release_callback = precheck_callback = blocked

    def candidate_callback(self, message):
        # Calculate locally before replacing shared data. The native callback
        # first clears uvz to None while numpy selection releases the GIL;
        # a persistent worker can observe that intermediate, invalid state.
        try:selected,count=select_candidate_uvz(message.data,self.parameters['target_selection'])
        except Rejected:selected,count=None,0
        self.last_candidate_uvz=selected
        self.last_candidate_count=count
        self.last_candidate_stamp_s=self.get_clock().now().nanoseconds*1e-9

    def current_target(self):
        # ROS callbacks append while the worker calculates stability. Snapshot
        # the deque, retaining the native validation and numeric parameters.
        return stable_target(list(self.targets),self.get_clock().now().nanoseconds*1e-9,
            self.parameters['target_frame'],int(self.parameters['stable_target_samples']),
            float(self.parameters['target_max_age_s']),float(self.parameters['stable_target_spread_m']),
            self.parameters['workspace'])

    def current_candidate_u(self):
        selected=self.last_candidate_uvz
        age=self.get_clock().now().nanoseconds*1e-9-self.last_candidate_stamp_s
        if age>float(self.parameters['candidate_max_age_s']):raise Rejected('candidate pixels are stale')
        if not self.last_candidate_count or selected is None:raise Rejected('no fresh selected blue cube candidate')
        if self.parameters['require_single_candidate'] and self.last_candidate_count!=1:
            raise Rejected('expected exactly one fresh blue cube candidate')
        return float(selected[0])

    def publish_status(self, stage, detail, **extra):
        super().publish_status(stage, detail, **extra)
        if self.owner:
            self.owner.native_status(dict(stage=stage, detail=detail, **extra))

    def acquire(self, owner):
        if self.execution_lock.locked() or self.hardware.command_pub is not None or self.holding:
            raise RuntimeError('当前抓取尚未释放或夹爪仍占用')
        if not owner.arm_free() or not owner.stationary(True) or owner.cancel.is_set():
            raise RuntimeError('开始抓取前停车或控制权检查未通过')
        from servo_controller_msgs.msg import ServosPosition
        from ros_robot_controller_msgs.msg import SetBusServoState
        self.owner = owner
        self.cancel_event = owner.cancel
        self.hardware.command_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.hardware.stop_pub = self.create_publisher(SetBusServoState, '/ros_robot_controller/bus_servo/set_state', 1)
        self.parameters['motion_enabled'] = True
        self.prepared = False
        self.clear_perception()

    def relinquish(self):
        if self.execution_lock.locked():
            raise RuntimeError('原生动作线程尚未退出')
        self.parameters['motion_enabled'] = False
        for key in ('command_pub', 'stop_pub'):
            publisher = getattr(self.hardware, key)
            if publisher is not None:
                self.destroy_publisher(publisher)
                setattr(self.hardware, key, None)
        self.owner = None

