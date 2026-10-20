"""User-selected init before chassis dispatch; no motion on import or preview."""
import time

INIT_POSITIONS = {'1': 450, '2': 820, '3': 174, '4': 93, '5': 865, '10': 50}


def init_pose():
    return dict(name='init', positions=dict(INIT_POSITIONS), duration_s=3.,
                tolerance_pulse=15, origin='user_named_target', physical_readback_verified=False)


class MotionInitMixin:
    def _init_holding_reference(self):
        client = getattr(self, 'depth_client', None)
        if client is None:
            return None
        status = client.session.status or {}
        if status.get('holding') is not True:
            return None
        if not client.session.fresh(time.monotonic()) or status.get('arm_released') is not True:
            raise RuntimeError('夹持结果过期或机械臂控制权尚未交还')
        value = ((status.get('motion_result') or {}).get('hold') or {}).get('gripper_readback')
        if type(value) is not int or not 61 <= value <= 334:
            raise RuntimeError('缺少本次运输的有效夹爪读回')
        return value

    def _ensure_motion_init(self, token):
        job = getattr(self, 'motion_init_job', None)
        if job and job['token'] == token:
            if job['error']:
                raise RuntimeError(job['error'])
            return job['complete']
        if job and not job['complete'] and not job['error']:
            return False
        if self._depth_busy():
            raise RuntimeError('抓取尚未结束，不能回init或行驶')
        if not self.node.zero_velocity_hold_ok():
            return False
        arm = self._arm()
        if arm.busy:
            raise RuntimeError('其他机械臂动作尚未结束')
        reference = self._init_holding_reference()
        arm.start('init_pose', holding_reference=reference)
        self.motion_init_job = dict(token=token, complete=False, error='', started=time.monotonic())
        self._set_status('行驶前回固定 init · 夹爪50 · 等待真实到位')
        return False

    def _poll_motion_init(self):
        job = getattr(self, 'motion_init_job', None)
        if not job or job['complete'] or job['error']:
            return
        arm = self._arm()
        arm.poll()
        if time.monotonic() - job['started'] > 20 and arm.busy:
            arm.cancel('行驶前init读回超时')
        if arm.state == 'failed':
            job['error'] = '行驶前init未完成：' + arm.error
            self._set_status(job['error'])
        elif arm.state == 'complete':
            arm.release_control()
            job['complete'] = True
            self._workspace_journal(dict(kind='motion_init_verified', token=str(job['token']),
                                         positions=dict(arm.positions), gripper_command=50))

    def _cancel_motion_init(self, reason):
        job = getattr(self, 'motion_init_job', None)
        if job and not job['complete'] and not job['error']:
            self._arm().cancel(reason)
            job['error'] = reason
