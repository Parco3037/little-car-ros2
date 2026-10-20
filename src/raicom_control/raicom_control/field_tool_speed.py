"""Requested speeds and mandatory controller/smoother/guard readback."""
import math
import time
from .speed_limits import LINEAR_CEILING, ANGULAR_CEILING, GUARD_SPEED_CONTRACT, guard_targets, linear_target


def speed_settings(value=None):
    result = dict(mode='phased', far=.25, near=.12, fine=.03, near_distance=.25, turn=.35)
    result.update(value or {})
    if result['mode'] not in ('phased', 'legacy'): raise ValueError('未知接近模式')
    for key in ('far', 'near', 'fine', 'near_distance', 'turn'):
        if type(result[key]) not in (int, float) or not math.isfinite(result[key]): raise ValueError('速度必须为有限数值')
    for key in ('far','near','fine'):
        if not 0 < result[key] <= LINEAR_CEILING:raise ValueError('行走速度须大于0且不超过软件上限2.0 m/s')
    if result['near_distance']<0:raise ValueError('减速距离不能为负数；0表示不启用接近段')
    if not 0<result['turn']<=ANGULAR_CEILING:raise ValueError('转弯速度须大于0且不超过软件上限2.0 rad/s')
    return result


NAMES = {
    'controller_server': ['FollowPath.plugin', 'FollowPath.far_speed', 'FollowPath.near_speed',
        'FollowPath.fine_speed', 'FollowPath.near_distance', 'FollowPath.validated_cap',
        'FollowPath.max_vel_x', 'FollowPath.max_speed_xy', 'FollowPath.min_vel_x',
        'general_goal_checker.xy_goal_tolerance', 'general_goal_checker.yaw_goal_tolerance',
        'general_goal_checker.stateful', 'FollowPath.xy_goal_tolerance',
        'FollowPath.turn_speed','FollowPath.max_vel_theta',
        'FollowPath.acc_lim_x','FollowPath.decel_lim_x','FollowPath.acc_lim_theta','FollowPath.decel_lim_theta'],
    'velocity_smoother': ['max_velocity', 'min_velocity','max_accel','max_decel'],
    'cmd_vel_guard': ['max_linear', 'max_angular','speed_contract_version','max_linear_accel','max_angular_accel'],
}
SET_PHASES={'set_controller':'controller_server','set_smoother':'velocity_smoother','set_guard':'cmd_vel_guard'}


def check_readback(config, readings, *, require_requested=True):
    config = speed_settings(config)
    if any(not isinstance(readings.get(name),(list,tuple)) or len(readings[name])!=len(names) for name,names in NAMES.items()):
        raise ValueError('限速回读缺少参数；请核对候选控制器与guard版本')
    c, s, g = (readings[name] for name in NAMES)
    if c[0] != 'raicom_nav2_workspace::PhasedDwbController': raise ValueError('当前导航服务不是分阶段模式，请先结束原导航服务')
    expected = [config[k] for k in ('far', 'near', 'fine', 'near_distance')]
    if any(type(n) not in (int, float) or not math.isfinite(n) for n in c[1:11]+c[12:]+g[:2]+g[3:]):
        raise ValueError('控制器 / guard 速度回读无效')
    if require_requested and any(abs(a-b)>1e-8 for a,b in zip(c[1:5],expected)): raise ValueError('速度请求与控制器回读不一致')
    if (any(not 0<n<=LINEAR_CEILING for n in c[1:4]+c[5:8]) or c[4]<0. or c[8]!=0. or
            any(not 0<n<=ANGULAR_CEILING for n in c[13:15]) or
            abs(c[9]-.02)>1e-9 or abs(c[10]-math.radians(10))>1e-9 or c[11] is not False or abs(c[12]-.02)>1e-9):
        raise ValueError('控制器速度 / 2 cm / 10° / stateful 配置不一致')
    if any(not isinstance(v, (list, tuple)) or len(v)!=3 or any(type(n) not in (int,float) or not math.isfinite(n) for n in v) for v in s):
        raise ValueError('平滑器速度回读无效')
    if not 0<s[0][0]<=LINEAR_CEILING or s[0][1]!=0. or not -.08<=s[1][0]<=0. or s[1][1]!=0.:
        raise ValueError('平滑器限速超出当前软件边界')
    if not 0<g[0]<=LINEAR_CEILING or not 0<g[1]<=ANGULAR_CEILING or not 0<s[0][2]<=ANGULAR_CEILING or not -ANGULAR_CEILING<=s[1][2]<0.:
        raise ValueError('guard 或转向限速无效')
    if g[2]!=GUARD_SPEED_CONTRACT:raise ValueError('guard 尚未支持停车动态调速；必须先核验新版本')
    if c[15:]!=[.5,-.5,.7,-.7] or s[2:]!=[[.5,0.,.7],[-.5,0.,-.7]] or g[3:]!=[.5,.7]:
        raise ValueError('控制器 / 平滑器 / guard 的原加减速度不一致')
    cap = min(c[5:8]+[s[0][0], g[0]])
    if require_requested and any(abs(n-linear_target(config))>1e-8 for n in c[5:8]+[s[0][0]]):
        raise ValueError('控制器与平滑器未同时采用所设实际限速')
    if require_requested and any(abs(n-config['turn'])>1e-8 for n in c[13:15]+[s[0][2],-s[1][2]]):
        raise ValueError('控制器与平滑器转弯速度未同步')
    if require_requested and any(abs(a-b)>1e-8 for a,b in zip(g[:2],guard_targets(config))):
        raise ValueError('guard 未采用所设线速度 / 角速度上限')
    return {'cap': cap, 'far': min(config['far'], cap), 'near': min(config['near'], cap),
            'fine': min(config['fine'], cap), 'turn':min(c[13:15]+[s[0][2],-s[1][2],g[1]]), 'readback': readings}


class ProfileReadback:
    """Only runs while parked, before granting any navigation motion lease."""
    def __init__(self, node, config):
        from rcl_interfaces.srv import GetParameters, SetParametersAtomically
        self.node, self.config = node, speed_settings(config)
        # Cancelling a ROS service future cannot cancel the server-side write.
        # Retain outstanding writes across UI attempts and drain them before
        # another transaction, so an old delayed write cannot overtake a new one.
        self.unsettled = node.__dict__.setdefault('_raicom_speed_writes', [])
        self.get_type, self.set_type = GetParameters, SetParametersAtomically
        self.clients = {name: node.create_client(GetParameters, '/' + name + '/get_parameters') for name in NAMES}
        self.setters = {name:node.create_client(SetParametersAtomically, '/'+name+'/set_parameters_atomically')
                        for name in SET_PHASES.values()}
        self.phase, self.pending, self.readings = 'preflight', {}, {}
        # Cold Nav2 parameter services can take longer while all lifecycle
        # nodes start. This transaction keeps the base parked until readback.
        self.deadline = time.monotonic()+30.
        self.result = None

    @staticmethod
    def _values(response):
        if response is None: raise ValueError('参数读取没有返回')
        def decode(v):
            if v.type==1: return v.bool_value
            if v.type==3: return v.double_value
            if v.type==4: return v.string_value
            if v.type==8: return list(v.double_array_value)
            raise ValueError('参数类型无效')
        return [decode(v) for v in response.values]

    def poll(self):
        if self.result is not None: return self.result
        if time.monotonic()>self.deadline: raise RuntimeError('控制器 / 平滑器 / guard 配置回读超时')
        if not self.node.zero_velocity_hold_ok(): raise RuntimeError('应用导航速度时必须停稳')
        for client, future in list(self.unsettled):
            if future.done():
                self.node.destroy_client(client)
                self.unsettled.remove((client,future))
        if self.unsettled:return None
        if self.phase in SET_PHASES:
            name=SET_PHASES[self.phase]
            setter=self.setters[name]
            if 'set' not in self.pending:
                if not setter.service_is_ready(): return
                from rclpy.parameter import Parameter
                if name=='controller_server':
                    data={'FollowPath.'+key+'_speed':float(self.config[key]) for key in ('far','near','fine')}
                    data['FollowPath.near_distance']=float(self.config['near_distance'])
                    data.update({key:linear_target(self.config) for key in ('FollowPath.validated_cap','FollowPath.max_vel_x','FollowPath.max_speed_xy')})
                    data.update({key:float(self.config['turn']) for key in ('FollowPath.turn_speed','FollowPath.max_vel_theta')})
                elif name=='velocity_smoother':
                    maximum=list(self.before['velocity_smoother'][0]);maximum[0]=linear_target(self.config)
                    maximum[2]=float(self.config['turn'])
                    minimum=list(self.before['velocity_smoother'][1]);minimum[2]=-float(self.config['turn'])
                    data={'max_velocity':maximum,'min_velocity':minimum}
                else:
                    data=dict(zip(('max_linear','max_angular'),guard_targets(self.config)))
                values=[Parameter(key,value=value).to_parameter_msg() for key,value in data.items()]
                self.pending['set']=setter.call_async(self.set_type.Request(parameters=values))
            elif self.pending['set'].done():
                if not self.pending['set'].result().result.successful: raise RuntimeError(name+' 拒绝速度设置；保持停车，配置需重新应用')
                self.pending={};self.readings={}
                self.phase={'set_controller':'set_smoother','set_smoother':'set_guard','set_guard':'read'}[self.phase]
        else:
            for name, names in NAMES.items():
                if name not in self.pending:
                    if self.clients[name].service_is_ready(): self.pending[name]=self.clients[name].call_async(self.get_type.Request(names=names))
                elif name not in self.readings and self.pending[name].done():
                    values=self._values(self.pending[name].result())
                    if len(values)!=len(names): raise ValueError(name+' 参数回读数量不一致；请核对该节点的参数声明')
                    self.readings[name]=values
            if len(self.readings)==len(NAMES):
                if self.phase=='preflight':
                    check_readback(self.config,self.readings,require_requested=False)
                    self.before=self.readings;self.readings={};self.pending={};self.phase='set_controller'
                else:self.result=check_readback(self.config,self.readings)
        return self.result

    def close(self):
        retained=[]
        for key, future in self.pending.items():
            client = self.setters[SET_PHASES[self.phase]] if key=='set' else self.clients[key]
            if not future.done():
                if key=='set':
                    self.unsettled.append((client,future));retained.append(client)
                else:client.remove_pending_request(future);future.cancel()
        for client in list(self.clients.values())+list(self.setters.values()):
            if client not in retained:self.node.destroy_client(client)
        self.pending={};self.clients={};self.setters={}
