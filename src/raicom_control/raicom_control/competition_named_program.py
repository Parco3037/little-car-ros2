"""User-taught targets with runtime readback; never label commands as measurements."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

from .field_tool_delivery import geometry_identity
from .field_tool_workspace_model import point_identity

SCHEMA='raicom.competition.named_program/v1'
NAMES=('auto1','2','3','4')
PHASES=['approach','place','release','withdraw']
IDS=('1','2','3','4','5','10')
DEFAULT_CLASSIFICATION_POSE=[450,620,180,260,875,50]


def default_source():
    local=Path(__file__).resolve().parents[1]/'config/competition_named_poses.json'
    if local.is_file():return local
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('raicom_control'))/'config/competition_named_poses.json'


def validate_source(source):
    if (source.get('schema')!='raicom.user_named_poses/v1' or source.get('servo_ids')!=[1,2,3,4,5,10]
            or source.get('gripper')!={'id':10,'close':50,'open':350}
            or set(source.get('poses',{}))!=set((*NAMES,'init'))):
        raise ValueError('指定姿态或10号夹爪50/350配置无效')
    for pose in [source.get('transport'),source.get('classification_pose',DEFAULT_CLASSIFICATION_POSE),*source['poses'].values()]:
        if not isinstance(pose,list) or len(pose)!=6 or any(type(v)is not int or not 0<=v<=1000 for v in pose):
            raise ValueError('指定姿态必须包含六个有效脉冲')
    duration=source.get('duration_s')
    if type(duration) not in (int,float) or not math.isfinite(duration) or not 1<=duration<=6:
        raise ValueError('动作时间须为1–6秒')
    if type(source.get('tolerance_pulse')) is not int or not 1<=source['tolerance_pulse']<=30:
        raise ValueError('指定姿态读回容差无效')
    return source


def build_program(document,config,source):
    from .competition_model import configuration
    config=configuration(document,config)
    validate_source(source)
    if config['transport']!='运输保持' or any(set(v)!=set(NAMES) for v in config['placements'].values()):
        raise ValueError('指定动作模式使用运输保持及auto1、2、3、4')
    return dict(schema=SCHEMA,route=deepcopy(document),geometry_identity=geometry_identity(document),
                config=deepcopy(config),source=deepcopy(source),physical_validation='pending')


def save_program(path,document,config,source_path=None):
    path=Path(path)
    value=build_program(document,config,json.loads(Path(source_path or default_source()).read_text(encoding='utf-8')))
    content=json.dumps(value,ensure_ascii=False,indent=2)+'\n'
    if path.is_file() and path.read_text(encoding='utf-8')==content:return path
    path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists():
        old=path.read_bytes();backup=path.with_name(path.stem+'.'+hashlib.sha256(old).hexdigest()[:16]+'.json')
        if not backup.exists():backup.write_bytes(old)
    temporary=path.with_suffix('.pending')
    temporary.write_text(content,encoding='utf-8');temporary.replace(path)
    return path


class NamedProgram:
    def __init__(self,path):
        self.path=Path(path).resolve();data=self.path.read_bytes()
        self.sha256=hashlib.sha256(data).hexdigest()
        self.dependencies={self.path:self.sha256}
        raw=json.loads(data)
        if raw.get('schema')!=SCHEMA:raise ValueError('指定动作文件格式无效')
        expected=build_program(raw['route'],raw['config'],raw['source'])
        if expected!=raw:raise ValueError('指定动作与地图/配置不一致')
        s=raw['source'];self.command_wrist=True
        def pose(name,values,gripper):
            positions=dict(zip(IDS,values));positions['10']=gripper
            return dict(name=name,positions=positions,duration_s=s['duration_s'],tolerance_pulse=s['tolerance_pulse'],
                        origin='user_named_target',physical_readback_verified=False)
        self.pick_hold=pose('抓取收拢',s['transport'],50)
        self.classify_pose=pose('夹取后识别',s.get('classification_pose',DEFAULT_CLASSIFICATION_POSE),50)
        self.carry=pose('运输init',s['poses']['init'],50)
        self.init=pose('init',s['poses']['init'],350)
        self.programs={name:{'approach':deepcopy(self.carry),'place':pose(name,s['poses'][name],50),
            'release':pose(name,s['poses'][name],350),'withdraw':deepcopy(self.init)} for name in NAMES}
        # These bounds detect empty closure/opening; they do NOT prove an object is held.
        self.value=dict(raw,gripper=dict(close=50,open=350,holding_min=61,holding_max=334,tolerance=15),
            fixed_wrist=dict(position=860,tolerance=25),classification=dict(pose=self.classify_pose,
            decision_source='http_yolo_post_grasp_pose'),placement_feedback=dict(verified=False))
        proof=dict(path=str(self.path),sha256=self.sha256,source='user_specified_targets')
        base=dict(verified=False,execution_basis='user_named_poses',runtime_readback_required=True,evidence=proof)
        fingerprints=[point_identity(raw['route']['points'][p]) for p in raw['config']['parks'].values()]
        self.catalog=dict(geometry_identity=raw['geometry_identity'],physical_validation='pending',
            transport={'运输保持':dict(base,holding_preserved=True)},
            placements={name:dict(base,phases=list(PHASES),bound_point_fingerprints=fingerprints,
                                 return_pose='init',gripper_close=50,gripper_release=350,
                                 completion_criterion='arrival_and_action_readback') for name in NAMES})

    def unchanged(self):
        return all(path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest()==digest
                   for path,digest in self.dependencies.items())

    def program(self,request):
        if request.get('geometry_identity')!=self.value['geometry_identity']:raise ValueError('地图或点位已变化')
        if request['operation']=='transport':
            if request.get('profile_id')!='运输保持' or request.get('keep_gripper_closed')is not True:
                raise ValueError('运输须保持本次夹爪指令')
            return deepcopy(self.pick_hold),[('transport',deepcopy(self.carry),'holding')]
        park,slot=request.get('park'),request.get('slot');config=self.value['config']
        if (park not in ('A','B') or type(slot)is not int or not 1<=slot<=4 or
                request.get('point')!=config['parks'][park] or
                request.get('profile_id')!=config['placements'][park][slot-1]):
            raise ValueError('园区、计数或指定动作不匹配')
        program=self.programs[request['profile_id']]
        return deepcopy(self.carry),[(phase,deepcopy(program[phase]),'holding' if phase in ('approach','place')
                                     else 'release' if phase=='release' else 'open') for phase in PHASES]


def load_program(path):
    if json.loads(Path(path).read_text(encoding='utf-8')).get('schema')==SCHEMA:return NamedProgram(path)
    from .competition_calibration import Calibration
    return Calibration(path)
