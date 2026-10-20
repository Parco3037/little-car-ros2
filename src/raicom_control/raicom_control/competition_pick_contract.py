"""Competition metadata around the existing grasp interface; no motion or camera."""
from copy import deepcopy

from .vision_decision import decide_markers
from .vision_mapping_cache import mapping_evidence_valid

SCHEMA='raicom.competition.pick/v1'


def validate_context(value):
    if not isinstance(value,dict) or value.get('schema')!=SCHEMA or value.get('event')!='PICK_READY':
        raise ValueError('比赛夹取上下文无效')
    for key in ('task_id','geometry_identity','mapping_id','point'):
        if not isinstance(value.get(key),str) or not value[key]:raise ValueError('比赛夹取缺少本轮身份')
    if type(value.get('item'))is not int or not 1<=value['item']<=8:raise ValueError('本件编号无效')
    if value.get('classification_stage','before_grasp') not in ('before_grasp','after_grasp'):
        raise ValueError('夹取分类阶段无效')
    mapping=value.get('mapping')
    if (not isinstance(mapping,dict) or mapping.get('valid')is not True
            or mapping.get('scope')!='this_task_only'
            or any(mapping.get(k)!=value[k] for k in ('task_id','geometry_identity','mapping_id'))
            or not mapping_evidence_valid(mapping)
            or mapping.get('label_to_park') not in ({'1':'A','2':'B'},{'1':'B','2':'A'})):
        raise ValueError('PICK_READY必须携带本轮有效映射')
    points=mapping.get('category_to_point',{})
    if (set(points)!= {'T','G'} or any(not isinstance(v,str) or not v for v in points.values())
            or points['T']==points['G']):raise ValueError('本轮映射缺少不同园区点位')
    return deepcopy(value)


def context_for(task):
    return validate_context(dict(schema=SCHEMA,event='PICK_READY',task_id=task.task_id,
        geometry_identity=task.geometry,mapping_id=task.mapping['mapping_id'],item=task.completed+1,
        point=task.target,mapping=deepcopy(task.mapping),classification_stage='after_grasp'))


def destination_from_pick(status,request):
    """A/B is the pick program's decision. Optional detections are cross-checked."""
    identity=request.get('pick_identity',{})
    if (not isinstance(status,dict) or status.get('state')!='succeeded' or status.get('holding')is not True
            or status.get('arm_released')is not True or not isinstance(identity,dict)
            or any(not identity.get(k) or status.get(k)!=identity[k] for k in ('backend_id','client_id','request_id'))
            or status.get('request_id')!=request.get('pick_request_id')):
        raise ValueError('园区回报不是当前已夹稳并交还控制权的抓取结果')
    result=status.get('competition_result')
    if not isinstance(result,dict) or result.get('schema')!=SCHEMA:
        raise ValueError('夹取程序尚未返回本件A/B')
    if (type(result.get('item'))is not int or
            any(result.get(k)!=request.get(k) for k in ('task_id','item','mapping_id','geometry_identity'))):
        raise ValueError('夹取园区回报属于其他轮次、物品或映射')
    mapping=request.get('mapping',{})
    if (not isinstance(mapping,dict) or mapping.get('valid')is not True
            or mapping.get('scope')!='this_task_only'
            or not mapping_evidence_valid(mapping)
            or any(mapping.get(k)!=request.get(k) for k in ('task_id','mapping_id','geometry_identity'))
            or mapping.get('label_to_park') not in ({'1':'A','2':'B'},{'1':'B','2':'A'})):
        raise ValueError('园区回报缺少本轮有效映射')
    park=result.get('park')
    if park not in ('A','B'):raise ValueError('夹取程序没有返回有效A/B')
    label=next(label for label,p in mapping['label_to_park'].items() if p==park)
    if result.get('label') is not None and result['label']!=label:raise ValueError('返回类别与A/B、本轮映射不一致')
    decision=None
    if 'marker_payload' in result:
        decision=decide_markers(result['marker_payload'])
        if decision['label']!=label:raise ValueError('最高置信度标识与返回园区不一致')
    point=mapping.get('category_to_point',{}).get('T' if label=='1' else 'G')
    if not isinstance(point,str) or not point:raise ValueError('返回园区没有对应导航点')
    return dict(park=park,label=label,point=point,classification_source='pick_program',
        label_basis='highest_confidence' if decision else 'pick_program_label' if result.get('label') else 'inverse_round_mapping',
        confirmed_frames=1 if decision else 0,decision=decision,pick_destination=deepcopy(result))
