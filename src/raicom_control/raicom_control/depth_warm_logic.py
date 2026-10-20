"""Pure checks for automatic native-grasp evidence; no physical sensor claims."""
from copy import deepcopy
from .competition_pick_contract import SCHEMA, validate_context
from .vision_decision import decide_markers


def marker_result(context, payload):
    context = validate_context(context)
    decision = decide_markers(payload)
    label = decision['label']
    if label not in ('1', '2'):
        raise ValueError('本次标志没有有效1/2判定')
    return dict(schema=SCHEMA, **{k:context[k] for k in ('task_id','item','geometry_identity','mapping_id')},
                label=label, park=context['mapping']['label_to_park'][label],
                marker_payload=deepcopy(payload), marker_decision=decision)


def automatic_holding(motion, terminal, samples):
    if not motion.complete(terminal) or len(samples) < 2:
        raise ValueError('缺少本次完整抓取与观察姿态动作记录')
    for values in samples[-2:]:
        if (set(values) != {1,2,3,4,5,10} or
                any(type(v) is not int or not 0 <= v <= 1000 for v in values.values())):
            raise ValueError('抓取结束读回缺失或数值无效')
        # The native motion result already verifies the observation target at
        # completion. Its later settled joints can sag; the classification
        # action commands and verifies the pose again before YOLO. Here only
        # require a stable post-close claw that did not reach empty-close 50.
        if not 61 <= values[10] <= 334:
            raise ValueError(f'夹爪非空读回未通过：ID10={values[10]}，闭爪命令50，要求61–334')
    if any(abs(samples[-1][i]-samples[-2][i])>3 for i in samples[-1]):
        raise ValueError('抓取结束后舵机读回仍未稳定')
    return dict(source='native_grasp_readback', samples=deepcopy(samples[-2:]),
                physical_holding_confirmed=False,
                criterion='native_observation_pose_complete_and_stable_nonempty_gripper_readback')
