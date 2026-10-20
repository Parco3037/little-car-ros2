"""Competition sequencing only. No ROS or actuator access."""
from copy import deepcopy
import math
import uuid

from .field_tool_delivery import geometry_identity
from .field_tool_workspace_model import point_identity


def configuration(document, value):
    c = deepcopy(value)
    points = document['points']
    if c.get('recognition') not in points:
        raise ValueError('请选择识别区点位')
    parks = c.get('parks', {})
    if set(parks) != {'A', 'B'} or any(p not in points for p in parks.values()) or parks['A'] == parks['B']:
        raise ValueError('请选择两个不同的园区点位')
    order = c.get('pick_order')
    if not isinstance(order, list) or not 1 <= len(order) <= 8 or any(p not in points for p in order):
        raise ValueError('夹取顺序须包含1–8件，每项选择已保存点位')
    if not isinstance(c.get('transport'), str) or not c['transport']:
        raise ValueError('请选择已标定运输姿态')
    for park in ('A', 'B'):
        actions = c.get('placements', {}).get(park)
        if not isinstance(actions, list) or len(actions) != 4 or any(not isinstance(a, str) or not a for a in actions):
            raise ValueError(f'请选择{park}园区四个放置动作')
        if len(set(actions)) != 4:
            raise ValueError(f'{park}园区四个落位须使用不同动作')
    return c


def validate_catalog(document, config, catalog):
    """No guessed pulses. Catalog is owned by the physical arm adapter."""
    if not isinstance(catalog, dict) or catalog.get('geometry_identity') != geometry_identity(document):
        raise ValueError('运输/放置标定尚未绑定当前地图与点位')
    transport = catalog.get('transport', {}).get(config['transport'], {})
    def executable(profile):
        return profile.get('verified') is True or (profile.get('execution_basis')=='user_named_poses'
                and profile.get('runtime_readback_required') is True and profile.get('verified') is False)
    if not executable(transport) or transport.get('holding_preserved') is not True or not transport.get('evidence'):
        raise ValueError('运输姿态缺少验证记录或保持夹紧保证')
    for park in ('A', 'B'):
        fingerprint = point_identity(document['points'][config['parks'][park]])
        for action in config['placements'][park]:
            p = catalog.get('placements', {}).get(action, {})
            points=p.get('bound_point_fingerprints',[]) if p.get('execution_basis')=='user_named_poses' else p.get('verified_point_fingerprints',[])
            if (not executable(p) or not p.get('evidence')
                    or fingerprint not in points
                    or p.get('phases') != ['approach', 'place', 'release', 'withdraw']):
                raise ValueError(f'{park}园区动作 {action} 尚未验证对应落点及四个步骤')


class CompetitionRun:
    TERMINAL = {'complete', 'failed', 'cancelled'}

    def __init__(self, document, config, now):
        self.config = configuration(document, config)
        self.geometry = geometry_identity(document)
        self.task_id = 'competition-' + uuid.uuid4().hex
        self.stage, self.target = 'nav_recognition', self.config['recognition']
        self.started, self.ended, self.deadline = now, None, now + 180
        self.completed, self.counts = 0, {'A': 0, 'B': 0}
        self.used_slots = {'A': [], 'B': []}
        self.mapping = self.pick_result = None
        self.park = None
        self.reason = ''
        self.arrivals, self.placements = [], []
        self.confirmations = []
        self.item_started = now

    def checkpoint(self):
        """Only completed placements are credited; a partial pick is retried."""
        return dict(schema=1, geometry=self.geometry, config=deepcopy(self.config),
                    completed=self.completed, counts=deepcopy(self.counts),
                    used_slots=deepcopy(self.used_slots))

    def resume(self, checkpoint):
        if (not isinstance(checkpoint, dict) or checkpoint.get('schema') != 1
                or checkpoint.get('geometry') != self.geometry
                or checkpoint.get('config') != self.config):
            raise ValueError('续赛记录与当前地图或比赛设置不一致')
        completed, counts, slots = (checkpoint.get(k) for k in
                                    ('completed', 'counts', 'used_slots'))
        if (type(completed) is not int or not 0 <= completed < len(self.config['pick_order'])
                or not isinstance(counts, dict) or set(counts) != {'A', 'B'}
                or not isinstance(slots, dict) or set(slots) != {'A', 'B'}
                or any(type(counts[p]) is not int or not 0 <= counts[p] <= 4
                       or not isinstance(slots[p], list) or len(slots[p]) != counts[p]
                       or len(set(slots[p])) != len(slots[p])
                       or any(type(i) is not int or i not in range(1, 5) for i in slots[p])
                       for p in ('A', 'B'))
                or sum(counts.values()) != completed):
            raise ValueError('续赛记录中的已完成件数或落位无效')
        self.completed, self.counts, self.used_slots = completed, deepcopy(counts), deepcopy(slots)

    def elapsed(self, now):
        return max(0., (self.ended if self.ended is not None else now) - self.started)

    def advance(self, stage, now, seconds=120):
        self.stage, self.deadline = stage, now + seconds

    def navigate_pick(self, now):
        self.target = self.config['pick_order'][self.completed]
        self.item_started = now
        self.advance('nav_pick', now, 180)

    def arrived(self, report, now):
        if self.stage not in {'nav_recognition', 'nav_pick', 'nav_park'}:
            return False
        valid = (report.get('name') == self.target and report.get('nav_status') == 4
                 and report.get('stopped') is True and report.get('accepted') is True)
        for key, limit in [('position_error_m', .02), ('yaw_error_deg', 10.)]:
            value = report.get(key)
            valid = valid and type(value) in (int, float) and math.isfinite(value) and 0 <= value <= limit
        if not valid:
            self.stop('到点未通过停车及2 cm / 10°验收', now)
            return False
        self.arrivals.append(deepcopy(report))
        next_stage = {'nav_recognition': 'recognition', 'nav_pick': 'pick', 'nav_park': 'place'}[self.stage]
        self.advance(next_stage, now, 60 if next_stage == 'recognition' else 600 if next_stage == 'place' else 120)
        return True

    def recognized(self, mapping, now):
        if self.stage != 'recognition': return False
        from .vision_mapping_cache import mapping_evidence_valid
        if (mapping.get('valid') is not True or mapping.get('task_id') != self.task_id
                or not isinstance(mapping.get('mapping_id'), str) or not mapping['mapping_id']
                or mapping.get('scope') != 'this_task_only'
                or not mapping_evidence_valid(mapping)
                or mapping.get('label_to_park') not in ({'1':'A','2':'B'},{'1':'B','2':'A'})
                or mapping.get('geometry_identity') != self.geometry):
            raise ValueError('映射不是本轮当前地图的有效识别结果')
        self.mapping = deepcopy(mapping)
        self.navigate_pick(now)
        return True

    def picked(self, result, expected, now):
        if self.stage != 'pick': return False
        if not isinstance(result, dict) or any(result.get(k) != expected.get(k) for k in ('backend_id', 'client_id', 'request_id')):
            return False
        if (result.get('state') != 'succeeded' or result.get('accepted') is not True
                or result.get('holding') is not True or result.get('arm_released') is not True
                or (result.get('holding_confirmation') or {}).get('source')=='operator'):
            detail = str(result.get('reason') or '抓取结果未确认夹持成功')[:160]
            self.stop('当前抓取未确认夹稳及控制权交还：' + detail, now)
            return False
        self.pick_result = deepcopy(result)
        self.confirmations.append(dict(item=self.completed+1,stage='holding',
            source=(result.get('holding_confirmation') or {}).get('source','unknown')))
        self.advance('classify', now, 60)
        return True

    def classified(self, result, now):
        if self.stage != 'classify': return False
        park, label = result.get('park'), result.get('label')
        if result.get('classification_source')=='operator':raise ValueError('比赛不再采用人工分类确认')
        if park not in ('A', 'B') or label not in ('1', '2') or self.mapping['label_to_park'][label] != park:
            raise ValueError('A/B信号或类别与本轮映射不一致')
        if self.counts[park] >= 4:
            raise ValueError('该园区四个落位已满，不能继续运输')
        self.park = park
        self.confirmations.append(dict(item=self.completed+1,stage='classification',
            source=result.get('classification_source','unknown')))
        self.advance('transport', now)
        return True

    def transported(self, result, now):
        if self.stage != 'transport': return False
        verified = (result.get('physical_readback_verified') is True or
                    (result.get('init_command_completed') is True and result.get('holding_readback_verified') is True))
        if not verified or not all(result.get(k) is True for k in ('holding', 'arm_released')) or not result.get('evidence'):
            raise ValueError('运输姿态缺少真实读回、夹持或控制权交还')
        self.target = self.config['parks'][self.park]
        self.advance('nav_park', now, 180)
        return True

    def placed(self, result, now):
        if self.stage != 'place': return False
        if (result.get('phases_completed') != ['approach', 'place', 'release', 'withdraw']
                or not (result.get('physical_readback_verified') is True or
                        (result.get('init_command_completed') is True
                         and result.get('action_readback_steps') == ['place','release']))
                or result.get('return_pose') != 'init'
                or result.get('arm_released') is not True or not result.get('evidence')):
            raise ValueError('放置/松爪尚未完成，或回init命令尚未结束')
        self.place_evidence = deepcopy(result)
        actual_slot=result.get('executed_slot',self.action_fields()['slot'])
        if (type(actual_slot)is not int or actual_slot not in range(1,5) or actual_slot in self.used_slots[self.park]
                or result.get('executed_profile_id',self.config['placements'][self.park][actual_slot-1])
                    !=self.config['placements'][self.park][actual_slot-1]):
            raise ValueError('实际放置姿态无效或该位置已使用')
        self.used_slots[self.park].append(actual_slot)
        self._complete_item(dict(result,completion_criterion='arrival_and_action_readback',
            confirmation_source='placement_action_readback',physical_placement_observed=False),now)
        return True

    def placement_verified(self, result, now):
        # Kept for late legacy callbacks. Only placed() completes a current item.
        return False

    def _complete_item(self, result, now):
        self.counts[self.park] += 1
        self.completed += 1
        self.confirmations.append(dict(item=self.completed,stage='placement',
            source=result.get('confirmation_source','unknown')))
        self.placements.append({'item': self.completed, 'park': self.park, 'slot': self.used_slots[self.park][-1],
                                'elapsed_s': now - self.item_started, 'result': deepcopy(result),
                                'action_result': self.place_evidence})
        self.advance('reset_pick', now, 10)
        return True

    def reset_done(self, now):
        if self.stage != 'reset_pick': return False
        if self.completed == len(self.config['pick_order']):
            self.stage, self.ended, self.deadline = 'complete', now, None
        else: self.navigate_pick(now)
        return True

    @property
    def manual_interventions(self):
        return sum(c['source']=='operator' for c in self.confirmations)

    @property
    def fully_automatic(self):
        return (self.stage=='complete' and len(self.confirmations)==self.completed*3
                and all(c['source'] in ('native_grasp_readback','physical_observation','http_yolo_held_roi','http_yolo_highest_confidence','http_yolo_post_grasp_pose','pick_program','placement_action_readback') for c in self.confirmations))

    def tick(self, now):
        if self.stage not in self.TERMINAL and self.deadline is not None and now >= self.deadline:
            self.stop('阶段超时：' + self.stage, now)

    def stop(self, reason, now, cancelled=False):
        if self.stage in self.TERMINAL: return
        self.stage = 'cancelled' if cancelled else 'failed'
        self.reason, self.ended, self.deadline = reason, now, None

    def context(self):
        item=self.completed if self.stage in {'reset_pick','complete'} else self.completed+1
        return dict(task_id=self.task_id, item=min(max(1,item),len(self.config['pick_order'])), geometry_identity=self.geometry,
                    mapping_id=self.mapping['mapping_id'] if self.mapping else '',
                    pick_request_id=self.pick_result['request_id'] if self.pick_result else '',
                    pick_identity={k:self.pick_result[k] for k in ('backend_id','client_id','request_id')} if self.pick_result else {})

    def action_fields(self):
        if self.stage == 'classify': return {'mapping': deepcopy(self.mapping),'classification_mode':'post_grasp_pose'}
        if self.stage == 'transport': return {'profile_id': self.config['transport'], 'keep_gripper_closed': True}
        slot = next(i for i in range(1,5) if i not in self.used_slots[self.park])
        return dict(park=self.park, point=self.config['parks'][self.park], slot=slot,
                    profile_id=self.config['placements'][self.park][slot - 1])
