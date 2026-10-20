"""Request, lease and physical-feedback gates, independent of ROS."""
from copy import deepcopy
import math
import re

from .competition_io import SCHEMA, KEYS, OPERATIONS, recent
from .vision_mapping_cache import mapping_evidence_valid


def valid_request(value, backend_id, wall):
    if (not isinstance(value, dict) or value.get('schema') != SCHEMA
            or value.get('backend_id') != backend_id or value.get('command') not in ('start','cancel')
            or value.get('operation') not in OPERATIONS or not recent(value,wall)):
        return False
    for key in KEYS:
        if key == 'item':
            if type(value.get(key)) is not int or not 1 <= value[key] <= 8: return False
        elif not isinstance(value.get(key),str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}',value[key]): return False
    expires, timeout = value.get('expires_unix'), value.get('timeout_s')
    return (type(expires) in (int,float) and math.isfinite(expires) and wall <= expires <= value['stamp_unix']+3.1
            and type(timeout) in (int,float) and math.isfinite(timeout)
            and 1 <= timeout <= (600 if value['operation']=='place' else 120))


def identity(value): return tuple(value[k] for k in KEYS)


def item_identity(value):
    return tuple(value[k] for k in ('client_id','task_id','item','geometry_identity','mapping_id','pick_request_id'))


class RequestGate:
    def __init__(self, backend_id):
        self.backend_id = backend_id
        self.pending = None
        self.seen, self.stopped_tasks, self.leases = set(), set(), {}
        self.cancel_reason = ''
        self.deadline = 0.

    def lease(self, value, now, wall):
        if not isinstance(value,dict) or value.get('schema') != SCHEMA: return
        if any(not isinstance(value.get(k),str) or not value[k] for k in ('client_id','task_id')): return
        expires = value.get('expires_unix')
        if type(expires) not in (int,float) or not math.isfinite(expires) or not 0 < expires-wall <= 1.6: return
        key = value['client_id'],value['task_id']
        if value.get('active') is False:
            self.stopped_tasks.add(key)
            if self.pending and key == (self.pending['client_id'],self.pending['task_id']): self.cancel_reason = '本轮已停止'
        elif value.get('active') is True and key not in self.stopped_tasks:
            self.leases[key] = now+min(1.5,expires-wall)

    def leased(self, request, now):
        key = request['client_id'],request['task_id']
        return key not in self.stopped_tasks and now < self.leases.get(key,0.)

    def admit(self, value, now, wall):
        if not valid_request(value,self.backend_id,wall): return 'invalid'
        key = value['client_id'],value['request_id']
        if value['command'] == 'cancel':
            self.seen.add(key)
            if self.pending and identity(self.pending) == identity(value):
                self.cancel_reason = '取消当前请求'; return 'cancel'
            return 'cancelled_before_start'
        if value['command'].startswith('confirm_'):
            if self.pending and identity(self.pending) == identity(value) and not self.cancel_reason and self.leased(value,now):
                return value['command']
            return 'invalid'
        if key in self.seen: return 'duplicate'
        if self.pending: return 'busy'
        self.seen.add(key)
        if not self.leased(value,now): return 'lease_missing'
        self.pending, self.deadline = deepcopy(value), now+value['timeout_s']
        self.cancel_reason = ''
        return 'start'

    def check(self, now):
        if self.pending:
            if not self.leased(self.pending,now): self.cancel_reason = self.cancel_reason or '工作台租约丢失'
            if now >= self.deadline: self.cancel_reason = self.cancel_reason or '操作超时'
        return not self.cancel_reason


def valid_mapping(mapping, request):
    return (isinstance(mapping,dict) and mapping.get('valid') is True and mapping.get('scope') == 'this_task_only'
            and mapping.get('task_id') == request['task_id'] and mapping.get('mapping_id') == request['mapping_id']
            and mapping.get('geometry_identity') == request['geometry_identity']
            and mapping_evidence_valid(mapping)
            and mapping.get('label_to_park') in ({'1':'A','2':'B'},{'1':'B','2':'A'})
            and isinstance(mapping.get('category_to_point'),dict))


def feedback_valid(value, request, provider, wall, after_wall):
    if (not isinstance(value,dict) or value.get('schema') != SCHEMA or not recent(value,wall)
            or value.get('stamp_unix',0) < after_wall or value.get('provider_id') != provider
            or any(value.get(k) != request[k] for k in KEYS)
            or any(value.get(k) != request.get(k) for k in ('park','point','slot','profile_id','placement_request_id'))):
        return False
    # These attestations must come from the installed, separately verified physical
    # observer. Servo completion and the pick backend's ownership flag are insufficient.
    return (value.get('status') == 'success' and value.get('placement_confirmed') is True
            and value.get('gripper_empty') is True and value.get('arm_released') is True
            and value.get('source') == 'physical_observation' and bool(value.get('evidence')))
