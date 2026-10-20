"""Automatic first, explicit current-request operator fallback. No actuator I/O."""
from copy import deepcopy
import math
import uuid

SCHEMA = 'raicom.physical_confirmation/v1'
IDENTITY = ('backend_id','client_id','request_id')


class Confirmation:
    def __init__(self, kind, identity, provider_id, now, wall, timeout=12.):
        self.kind, self.identity, self.provider_id = kind,deepcopy(identity),provider_id
        self.check_id = uuid.uuid4().hex
        self.started_wall, self.deadline = wall,now+timeout
        self.state, self.reason, self.proof = 'automatic','自动检测中',None

    def request(self):
        return dict(self.identity,schema=SCHEMA,kind=self.kind,check_id=self.check_id,command='check',
                    provider_id=self.provider_id,stamp_unix=self.started_wall)

    def tick(self, now):
        if self.state == 'automatic' and now >= self.deadline:
            self.fallback('自动检测未获得有效结果，请人工检查当前实物')

    def fallback(self, reason):
        if self.state == 'automatic': self.state,self.reason = 'manual_required',reason

    def cancel(self): self.state,self.reason,self.proof = 'cancelled','已取消',None

    def observe(self, value, wall):
        if self.state not in ('automatic','manual_required') or not isinstance(value,dict): return False
        stamp = value.get('stamp_unix')
        if (value.get('schema') != SCHEMA or value.get('check_id') != self.check_id or value.get('kind') != self.kind
                or not self.provider_id or value.get('provider_id') != self.provider_id
                or any(value.get(k) != v for k,v in self.identity.items())
                or type(stamp) not in (float,int) or not math.isfinite(stamp)
                or not self.started_wall <= stamp <= wall+.5 or wall-stamp > 3
                or value.get('source') != 'physical_observation' or not value.get('evidence')):
            return False
        required = ('holding','arm_released') if self.kind == 'holding' else ('placement_confirmed','gripper_empty','arm_released')
        if value.get('status') == 'success' and all(value.get(k) is True for k in required):
            self.proof = deepcopy(value); self.state,self.reason = 'confirmed','自动实物检测通过'; return True
        if value.get('status') in ('failed','uncertain'):
            self.fallback('自动检测不能确认：'+str(value.get('reason','请检查实物')))
        return False

    def operator(self, identity, wall):
        if self.state != 'manual_required' or identity != self.identity: return False
        self.proof = dict(self.request(),source='operator',status='success',stamp_unix=wall,
                          evidence='explicit_current_request_confirmation')
        self.state,self.reason = 'confirmed','人工确认兜底'; return True
