"""Offline continuous scan tracking prototype. NO ROS, TF, or actuator output.

Fixed references prevent frame-by-frame accumulation while stationary. A strong
overlapping observation may become a keyframe, preserving the accumulated pose.
This remains local odometry, not loop closure, global localization or navigation.
"""
from dataclasses import dataclass
import math

import numpy as np

from scan_anchor_experiment import match, rotation, scan_points, surface_reference


def compose(left, right):
    """T_world_reference * T_reference_current, with continuous (unwrapped) yaw."""
    a,b=np.asarray(left,dtype=float),np.asarray(right,dtype=float)
    xy=a[:2]+rotation(a[2])@b[:2]
    return np.array([xy[0],xy[1],a[2]+b[2]])


def angle_difference(a,b):
    return math.atan2(math.sin(a-b),math.cos(a-b))


def sensor_pose_to_base(pose,mount_rotation,mount_translation):
    """Conjugate relative lidar motion by its measured base<-lidar mount.

    Supports a horizontal inverted sensor too. This is still relative to the
    first base pose, never an estimate in the saved map's coordinates.
    """
    p=np.asarray(pose,dtype=float); r=np.asarray(mount_rotation,dtype=float)
    t=np.asarray(mount_translation,dtype=float)
    if p.shape!=(3,) or r.shape!=(2,2) or t.shape!=(2,) or \
       not all(np.isfinite(v).all() for v in (p,r,t)):
        raise ValueError('Invalid relative pose or mounting')
    if not np.allclose(r@r.T,np.eye(2),atol=1e-5):
        raise ValueError('Mounting must be planar orthogonal')
    base_rotation=r@rotation(p[2])@r.T
    xy=r@p[:2]+t-base_rotation@t
    return np.array([xy[0],xy[1],np.linalg.det(r)*p[2]])


def relative_to(origin,pose):
    origin,pose=np.asarray(origin),np.asarray(pose)
    xy=rotation(-origin[2])@(pose[:2]-origin[:2])
    return np.array([*xy,pose[2]-origin[2]])


def strong_geometry(result):
    """Classify overlap strength without accepting an unconverged pose.

    This is only a work-budget decision.  A fit that exhausts its numerical
    iteration budget remains invalid; strong active-reference overlap merely
    means that spending up to two more full ICP budgets on historical
    references is not justified for the same scan.
    """
    values = [result.get('inlier_fraction'), result.get('hessian_condition'),
              result.get('inlier_plane_rms_m')]
    return (all(value is not None and math.isfinite(value) for value in values)
            and values[0] >= .7 and values[1] <= 100 and values[2] <= .015)


def strong_fit(result):
    return bool(result.get('valid')) and strong_geometry(result)


@dataclass(frozen=True)
class TrackingConfig:
    max_age_sec: float=.3
    # Match RuntimeFusionGate.MAX_RECOVERY_SEC: rejected measurements never
    # publish, while a live scan stream gets the documented one-second window
    # to produce a new accepted match before loss is latched.
    max_gap_sec: float=1.
    max_linear_speed: float=.45
    max_angular_speed: float=.8
    translation_noise_m: float=.015
    rotation_noise_rad: float=.025
    keyframe_distance_m: float=.12
    keyframe_angle_rad: float=.10
    keyframe_confirmation_frames: int=2

    def __post_init__(self):
        for name,value in vars(self).items():
            if not math.isfinite(value) or value<=0:
                raise ValueError('Invalid positive tracking parameter '+name)
        if self.keyframe_distance_m>=.25 or self.keyframe_angle_rad>=.25:
            raise ValueError('Keyframe thresholds must leave room inside matcher domain')
        if type(self.keyframe_confirmation_frames) is not int or self.keyframe_confirmation_frames<2:
            raise ValueError('Keyframe confirmation needs at least two frames')


class ScanTracker:
    def __init__(self,config=None):
        self.config=config or TrackingConfig()
        self.reset()

    def reset(self):
        """Explicit caller action. Never automatically reset/teleport after loss."""
        self.status='UNINITIALIZED'; self.geometry=None; self.target=None
        self.keyframe_pose=np.zeros(3); self.relative_pose=np.zeros(3)
        self.last_pose=None; self.last_good_stamp=None; self.last_seen_stamp=None
        self.last_now=None; self.keyframe_id=0; self.promotion_streak=0
        self.reference_bank=[]; self.active_reference_id=0
        self.pending_reference_id=None; self.pending_reference_streak=0

    def remember_reference(self):
        self.reference_bank.append(dict(id=self.keyframe_id,pose=self.keyframe_pose.copy(),target=self.target))
        self.reference_bank=self.reference_bank[-8:]
        self.active_reference_id=self.keyframe_id
        self.pending_reference_id=None; self.pending_reference_streak=0

    def historical_reference(self,current,result):
        """Try at most two nearby stored references when the active fit weakens.

        Never widen ICP's local domain. Two consecutive strong matches to the
        same historical reference are needed; no reset, teleport or map lookup.
        """
        if result['valid'] and result['inlier_fraction']>=.75:
            self.pending_reference_id=None; self.pending_reference_streak=0
            return result,None
        if not result['valid'] and strong_geometry(result):
            # Numerical non-convergence is still a rejected measurement.  A
            # strong active-reference overlap is not a lost-reference signal,
            # so historical retries would only multiply the callback/GIL
            # budget without making this pose valid.
            self.pending_reference_id=None; self.pending_reference_streak=0
            return result,None
        candidates=[]
        for entry in self.reference_bank:
            if entry['id']==self.active_reference_id: continue
            seed=relative_to(entry['pose'],self.last_pose)
            if np.linalg.norm(seed[:2])>.25 or abs(seed[2])>.25: continue
            candidates.append((float(np.linalg.norm(seed[:2])+.15*abs(seed[2])),entry,seed))
        alternatives=[]
        for _,entry,seed in sorted(candidates,key=lambda item:item[0])[:2]:
            fit=match(current,*entry['target'],initial_pose=seed,allow_bounded_plateau=True)
            if not strong_fit(fit): continue
            if result['valid'] and fit['inlier_fraction']<result['inlier_fraction']+.08: continue
            proposed=compose(entry['pose'],[fit['x_m'],fit['y_m'],fit['yaw_rad']])
            if result['valid']:
                active=compose(self.keyframe_pose,[result['x_m'],result['y_m'],result['yaw_rad']])
                if np.linalg.norm(proposed[:2]-active[:2])>.02 or abs(angle_difference(proposed[2],active[2]))>.04:
                    continue
            alternatives.append((fit['inlier_fraction'],entry,fit))
        if not alternatives:
            self.pending_reference_id=None; self.pending_reference_streak=0
            return result,None
        _,entry,fit=max(alternatives,key=lambda item:item[0])
        self.pending_reference_streak=self.pending_reference_streak+1 if self.pending_reference_id==entry['id'] else 1
        self.pending_reference_id=entry['id']
        if self.pending_reference_streak<2: return result,None
        # Commit reference selection only after the caller's temporal jump gate.
        return fit,entry

    def state_report(self,reason):
        return dict(valid=False,status=self.status,reason=reason,keyframe_id=self.keyframe_id,
                    last_good_pose=None if self.last_pose is None else self.last_pose.tolist(),
                    last_good_stamp=self.last_good_stamp)

    def reject(self,reason,fatal=False,keep_history_candidate=False):
        self.promotion_streak=0
        if fatal or not keep_history_candidate:
            self.pending_reference_id=None; self.pending_reference_streak=0
        if fatal: self.status='LOST'
        return self.state_report(reason)

    def clock_check(self,now):
        if not math.isfinite(now): return self.reject('Nonfinite clock',fatal=True)
        if self.last_now is not None and now<self.last_now:
            return self.reject('Clock moved backwards',fatal=True)
        self.last_now=now
        if self.last_good_stamp is not None and now-self.last_good_stamp>self.config.max_gap_sec:
            return self.reject('No recent accepted scan; explicit reset required',fatal=True)
        return None

    def tick(self,now):
        """Required health check when no scan arrives; never returns a new pose."""
        try: now=float(now)
        except (ValueError,TypeError,OverflowError): return self.reject('Invalid clock',fatal=True)
        error=self.clock_check(now)
        if error: return error
        if self.status=='LOST': return self.reject('Explicit reset required')
        return self.state_report('No new measurement')

    @staticmethod
    def scan_geometry(scan):
        frame=scan['frame_id']; count=len(scan['ranges'])
        values=[float(scan[k]) for k in ('angle_min','angle_increment','range_min','range_max')]
        if not isinstance(frame,str) or not frame or not 64<=count<=2048:
            raise ValueError('Invalid frame identity or bin count')
        if not np.isfinite(values).all() or not 0<values[1]<.2 or not 0<values[2]<values[3]:
            raise ValueError('Invalid angular/range geometry')
        if values[1]*(count-1)>2*math.pi+.01:
            raise ValueError('Scan spans more than one revolution')
        return (frame,count,*values)

    def update(self,scan,now):
        try: now=float(now)
        except (ValueError,TypeError,OverflowError): return self.reject('Invalid clock',fatal=True)
        error=self.clock_check(now)
        if error: return error
        if self.status=='LOST': return self.reject('Explicit reset required')
        # Invalid metadata on an established stream is a geometry discontinuity,
        # not a transient lack of returns. Do not keep using the old reference.
        try:
            geometry=self.scan_geometry(scan)
        except (KeyError,TypeError,ValueError,OverflowError) as error:
            return self.reject('Invalid scan geometry: '+str(error),fatal=self.geometry is not None)
        if self.geometry is not None and geometry!=self.geometry:
            return self.reject('Scan geometry or frame changed',fatal=True)
        try:
            stamp=float(scan['stamp'])
            if not math.isfinite(stamp): raise ValueError('Nonfinite stamp')
            if not 0<=now-stamp<=self.config.max_age_sec:
                return self.reject('Stale or future scan')
            if self.last_seen_stamp is not None and stamp<self.last_seen_stamp:
                return self.reject('Sensor timestamp moved backwards',fatal=True)
            if stamp==self.last_seen_stamp: return self.state_report('Duplicate stamp')
            self.last_seen_stamp=stamp
            points,valid=scan_points(scan); current=points[valid]
            if self.target is None:
                target=surface_reference(scan)
                quality=match(current,*target)
                if not quality['valid']: return self.reject('Initial reference is unobservable: '+quality['reason'])
                self.target=target; self.geometry=geometry; self.status='TRACKING'
                self.last_pose=np.zeros(3); self.last_good_stamp=stamp
                self.remember_reference()
                return dict(valid=True,status=self.status,stamp=stamp,pose=self.last_pose.tolist(),
                            keyframe_id=0,keyframe_changed=False,reason='Relative origin initialized; not map pose',fit=quality)
            result=match(current,*self.target,initial_pose=self.relative_pose,allow_bounded_plateau=True)
            result,reused=self.historical_reference(current,result)
            if not result['valid']:
                rejected=self.reject(result['reason'],keep_history_candidate=self.pending_reference_streak>0)
                rejected['fit']=result  # Evidence only; never accepted as a pose.
                return rejected
            relative=np.array([result['x_m'],result['y_m'],result['yaw_rad']])
            proposed=compose(self.keyframe_pose if reused is None else reused['pose'],relative)
            dt=stamp-self.last_good_stamp
            if dt<=0: return self.reject('Nonpositive accepted-scan interval',fatal=True)
            translation=float(np.linalg.norm(proposed[:2]-self.last_pose[:2]))
            rotation_step=abs(angle_difference(proposed[2],self.last_pose[2]))
            if translation>self.config.max_linear_speed*dt+self.config.translation_noise_m or \
               rotation_step>self.config.max_angular_speed*dt+self.config.rotation_noise_rad:
                return self.reject('Implausible pose jump; explicit reset required',fatal=True)
            # An accepted scan never renews time for any previously rejected scan.
            if reused is not None:
                self.target=reused['target']; self.keyframe_pose=reused['pose'].copy()
                self.active_reference_id=reused['id']; self.promotion_streak=0
                self.pending_reference_id=None; self.pending_reference_streak=0
            self.last_pose=proposed; self.last_good_stamp=stamp; self.relative_pose=relative
            moved=np.linalg.norm(relative[:2])>=self.config.keyframe_distance_m or abs(relative[2])>=self.config.keyframe_angle_rad
            self.promotion_streak=self.promotion_streak+1 if moved and strong_fit(result) and reused is None else 0
            changed=False; replacement_error=None
            if self.promotion_streak>=self.config.keyframe_confirmation_frames:
                try:
                    new_target=surface_reference(scan)
                    check=match(current,*new_target)
                    if not check['valid']: raise ValueError('Replacement reference is unobservable')
                    self.target=new_target; self.keyframe_pose=proposed.copy()
                    self.relative_pose=np.zeros(3); self.keyframe_id+=1; changed=True
                    self.remember_reference()
                except ValueError as error:
                    replacement_error=str(error)
                self.promotion_streak=0
            return dict(valid=True,status=self.status,stamp=stamp,pose=proposed.tolist(),
                        keyframe_id=self.keyframe_id,keyframe_changed=changed,fit=result,
                        active_reference_id=self.active_reference_id,
                        reference_reused_from=None if reused is None else reused['id'],
                        replacement_error=replacement_error,reason='Local scan odometry; not global acceptance')
        except (KeyError,TypeError,ValueError,OverflowError,np.linalg.LinAlgError) as error:
            return self.reject(str(error))
