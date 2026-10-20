"""No live speed change: decide whether two matched physical trials qualify."""
import math


def valid_run(run,speed):
    if (run.get('physical_run') is not True or run.get('cruise_m_s')!=speed
            or run.get('controller_limit_m_s')!=speed or run.get('smoother_limit_m_s')!=speed
            or run.get('reverse_limit_m_s')!=.08 or not run.get('route_sha256')
            or not run.get('transport_state_sha256') or not run.get('segments')):return False
    if type(run.get('elapsed_s')) not in (int,float) or not math.isfinite(run['elapsed_s']) or run['elapsed_s']<=0:return False
    for segment in run['segments']:
        if not (segment.get('nav_status')==4 and segment.get('stopped') is True
                and type(segment.get('position_error_m')) in (int,float) and 0<=segment['position_error_m']<=.02
                and type(segment.get('yaw_error_deg')) in (int,float) and 0<=segment['yaw_error_deg']<=10.):return False
    return True


def speed_decision(baseline,trials):
    accepted=valid_run(baseline,.25) and len(trials)==2
    previous=baseline.get('run_sequence')
    for trial in trials:
        accepted=accepted and valid_run(trial,.35)
        accepted=accepted and all(trial.get(k)==baseline.get(k) for k in ('route_sha256','transport_state_sha256'))
        accepted=accepted and [s.get('name') for s in trial.get('segments',[])]==[s.get('name') for s in baseline.get('segments',[])]
        accepted=accepted and type(trial.get('run_sequence')) is int and type(previous) is int and trial['run_sequence']==previous+1
        accepted=accepted and trial.get('elapsed_s',float('inf'))<baseline.get('elapsed_s',0)
        previous=trial.get('run_sequence')
    return {'retain_035':bool(accepted),'selected_cruise_m_s':.35 if accepted else .25}
