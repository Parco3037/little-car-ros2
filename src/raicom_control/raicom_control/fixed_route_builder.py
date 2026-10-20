"""Compile surveyed map geometry into immutable stop/turn/FollowPath segments."""
import argparse
import json
import math
from pathlib import Path
import hashlib
from .fixed_route_assets import binding, fingerprint, pose
from .navigation_acceptance_logic import static_pose_collision_cells

REQUIRED_LEGS=(('start','shibie'),('shibie','pick'),('pick','pick_A'),
               ('pick_A','pick_B'),('pick','pick_B'),('pick_A','pick'),('pick_B','pick'))


def samples(a,b):
    delta=math.remainder(b[2]-a[2],2*math.pi)
    n=max(1,math.ceil(math.hypot(b[0]-a[0],b[1]-a[1])/.01),math.ceil(abs(delta)/math.radians(1)))
    return [[a[0]+(b[0]-a[0])*i/n,a[1]+(b[1]-a[1])*i/n,a[2]+delta*i/n] for i in range(n+1)]


def segment(kind,a,b):
    points=samples(a,b)
    return {'kind':kind,'controller_id':'FixedReverse' if kind=='reverse' else 'FixedForward',
            'start':list(a),'end':list(b),'path':points if kind!='turn' else [list(b)],
            'static_sweep':points,'timeout_s':30. if kind=='turn' else 120.,'stop_after':True}


def compile_leg(source,target,waypoints,pick_exit=None):
    current=list(source);result=[]
    if pick_exit is not None:
        if len(pick_exit)!=2:raise ValueError('pick_exit must be measured map x/y')
        dx,dy=pick_exit[0]-current[0],pick_exit[1]-current[1]
        along=dx*math.cos(current[2])+dy*math.sin(current[2])
        cross=-dx*math.sin(current[2])+dy*math.cos(current[2])
        if along>=-.03 or abs(cross)>.005:raise ValueError('pick exit must be behind pick on its departure heading')
        exit_pose=[*pick_exit,current[2]]
        result.append(segment('reverse',current,exit_pose));current=exit_pose
    for xy in list(waypoints)+[target[:2]]:
        if len(xy)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in xy):
            raise ValueError('Invalid map waypoint')
        distance=math.hypot(xy[0]-current[0],xy[1]-current[1])
        if distance<.001:continue
        heading=math.atan2(xy[1]-current[1],xy[0]-current[0])
        turned=[current[0],current[1],heading]
        if abs(math.remainder(heading-current[2],2*math.pi))>1e-6:
            result.append(segment('turn',current,turned))
        end=[xy[0],xy[1],heading]
        result.append(segment('forward',turned,end));current=end
    if abs(math.remainder(target[2]-current[2],2*math.pi))>1e-6:
        result.append(segment('turn',current,target))
    elif result:
        result[-1]['end']=list(target);result[-1]['path'][-1]=list(target)
    if not result:result=[segment('turn',source,target)]
    return result


def build(route,map_file,footprint,survey):
    path=Path(map_file)
    import yaml
    meta=yaml.safe_load(path.read_text(encoding='utf-8'))
    image=(path.parent/meta['image']).resolve()
    image.relative_to(path.parent.resolve())
    bind=binding(route,hashlib.sha256(path.read_bytes()).hexdigest(),hashlib.sha256(image.read_bytes()).hexdigest())
    if survey.get('binding')!=bind:raise ValueError('Survey belongs to different map/points')
    if survey.get('pick_exit') is None:raise ValueError('pick exit geometry has not been surveyed')
    if not survey.get('geometry_evidence'):raise ValueError('Missing source of surveyed geometry')
    points=footprint['points_xy_m']
    if footprint.get('padding_m')!=0. or len(points)<3:raise ValueError('Need actual footprint with zero extra padding')
    from .navigation_acceptance_logic import _pgm_pixels
    width,height,_,_=_pgm_pixels(image)
    legs={};failures=[];checked=0
    for first,last in REQUIRED_LEGS:
        key=first+'->'+last
        if not isinstance(survey.get('turning_points',{}).get(key),list):raise ValueError('Unsurveyed leg '+key)
        parts=compile_leg(pose(route['points'][first]),pose(route['points'][last]),
                          survey['turning_points'][key],survey['pick_exit'] if first=='pick' else None)
        for i,part in enumerate(parts):
            for p in part['static_sweep']:
                checked+=1
                c,s=math.cos(p[2]),math.sin(p[2])
                vertices=[(p[0]+x*c-y*s,p[1]+x*s+y*c) for x,y in points]
                inside=all(meta['origin'][0]<x<meta['origin'][0]+width*meta['resolution'] and
                           meta['origin'][1]<y<meta['origin'][1]+height*meta['resolution'] for x,y in vertices)
                blocked=static_pose_collision_cells(path,footprint,dict(zip(('x','y','yaw_rad'),p)))
                if blocked or not inside:
                    failures.append({'leg':key,'segment':i,'pose':p,'cells':blocked,'inside_map':inside})
            part.pop('static_sweep')
        legs[key]=parts
    result={'schema_version':1,'binding':bind,'footprint':footprint,
            'geometry_evidence':survey['geometry_evidence'],'site_confirmed':survey.get('site_confirmed') is True,
            'endpoints':{k:pose(v) for k,v in route['points'].items()},'pick_exit':survey['pick_exit'],
            'turning_points':survey['turning_points'],'legs':legs,
            'static_check':{'passed':not failures,'sample_count':checked,'failures':failures},
            'cruise_m_s':.25,'reverse_m_s':.08,'position_tolerance_m':.02,'yaw_tolerance_deg':10.}
    result['content_sha256']=fingerprint(result)
    return result


def validate_bundle(bundle,*,require_site=True):
    copy=dict(bundle);sha=copy.pop('content_sha256',None)
    if sha!=fingerprint(copy):raise ValueError('Fixed route content fingerprint mismatch')
    if bundle.get('static_check',{}).get('passed') is not True:raise ValueError('Static geometry did not pass')
    if require_site and bundle.get('site_confirmed') is not True:raise ValueError('Lab geometry not confirmed onsite')
    if (bundle.get('cruise_m_s')!=.25 or bundle.get('reverse_m_s')!=.08 or
            bundle.get('position_tolerance_m')!=.02 or bundle.get('yaw_tolerance_deg')!=10.):
        raise ValueError('Baseline speed/tolerances changed')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for arg in ('route','map','footprint','survey','output'):p.add_argument('--'+arg,required=True,type=Path)
    a=p.parse_args()
    result=build(json.loads(a.route.read_text(encoding='utf-8')),a.map,
                 json.loads(a.footprint.read_text(encoding='utf-8')),json.loads(a.survey.read_text(encoding='utf-8')))
    with a.output.open('x',encoding='utf-8') as stream:json.dump(result,stream,ensure_ascii=False,indent=2)
    print(json.dumps({'saved':str(a.output),'static_pass':result['static_check']['passed'],'site_confirmed':result['site_confirmed']}))
    if not result['static_check']['passed']:raise SystemExit(2)


if __name__=='__main__':main()
