"""Offline fixed-reference point-to-line experiment, NOT a ROS odometry source.

Fits each scan independently to a reference; does not accumulate frame-to-frame
increments or force zero motion. Degenerate geometry and poor matches reject.
No sensor correction, TF, navigation, bias update, or actuator publication.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import tarfile

import numpy as np


def rotation(yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-s],[s,c]])


def scan_points(scan):
    values=np.array([np.nan if v is None else v for v in scan['ranges']],dtype=float)
    angles=scan['angle_min']+np.arange(len(values))*scan['angle_increment']
    valid=np.isfinite(values)&(values>=scan['range_min'])&(values<=scan['range_max'])
    return np.column_stack((values*np.cos(angles),values*np.sin(angles))),valid


def reference(scans):
    if len(scans)<8:
        raise ValueError('Need at least eight reference scans')
    geometry=('angle_min','angle_increment','range_min','range_max')
    if any(len(s['ranges'])!=len(scans[0]['ranges']) or
           any(s[k]!=scans[0][k] for k in geometry) for s in scans):
        raise ValueError('Reference scan geometry changed')
    matrix=np.array([[np.nan if v is None else v for v in s['ranges']] for s in scans])
    # Empty columns are never interpolated or filled with invented returns.
    medians=[float(np.median(column[np.isfinite(column)])) if np.isfinite(column).sum()>=len(scans)*.7
             else None for column in matrix.T]
    return surface_reference(dict(scans[0],ranges=medians))


def surface_reference(scan):
    """Extract local surfaces from one actual scan; do not fill missing returns."""
    points,valid=scan_points(scan)
    indices=np.arange(1,len(points)-1)
    valid_triplet=valid[:-2]&valid[1:-1]&valid[2:]
    indices=indices[valid_triplet]
    left=points[indices]-points[indices-1]; right=points[indices+1]-points[indices]
    left_length=np.linalg.norm(left,axis=1); right_length=np.linalg.norm(right,axis=1)
    smooth=(left_length>.001)&(right_length>.001)&(left_length<.12)&(right_length<.12)
    # Reject corners/occlusion edges rather than inventing a surface normal.
    cosine=np.sum(left*right,axis=1)/np.maximum(left_length*right_length,1e-12)
    indices=indices[smooth&(cosine>.5)]
    tangent=points[indices+1]-points[indices-1]
    length=np.linalg.norm(tangent,axis=1)
    if len(indices)<30 or np.any(length<1e-6):
        raise ValueError('Insufficient smooth reference geometry')
    normals=np.column_stack((-tangent[:,1],tangent[:,0]))/length[:,None]
    return points[indices],normals


def bounded_iteration_plateau(history):
    """Numerical stopping rule, not a claim of submillimeter sensor accuracy.

    Hard correspondence/trimming changes can cycle between nearby solutions.
    Require six iterates to stay within 0.25mm / 0.0005rad, with residual RMS
    spread below 0.1mm. The independent final overlap/residual gates still apply.
    """
    if len(history)<6: return False
    values=np.asarray(history[-6:],dtype=float)
    if values.shape!=(6,4) or not np.isfinite(values).all(): return False
    distances=np.linalg.norm(values[:,None,:2]-values[None,:,:2],axis=2)
    return bool(distances.max()<=.00025 and np.ptp(values[:,2])<=.0005 and np.ptp(values[:,3])<=.0001)


def nearest_reference(points, targets):
    """Exact brute-force 2D distance, without an NxMx2 temporary/reduction.

    Keep squared x plus squared y in the original order and argmin's first-index
    tie rule. This is not approximate search or a change to any acceptance gate.
    Inputs have already been validated by match; neither input is mutated.
    """
    distance2 = np.subtract.outer(points[:, 0], targets[:, 0])
    dy = np.subtract.outer(points[:, 1], targets[:, 1])
    np.square(distance2, out=distance2)
    np.square(dy, out=dy)
    distance2 += dy
    nearest = distance2.argmin(axis=1)
    return nearest, np.sqrt(distance2[np.arange(len(points)), nearest])


def match(points, targets, normals, initial_pose=None, trace=None, allow_bounded_plateau=False):
    points,targets,normals=(np.asarray(v,dtype=float) for v in (points,targets,normals))
    if any(v.ndim!=2 or v.shape[1]!=2 or not np.isfinite(v).all() for v in (points,targets,normals)):
        raise ValueError('Malformed/nonfinite point geometry')
    if len(points)<30 or len(targets)<30 or targets.shape!=normals.shape:
        raise ValueError('Insufficient point geometry')
    if not np.allclose(np.linalg.norm(normals,axis=1),1,atol=1e-6):
        raise ValueError('Reference normals must be unit vectors')
    pose=np.zeros(3) if initial_pose is None else np.asarray(initial_pose,dtype=float).copy()
    if pose.shape!=(3,) or not np.isfinite(pose).all():
        raise ValueError('Invalid initial pose')
    if np.linalg.norm(pose[:2])>.30 or abs(pose[2])>.30:
        return dict(valid=False,reason='Initial pose outside bounded local reference domain')
    condition=None; converged=False; convergence_mode=None; history=[]
    for iteration in range(40):
        transformed=points@rotation(pose[2]).T+pose[:2]
        nearest,distances=nearest_reference(transformed,targets)
        n=normals[nearest]
        residual=np.sum(n*(transformed-targets[nearest]),axis=1)
        spatial=distances<=.12
        if spatial.sum()<30:
            return dict(valid=False,reason='Too few nearby reference correspondences')
        keep=spatial&(np.abs(residual)<=max(.008,np.quantile(np.abs(residual[spatial]),.8)))
        if keep.sum()<30 or keep.mean()<.45:
            return dict(valid=False,reason='Insufficient inlier overlap')
        q=transformed[keep]; a=n[keep]; r=residual[keep]
        jacobian=np.column_stack((a,-a[:,0]*q[:,1]+a[:,1]*q[:,0]))
        multiplicity=np.bincount(nearest[keep],minlength=len(targets))[nearest[keep]]
        weights=np.minimum(1.,.02/np.maximum(np.abs(r),1e-12))/multiplicity
        hessian=jacobian.T@(weights[:,None]*jacobian)
        condition=float(np.linalg.cond(hessian))
        if not math.isfinite(condition) or condition>10000:
            return dict(valid=False,reason='Unobservable/degenerate reference geometry')
        delta=np.linalg.solve(hessian,-jacobian.T@(weights*r))
        # Damped local steps handle the rigid transform's nonlinearity. The
        # total-pose domain below is still checked; no final pose is clipped.
        scale=min(1.,.05/max(float(np.linalg.norm(delta[:2])),1e-12),
                  .05/max(float(abs(delta[2])),1e-12))
        delta*=scale
        pose[:2]=rotation(delta[2])@pose[:2]+delta[:2]; pose[2]+=delta[2]
        history.append([*pose,float(np.sqrt(np.mean(r*r)))])
        history=history[-6:]
        if trace is not None:
            trace.append(dict(iteration=iteration,pose=pose.tolist(),delta=delta.tolist(),
                              residual_rms=float(np.sqrt(np.mean(r*r))),inliers=int(keep.sum())))
        if np.linalg.norm(pose[:2])>.30 or abs(pose[2])>.30:
            return dict(valid=False,reason='Outside bounded local reference domain')
        if np.linalg.norm(delta)<1e-6:
            converged=True; convergence_mode='strict_step'; break
        if allow_bounded_plateau and bounded_iteration_plateau(history):
            converged=True; convergence_mode='bounded_iteration_plateau'; break
    transformed=points@rotation(pose[2]).T+pose[:2]
    nearest,distances=nearest_reference(transformed,targets)
    residual=np.sum(normals[nearest]*(transformed-targets[nearest]),axis=1)
    inlier=(distances<=.12)&(np.abs(residual)<=.03)
    rms=float(np.sqrt(np.mean(residual[inlier]**2))) if inlier.any() else None
    valid=bool(converged and inlier.sum()>=30 and inlier.mean()>=.5 and rms is not None and rms<=.02)
    return dict(valid=valid,reason='Local fit only, not physical acceptance' if valid else 'Convergence or residual gate failed',
        converged=converged,convergence_mode=convergence_mode,iterations=iteration+1,x_m=float(pose[0]),y_m=float(pose[1]),
        yaw_rad=float(pose[2]),yaw_deg=float(np.degrees(pose[2])),points=len(points),
        inliers=int(inlier.sum()),inlier_fraction=float(inlier.mean()),inlier_plane_rms_m=rms,
        all_point_nearest_p90_m=float(np.percentile(distances,90)),hessian_condition=condition)


def analyze(data):
    scans=data['scans']
    training=[s for s in scans if s['t']<10]
    points,normals=reference(training)
    output=dict(scope='Offline fixed-reference experiment; NOT live odometry or physical stationarity acceptance',
        zero_commands=bool(data.get('speeds') and all(all(v==0 for v in row[1:]) for row in data['speeds'])),
        reference_frames=len(training),reference_points=len(points),frames=[],production_modified=False)
    base=scans[0]
    for scan in scans:
        if scan['t']<30:
            continue
        if len(scan['ranges'])!=len(base['ranges']) or any(scan[k]!=base[k] for k in
                ('angle_min','angle_increment','range_min','range_max')):
            result=dict(valid=False,reason='Input scan geometry changed')
        else:
            current,valid=scan_points(scan)
            result=match(current[valid],points,normals)
        output['frames'].append(dict(t=scan['t'],**result))
    accepted=[f for f in output['frames'] if f['valid']]
    output['accepted_frames']=len(accepted); output['total_frames']=len(output['frames'])
    if accepted:
        output['accepted_only_metrics']=dict(max_abs_yaw_deg=max(abs(f['yaw_deg']) for f in accepted),
            yaw_span_deg=max(f['yaw_deg'] for f in accepted)-min(f['yaw_deg'] for f in accepted),
            max_xy_m=max(math.hypot(f['x_m'],f['y_m']) for f in accepted))
    output['all_frames_fit']=bool(output['frames'] and len(accepted)==len(output['frames']))
    return output


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('archive',type=Path)
    parser.add_argument('--member',default='warmed_capture_180s.json')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    with tarfile.open(args.archive) as source:
        raw=source.extractfile(args.member).read()
    result=analyze(json.loads(raw))
    result['source_member']=args.member; result['source_sha256']=hashlib.sha256(raw).hexdigest()
    with args.output.open('x',encoding='utf-8') as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in result.items() if k!='frames'},indent=2))


if __name__=='__main__':
    main()
