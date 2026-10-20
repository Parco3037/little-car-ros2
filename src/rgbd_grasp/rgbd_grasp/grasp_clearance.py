"""Cartesian sag allowance and checked approach geometry. No hardware access."""
import math
import numpy as np
from .grasp_core import Rejected, finite_vector, interpolate, validate_commanded_fk


def allowance(value, maximum, name):
    if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= maximum:
        raise Rejected(f'{name} must be between 0 and {maximum} m')
    return float(value)


def raised_side_path(target, offset, approach_m, step_m, sag_m, clearance_m):
    """Keep a radial side grasp; add height before IK, not guessed joint pulses."""
    sag_m = allowance(sag_m, .040, 'sag_compensation_m')
    clearance_m = allowance(clearance_m, .030, 'pregrasp_clearance_m')
    grasp = finite_vector(target, 3, 'target') + finite_vector(offset, 3, 'offset')
    grasp[2] += sag_m
    radius = float(np.linalg.norm(grasp[:2]))
    if radius < .05 or not math.isfinite(approach_m) or not .02 <= approach_m <= .10:
        raise Rejected('invalid radial approach')
    pregrasp = grasp.copy()
    pregrasp[:2] -= grasp[:2] / radius * approach_m
    pregrasp[2] += clearance_m
    # Retain the added clearance through the first half of the forward motion,
    # then descend smoothly to the compensated grasp height, never below it.
    corner = grasp.copy()
    corner[:2] -= grasp[:2] / radius * approach_m * .5
    corner[2] += clearance_m
    first_half = interpolate(pregrasp, corner, step_m)
    radial_distance = float(np.linalg.norm(grasp[:2] - corner[:2]))
    # Smoothstep starts and ends the descent with zero vertical slope.
    count = max(1, int(math.ceil(math.hypot(radial_distance,
                                          1.5 * clearance_m) / step_m)))
    descent = []
    for index in range(1, count + 1):
        fraction = index / count
        smooth = fraction * fraction * (3.0 - 2.0 * fraction)
        point = corner + (grasp - corner) * fraction
        point[2] = grasp[2] + clearance_m * (1.0 - smooth)
        descent.append(point)
    return pregrasp, first_half + descent, grasp


def joint_ramp(start, end, step, fixed_id5):
    start = finite_vector(start, 5, 'ramp start')
    end = finite_vector(end, 5, 'ramp end')
    if not math.isfinite(step) or not 0 < step <= 60:
        raise Rejected('invalid joint ramp step')
    count = max(1, int(math.ceil(np.max(np.abs(end[:4]-start[:4])) / step)))
    result=[]
    for i in range(1, count+1):
        point=np.rint(start+(end-start)*i/count).astype(int)
        point[4]=int(fixed_id5)
        result.append(point)
    return result


def validate_ramp_height(start, end, fk, workspace, fixed_id5):
    """Inspect the entire joint interpolation, including between command points."""
    start_z = float(fk(start)[2]); end_z = float(fk(end)[2])
    floor = min(start_z, end_z) - .002  # Only model interpolation/rounding tolerance.
    minimum = min(start_z, end_z)
    for point in joint_ramp(start, end, 2, fixed_id5):
        xyz = validate_commanded_fk(fk(point), workspace)
        minimum=min(minimum,float(xyz[2]))
        if xyz[2] < floor:
            raise Rejected('pregrasp joint interpolation dips below its endpoint height')
    return minimum


def checked_height(commanded, measured, sag_m):
    """Reject a measured drop that consumes the explicit sag allowance.

    FK reflects measured joint angles, not finger flex or contact force. This is
    a downward tracking guard, not a table detector or physical grasp proof.
    """
    goal=finite_vector(commanded,3,'commanded FK')
    actual=finite_vector(measured,3,'measured FK')
    budget=allowance(sag_m,.040,'sag_compensation_m')+.005
    drop=float(goal[2]-actual[2])
    if drop > budget:
        raise Rejected(f'arm dropped {drop*1000:.1f} mm below command; allowance {budget*1000:.1f} mm; forward motion stopped')
    return drop
