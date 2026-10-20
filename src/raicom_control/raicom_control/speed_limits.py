"""Software ceilings, distinct from configured speeds and physical acceptance."""
import math

LINEAR_CEILING = 2.0
ANGULAR_CEILING = 2.0
GUARD_SPEED_CONTRACT = 'raicom_guard_speed/v1'


def validate_guard_limits(linear, angular):
    for name, value, ceiling in [('max_linear',linear,LINEAR_CEILING),
                                  ('max_angular',angular,ANGULAR_CEILING)]:
        if type(value) not in (int,float) or not math.isfinite(value) or not 0<value<=ceiling:
            raise ValueError(f'{name} must be finite and in (0, {ceiling}]')


def guard_targets(config):
    # Keep the existing .25/.50 manual keyboard usable after navigation. The
    # controller and smoother enforce lower selected navigation speeds.
    return [max(.50,linear_target(config)),max(.50,float(config['turn']))]


def linear_target(config):
    return max(float(config.get(key,default)) for key,default in [('far',.25),('near',.12),('fine',.03)])
