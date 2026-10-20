"""Pure fail-closed policy shared by the fusion runtime and its profiler."""
import argparse
from pathlib import Path


def build_argument_parser(*, add_help=True):
    parser = argparse.ArgumentParser(add_help=add_help)
    parser.add_argument('--directory', type=Path, required=True)
    parser.add_argument('--session', required=True)
    parser.add_argument('--parent-pid', type=int, required=True)
    parser.add_argument('--parent-start', required=True)
    parser.add_argument('--old-pid', type=int, required=True)
    parser.add_argument('--old-start', required=True)
    parser.add_argument('--main-cpu', type=int, required=True,
                        help='Assigned CPU for the scan-matching supervisor thread')
    parser.add_argument('--child-cpu', type=int, required=True,
                        help='Different CPU for the private robot_localization EKF')
    parser.add_argument('--reset-origin', action='store_true',
                        help='Explicit recovery only; never valid for STANDBY profiling')
    parser.add_argument('--standby-only', action='store_true',
                        help='Forbid activation and production odometry handoff')
    return parser


def validate_arguments(args, parser):
    if not 16 <= len(args.session) <= 100 or args.old_pid <= 1 or args.parent_pid <= 1:
        parser.error('Explicit session and process identities required')
    if args.main_cpu < 0 or args.child_cpu < 0 or args.main_cpu == args.child_cpu:
        parser.error('Distinct nonnegative main/child CPUs required')
    return args


def derive_affinity_roles(cpu_count, main_cpu, child_cpu):
    """Reserve separate CPUs for DDS/input and bounded diagnostic readers."""
    if cpu_count is None or cpu_count <= max(main_cpu, child_cpu):
        raise ValueError('Requested CPU is outside the host CPU range')
    spare = [cpu for cpu in range(cpu_count) if cpu not in (main_cpu, child_cpu)]
    if len(spare) < 2:
        raise ValueError('Four CPUs are required for main, EKF, DDS/input and diagnostics')
    return spare[0], spare[1]


def enforce_standby_contract(*, standby_only, directory, observed_old_start,
                             expected_old_start):
    """Fail closed before any STANDBY-only diagnostic can approach handoff."""
    if not standby_only:
        return
    if (directory/'activate.json').exists():
        raise RuntimeError('Activation token forbidden in STANDBY-only mode')
    if observed_old_start != expected_old_start:
        raise RuntimeError('Observed old odometry process changed in STANDBY-only mode')


def activation_requested(*, standby_only, mode, ready_elapsed, token_path):
    """The sole policy predicate guarding production publisher creation."""
    return (not standby_only and mode == 'STANDBY'
            and ready_elapsed > 15 and token_path.exists())


def initialization_timed_out(*, ready_elapsed, gate_state):
    """Only a still-uninitialized snapshot may trip the five-second limit.

    PAUSED proves the gate was previously established and has already revoked
    its motion lease.  It must never be mistaken for startup merely because a
    concurrent evaluation temporarily changes an internal implementation bit.
    LOST is handled by the runtime's preceding fatal-state branch.
    """
    return float(ready_elapsed) > 5.0 and gate_state == 'WAITING'
