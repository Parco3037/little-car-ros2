"""Pure safety decisions used by the home mission ROS node."""


def validate_mission_configuration(
    *,
    goal_configured: bool,
    arrival_action_enabled: bool,
    gripper_calibrated: bool,
    gripper_open_pulse: float,
    gripper_close_pulse: float,
    gripper_max_delta_pulse: float,
) -> str | None:
    if not goal_configured:
        return 'home_pick_station is not configured'
    if not arrival_action_enabled:
        return None
    if not gripper_calibrated:
        return 'gripper is not calibrated'
    if not (
        0.0 <= gripper_open_pulse <= 1000.0
        and 0.0 <= gripper_close_pulse <= 1000.0
    ):
        return 'gripper pulses must be within 0..1000'
    if (
        gripper_open_pulse == gripper_close_pulse
        or abs(gripper_open_pulse - gripper_close_pulse) > gripper_max_delta_pulse
    ):
        return 'gripper pulse delta is zero or exceeds the configured safety limit'
    return None


def requires_joint_state(arrival_action_enabled: bool) -> bool:
    return arrival_action_enabled


def should_run_arrival_action(arrival_action_enabled: bool) -> bool:
    return arrival_action_enabled
