from types import SimpleNamespace as NS
from test_guard_speed_offline import guard_fixture
from raicom_control.guard_logic import ZERO_COMMAND


def test_diagnostics_separate_missing_lease_from_missing_command_without_enabling_motion():
    g, _, clock, _, enable, twist = guard_fixture()
    assert g.diagnostic_snapshot(clock[0], ZERO_COMMAND)['reason'] == 'lease_absent'
    enable()
    assert g.diagnostic_snapshot(clock[0], ZERO_COMMAND)['reason'] == 'input_absent'
    message = twist(); message.linear.x = .1
    g._on_command(message)
    clock[0] += .05
    g._on_command(message)
    state = g.diagnostic_snapshot(clock[0], g.state.command_at(clock[0]))
    assert state['reason'] == 'passing' and state['input_count'] == 2
    clock[0] += .2
    enable()
    assert g.diagnostic_snapshot(clock[0], ZERO_COMMAND)['reason'] == 'input_expired'
    g._on_stop(None)
    assert g.diagnostic_snapshot(clock[0], ZERO_COMMAND)['reason'] == 'stop_latched'
    assert g.state.command_at(clock[0]).is_zero
