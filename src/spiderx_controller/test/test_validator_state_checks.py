"""colcon test: the runtime validators accept only a controller state that is exactly 'active' (M4.1).

Runs each validator's own controller-state check (the `ctrls=$(ros2 control list_controllers ...)`
line and the loop after it) in bash. `ros2` is replaced by a shell function that prints a recorded
`ros2 control list_controllers` listing, so no ROS graph or simulator is needed.
"""
import os
import subprocess

import pytest

SCRIPTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'scripts')
VALIDATORS = ['validate_m1_control.sh', 'validate_m2_posture.sh', 'validate_m3_kinematics.sh',
              'validate_m4_all_leg_ik.sh']
JSB, LTC = 'joint_state_broadcaster', 'leg_trajectory_controller'
TYPES = {JSB: 'joint_state_broadcaster/JointStateBroadcaster',
         LTC: 'joint_trajectory_controller/JointTrajectoryController'}
COLOURS = {'active': '\033[92m', 'inactive': '\033[96m', 'unconfigured': '\033[93m'}
END = '\033[0m'
# 'inactive', values starting with 'inactive', and other values with characters in front of 'active'
NOT_ACTIVE = ['inactive', 'inactive_pending', 'unconfigured', 'finalized', 'reactive', 'xactive',
              'notactive']


def _listing(states):
    """`ros2 control list_controllers` output for {controller: state}, colours included.

    Same layout as ros2controlcli's print_controller_state (verb/list_controllers.py).
    """
    w_name = max(len(n) for n in states)
    w_type = max(len(TYPES[n]) for n in states)
    w_state = max(len(s) for s in states.values())
    lines = []
    for name, state in states.items():
        colour = COLOURS.get(state, '')
        lines.append(f'{colour}{name:<{w_name}}{END} {TYPES[name]:<{w_type}}  '
                     f'{colour}{state:<{w_state}}{END}')
    return '\n'.join(lines) + '\n'


def _check_block(script):
    """The validator's `ctrls=$(ros2 control list_controllers ...)` line and the loop after it."""
    with open(os.path.join(SCRIPTS, script)) as f:
        lines = f.read().splitlines()
    start = next(i for i, line in enumerate(lines)
                 if line.strip().startswith('ctrls=$(ros2 control list_controllers'))
    end = next(i for i in range(start, len(lines)) if lines[i].strip() == 'done')
    return '\n'.join(lines[start:end + 1])


def _reported(script, states):
    """Run the validator's own check against a recorded listing: {controller: reported active}."""
    program = '\n'.join([
        'pass() { printf "PASS %s\\n" "$1"; }',
        'bad() { printf "FAIL %s\\n" "$1"; }',
        'ros2() { printf "%s" "$LISTING"; }',  # stands in for `ros2 control list_controllers`
        _check_block(script)])
    result = subprocess.run(['bash', '-c', program], capture_output=True, text=True, timeout=60,
                            env={**os.environ, 'LISTING': _listing(states)})
    assert result.returncode == 0, result.stderr
    reported = {}
    for line in result.stdout.splitlines():
        verdict, controller = line.split()[:2]
        reported[controller] = verdict == 'PASS'
    return reported


@pytest.mark.parametrize('script', VALIDATORS)
def test_the_check_covers_both_controllers(script):
    assert 'for c in joint_state_broadcaster leg_trajectory_controller; do' in _check_block(script)


@pytest.mark.parametrize('script', VALIDATORS)
def test_both_active_pass(script):
    assert _reported(script, {JSB: 'active', LTC: 'active'}) == {JSB: True, LTC: True}


@pytest.mark.parametrize('script', VALIDATORS)
def test_the_start_up_race_state_is_not_reported_as_active(script):
    """joint_state_broadcaster inactive while leg_trajectory_controller is active."""
    assert _reported(script, {JSB: 'inactive', LTC: 'active'}) == {JSB: False, LTC: True}


@pytest.mark.parametrize('script', VALIDATORS)
def test_an_inactive_trajectory_controller_is_not_reported_as_active(script):
    assert _reported(script, {JSB: 'active', LTC: 'inactive'}) == {JSB: True, LTC: False}


@pytest.mark.parametrize('script', VALIDATORS)
@pytest.mark.parametrize('state', NOT_ACTIVE)
def test_a_state_other_than_active_never_passes(script, state):
    assert _reported(script, {JSB: state, LTC: state}) == {JSB: False, LTC: False}
