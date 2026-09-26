"""colcon test: controller.launch.py timeouts, and the second spawner runs only after success."""
import importlib.util
import os
import sys
from unittest import mock

from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import ExecuteProcess
from launch.substitutions import TextSubstitution
from launch.utilities import perform_substitutions

LAUNCH_FILE = os.path.join(os.path.dirname(__file__), '..', 'launch', 'controller.launch.py')


def _load():
    spec = importlib.util.spec_from_file_location('controller_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_spawners_use_generous_timeouts():
    mod = _load()
    assert float(mod.SERVICE_CALL_TIMEOUT_S) > float(mod.SWITCH_TIMEOUT_S) >= 30
    for controller in ('joint_state_broadcaster', 'leg_trajectory_controller'):
        cmd = mod._spawner(controller).cmd
        args = [perform_substitutions(LaunchContext(), part) for part in cmd[1:]
                if all(isinstance(s, TextSubstitution) for s in part)]
        assert args[0] == controller
        assert args[args.index('--switch-timeout') + 1] == mod.SWITCH_TIMEOUT_S
        assert args[args.index('--service-call-timeout') + 1] == mod.SERVICE_CALL_TIMEOUT_S


def _run(first_returncode, tmp_path):
    """Stand-in spawners: 'first' exits with the given code, 'second' creates a marker file.

    Returns (second spawner started, error messages logged by the handler).
    """
    mod = _load()
    marker = tmp_path / 'second_started'
    first = ExecuteProcess(cmd=[sys.executable, '-c', f'raise SystemExit({first_returncode})'])
    second = ExecuteProcess(cmd=[sys.executable, '-c', f'open({str(marker)!r}, "w").close()'])
    ls = LaunchService()
    ls.include_launch_description(LaunchDescription([
        first,
        mod._on_success(first, 'first', then=[second]),
        mod._on_success(second, 'second'),
    ]))
    logger = mock.Mock()
    with mock.patch.object(mod, 'get_logger', return_value=logger):
        ls.run()
    return marker.exists(), [c.args[0] for c in logger.error.call_args_list]


def test_second_spawner_starts_after_success(tmp_path):
    started, errors = _run(0, tmp_path)
    assert started
    assert errors == []


def test_failure_stops_the_chain_and_logs_an_error(tmp_path):
    started, errors = _run(1, tmp_path)
    assert not started
    assert len(errors) == 1
    assert 'first failed (exit code 1)' in errors[0] and 'NOT started' in errors[0]
