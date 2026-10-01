"""colcon test: controller.launch.py start-up ordering (M4.1). No Gazebo or controller manager.

leg_trajectory_controller may start only after the joint_state_broadcaster spawner exited with
code 0: never after a failed or killed spawner, and never during a launch shutdown (Ctrl+C).
"""
import importlib.util
import os
import subprocess
import sys
from unittest import mock

from ament_index_python.packages import get_package_prefix, PackageNotFoundError
from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.events.process import ProcessExited
from launch.substitutions import TextSubstitution
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import pytest

LAUNCH_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'launch',
                           'controller.launch.py')
JSB, LTC = 'joint_state_broadcaster', 'leg_trajectory_controller'
JSB_FAILED = ('joint_state_broadcaster startup failed; leg_trajectory_controller was not started; '
              'press Ctrl+C and relaunch.')
LTC_FAILED = 'leg_trajectory_controller startup failed; press Ctrl+C and relaunch.'
FAILED_EXITS = [1, 2, -15, -9]  # error exits, and killed by SIGTERM / SIGKILL


def _load():
    spec = importlib.util.spec_from_file_location('spiderx_controller_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _arguments(node):
    """The spawner command-line arguments: everything between the executable and --ros-args."""
    args = []
    for part in node.cmd[1:]:
        if not all(isinstance(s, TextSubstitution) for s in part):
            break
        text = perform_substitutions(LaunchContext(), part)
        if text == '--ros-args':
            break
        args.append(text)
    return args


def _exit(launch, spawner, returncode, shutdown=False):
    """Deliver one ProcessExited event of `spawner` to its exit handler, as launch would.

    Returns (actions the handler asked launch to run, errors the handler logged).
    """
    event = ProcessExited(action=spawner, name='spawner', cmd=['spawner'], cwd=None, env=None,
                          pid=1, returncode=returncode)
    matching = [h for h in launch['handlers'] if h.matches(event)]
    assert len(matching) == 1, 'expected exactly one exit handler per spawner'
    context = LaunchContext()
    context._set_is_shutdown(shutdown)
    logger = mock.Mock()
    with mock.patch.object(launch['module'], 'get_logger', return_value=logger):
        actions = matching[0].handle(event, context)
    return list(actions or []), [c.args[0] for c in logger.error.call_args_list]


@pytest.fixture
def launch():
    """The real launch description: its module, direct spawners, exit handlers and spawners."""
    module = _load()
    entities = module.generate_launch_description().entities
    result = {'module': module,
              'direct': [e for e in entities if isinstance(e, Node)],
              'handlers': [e.event_handler for e in entities if isinstance(e, RegisterEventHandler)]}
    jsb = result['direct'][0]
    chained, _ = _exit(result, jsb, 0)  # the trajectory spawner is only reachable through here
    result['spawners'] = {JSB: jsb, LTC: chained[0] if chained else None}
    return result


# ------------------------------------------------------------ spawner command lines
@pytest.mark.parametrize('controller', [JSB, LTC])
def test_exact_spawner_arguments(controller):
    node = _load()._spawner(controller)
    assert (node.node_package, node.node_executable) == ('controller_manager', 'spawner')
    assert _arguments(node) == [controller,
                                '--controller-manager', '/controller_manager',
                                '--controller-manager-timeout', '120',
                                '--switch-timeout', '60',
                                '--service-call-timeout', '75']


def test_service_call_timeout_exceeds_switch_timeout():
    """Otherwise the spawner re-sends a pending switch request, which fails as 'already active'."""
    module = _load()
    assert float(module.SERVICE_CALL_TIMEOUT_S) > float(module.SWITCH_TIMEOUT_S)


def test_installed_spawner_supports_these_options():
    try:
        prefix = get_package_prefix('controller_manager')
    except PackageNotFoundError:
        pytest.skip('controller_manager is not installed')
    spawner = os.path.join(prefix, 'lib', 'controller_manager', 'spawner')
    result = subprocess.run([spawner, '--help'], capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stderr
    for option in ('--controller-manager-timeout', '--switch-timeout', '--service-call-timeout'):
        assert option in result.stdout


# ------------------------------------------------------------ the real launch description
def test_only_the_broadcaster_spawner_is_started_directly(launch):
    assert [_arguments(s)[0] for s in launch['direct']] == [JSB]
    assert len(launch['handlers']) == 2
    assert all(isinstance(h, OnProcessExit) for h in launch['handlers'])


def test_broadcaster_exit_0_starts_the_trajectory_spawner(launch):
    actions, errors = _exit(launch, launch['spawners'][JSB], 0)
    assert [_arguments(a)[0] for a in actions] == [LTC]
    assert errors == []


@pytest.mark.parametrize('returncode', FAILED_EXITS)
def test_broadcaster_failure_never_starts_the_trajectory_spawner(launch, returncode):
    actions, errors = _exit(launch, launch['spawners'][JSB], returncode)
    assert actions == []
    assert len(errors) == 1  # exactly one clear error entry
    assert errors[0].startswith(JSB_FAILED)
    assert f'(spawner exit code {returncode};' in errors[0]


@pytest.mark.parametrize('controller', [JSB, LTC])
@pytest.mark.parametrize('returncode', [0, 1, -2, -15])
def test_nothing_starts_and_nothing_is_logged_during_shutdown(launch, controller, returncode):
    """Ctrl+C: an interrupted spawner exits 0 (or dies by a signal); no successor, no error."""
    assert _exit(launch, launch['spawners'][controller], returncode, shutdown=True) == ([], [])


def test_trajectory_spawner_exit_0_starts_nothing_else(launch):
    assert _exit(launch, launch['spawners'][LTC], 0) == ([], [])


@pytest.mark.parametrize('returncode', FAILED_EXITS)
def test_trajectory_spawner_failure_is_reported_once(launch, returncode):
    actions, errors = _exit(launch, launch['spawners'][LTC], returncode)
    assert actions == []
    assert len(errors) == 1 and errors[0].startswith(LTC_FAILED)


# ------------------------------------------------------------ real LaunchService, real processes
def _run_chain(tmp_path, first_code, shutdown_after_s=None):
    """Run `first` -> gate -> `second` with stand-in processes in a real LaunchService.

    Returns (second process started, errors logged by the gate).
    """
    module = _load()
    marker = tmp_path / 'second_started'
    first = ExecuteProcess(cmd=[sys.executable, '-c', first_code])
    second = ExecuteProcess(cmd=[sys.executable, '-c', f'open({str(marker)!r}, "w").close()'])
    entities = [first, RegisterEventHandler(OnProcessExit(
        target_action=first, on_exit=module._start_after_success(JSB, second, LTC)))]
    if shutdown_after_s is not None:  # like Ctrl+C: shut the launch down while `first` runs
        entities.append(TimerAction(period=shutdown_after_s,
                                    actions=[EmitEvent(event=Shutdown(reason='test Ctrl+C'))]))
    service = LaunchService()
    service.include_launch_description(LaunchDescription(entities))
    logger = mock.Mock()
    with mock.patch.object(module, 'get_logger', return_value=logger):
        service.run()
    return marker.exists(), [c.args[0] for c in logger.error.call_args_list]


def test_process_exit_0_starts_the_next_process(tmp_path):
    assert _run_chain(tmp_path, 'raise SystemExit(0)') == (True, [])


@pytest.mark.parametrize('first_code,returncode', [
    ('raise SystemExit(1)', 1),
    ('raise SystemExit(2)', 2),
    ('import os, signal; os.kill(os.getpid(), signal.SIGTERM)', -15),
])
def test_failed_process_does_not_start_the_next_process(tmp_path, first_code, returncode):
    started, errors = _run_chain(tmp_path, first_code)
    assert not started
    assert len(errors) == 1 and errors[0].startswith(JSB_FAILED)
    assert f'(spawner exit code {returncode};' in errors[0]


def test_shutdown_while_running_does_not_start_the_next_process(tmp_path):
    """The stand-in exits 0 on SIGINT, like a spawner interrupted by Ctrl+C."""
    code = ('import signal, sys, time\n'
            'signal.signal(signal.SIGINT, lambda *_: sys.exit(0))\n'
            'time.sleep(60)\n')
    assert _run_chain(tmp_path, code, shutdown_after_s=1.0) == (False, [])
