"""colcon test: M6.0-A offline preflight CLI (ROS-free). Exit 0 = PASS, 2 = REFUSED."""
import copy
import filecmp
import json
import os

import pytest

from spiderx_controller import m6_offline_preflight as cli
from spiderx_controller import m6_trajectory as m6t
from spiderx_controller.config_check import load_urdf

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')


@pytest.fixture(scope='module')
def sources():
    return m6t.load_sources(SRC_CONFIG, load_urdf())


def run(sources, *args):
    return cli.main(['m6_offline_preflight', '--config-dir', SRC_CONFIG, *args], sources=sources)


def test_check_only_passes_and_writes_nothing(sources, tmp_path, capsys):
    assert run(sources, '--check-only', '--out', str(tmp_path)) == cli.EXIT_OK
    assert os.listdir(tmp_path) == []
    out = capsys.readouterr().out
    assert 'Preflight: PASS' in out
    assert 'not a command cap' in out
    assert 'no motion, tracking, contact or walking is shown' in out


def test_writes_deterministic_files(sources, tmp_path):
    a, b = tmp_path / 'a', tmp_path / 'b'
    assert run(sources, '--out', str(a)) == cli.EXIT_OK
    assert run(sources, '--out', str(b)) == cli.EXIT_OK
    (tid,) = os.listdir(a)
    assert os.listdir(b) == [tid]
    for name in ('trajectory.json', 'preflight.json'):
        assert filecmp.cmp(a / tid / name, b / tid / name, shallow=False)
    report = json.loads((a / tid / 'preflight.json').read_text())
    assert report['verdict'] == 'PASS' and report['trajectory_id'] == tid
    assert 'timestamp' not in (a / tid / 'preflight.json').read_text()


def test_existing_output_is_never_overwritten(sources, tmp_path, capsys):
    assert run(sources, '--out', str(tmp_path)) == cli.EXIT_OK
    (tid,) = os.listdir(tmp_path)
    before = (tmp_path / tid / 'trajectory.json').read_text()
    assert run(sources, '--out', str(tmp_path)) == cli.EXIT_REFUSED
    assert 'already exists' in capsys.readouterr().out
    assert (tmp_path / tid / 'trajectory.json').read_text() == before


def test_given_valid_trajectory_file_passes(sources, tmp_path):
    path = tmp_path / 't.json'
    m6t.write_json(str(path), m6t.build_trajectory(sources))
    assert run(sources, '--check-only', '--trajectory', str(path)) == cli.EXIT_OK


@pytest.mark.parametrize('mutate, code', [
    (lambda t: t['points'][1]['positions'].__setitem__(2, 0.1224), 'displacement_exceeds_cap'),
    (lambda t: t['points'][2].__setitem__('time_from_start_s', 31.0), 'duration_exceeds_max'),
    (lambda t: t['points'][0].__setitem__('time_from_start_s', 0.0), 'start_delay_missing'),
    (lambda t: t.__setitem__('mode', 'cyclic'), 'mode_not_single'),
    (lambda t: t['joint_names'].reverse(), 'joint_order_not_canonical'),
])
def test_refused_trajectory_file_exits_2_and_writes_nothing(mutate, code, sources, tmp_path,
                                                            capsys):
    t = copy.deepcopy(m6t.build_trajectory(sources))
    mutate(t)
    t['trajectory_id'] = m6t.compute_trajectory_id(t)
    path = tmp_path / 't.json'
    path.write_text(json.dumps(t))
    out_dir = tmp_path / 'out'
    assert run(sources, '--trajectory', str(path), '--out', str(out_dir)) == cli.EXIT_REFUSED
    out = capsys.readouterr().out
    assert f'REFUSED {code}' in out
    assert 'no goal can be constructed or sent' in out
    assert not out_dir.exists()


def test_nan_in_file_is_refused(sources, tmp_path, capsys):
    t = m6t.build_trajectory(sources)
    text = json.dumps(t).replace('"time_from_start_s": 6.0', '"time_from_start_s": NaN')
    path = tmp_path / 't.json'
    path.write_text(text)
    assert run(sources, '--check-only', '--trajectory', str(path)) == cli.EXIT_REFUSED
    assert 'REFUSED non_finite_value' in capsys.readouterr().out


def test_unreadable_trajectory_file_is_refused(sources, tmp_path, capsys):
    path = tmp_path / 't.json'
    path.write_text('{not json')
    assert run(sources, '--check-only', '--trajectory', str(path)) == cli.EXIT_REFUSED
    assert run(sources, '--check-only', '--trajectory', str(tmp_path / 'missing.json')) == \
        cli.EXIT_REFUSED


def test_bad_sources_are_refused(tmp_path, capsys):
    s = m6t.Sources(config_dir=str(tmp_path),
                    errors=(('neutral_source_missing', 'spiderx_poses.yaml not found'),))
    assert cli.main(['m6_offline_preflight', '--check-only'], sources=s) == cli.EXIT_REFUSED
    out = capsys.readouterr().out
    assert 'REFUSED neutral_source_missing' in out


def test_json_report_flag(sources, capsys):
    assert run(sources, '--check-only', '--json') == cli.EXIT_OK
    out = capsys.readouterr().out
    payload, _ = json.JSONDecoder().raw_decode(out[out.index('{'):])
    assert payload['schema'] == m6t.REPORT_SCHEMA and payload['ok'] is True


def test_cli_has_no_dispatch_path():
    import inspect
    text = inspect.getsource(cli) + inspect.getsource(m6t)
    for word in ('send_goal', 'ActionClient', 'create_publisher', 'rclpy'):
        assert word not in text
