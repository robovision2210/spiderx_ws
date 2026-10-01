"""colcon test: M4.5 Batch E - report writers (CSV/JSON/PNG) and the offline CLI runner."""
import csv
import json
import os
import sys

import pytest

from spiderx_controller import gait_metrics as gm
from spiderx_controller import gait_report as gr
from spiderx_controller import m4_5_gait_analysis as cli
from spiderx_controller import leg_kinematics as lk

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
GAIT_YAML = os.path.join(SRC_CONFIG, 'm4_5_gaits.yaml')
N_FAST = 40
PNGS = ('phase_diagram.png', 'foot_trajectories.png', 'joint_angles.png', 'stability_margin.png')


@pytest.fixture(scope='module')
def inputs():
    return gm.load_inputs(config_dir=SRC_CONFIG)


@pytest.fixture(scope='module')
def ripple(inputs):
    cfg, geoms, mass = inputs
    return gm.evaluate_gait(cfg.gait('ripple'), cfg, geoms, mass, n=N_FAST)


def _run(*args):
    return cli.main(['m4_5_gait_analysis', '--config', GAIT_YAML, '--config-dir', SRC_CONFIG,
                     *args])


def _read(path):
    with open(path, 'rb') as f:
        return f.read()


# ------------------------------------------------------------ formatting
@pytest.mark.parametrize('value,text', [
    (None, ''), (True, 'true'), (False, 'false'), (3, '3'), (0.1 + 0.2, '0.3'),
    (1.0 / 3.0, '0.3333333333'), ('swing', 'swing'),
])
def test_fmt_is_deterministic_and_short(value, text):
    assert gr.fmt(value) == text


def test_jsonable_rounds_floats_and_lists_tuples():
    assert gr.jsonable({'a': (0.1 + 0.2, None), 1: [1.0 / 3.0]}) == \
        {'a': [0.3, None], '1': [0.3333333333]}


# ------------------------------------------------------------ tables
def test_samples_csv_header_and_rows(ripple, tmp_path):
    rows = gr.samples_rows(ripple)
    assert len(rows) == N_FAST + 1
    header = rows[0]
    assert header[:9] == ['k', 'u', 't_s', 'support_count', 'support_legs', 'static_margin_m',
                          'com_x_m', 'com_y_m', 'com_z_m']
    assert len(header) == 9 + 8 * len(lk.ALL_LEGS)
    assert 'LF_q_knee_rad' in header and 'RR_ik_reason' in header
    assert all(len(r) == len(header) for r in rows)
    path = tmp_path / 'samples.csv'
    gr.write_csv(path, rows)
    with open(path, newline='') as f:
        back = list(csv.reader(f))
    assert back == rows
    first = dict(zip(header, back[1]))
    assert first['k'] == '0' and first['u'] == '0'
    assert first['LF_phase'] in ('swing', 'stance') and first['LF_ik_reason'] == 'ok'


def test_report_dict_carries_scope_verdict_and_labels(ripple, inputs):
    d = gr.gait_report_dict(ripple, inputs[0])
    assert 'OFFLINE' in d['scope'] and 'not walking' in d['scope'] and 'hardware' in d['scope']
    assert d['verdict'] == ('PASS' if ripple.passed else 'FAIL')
    assert d['failed_checks'] == ripple.failed_checks
    assert d['samples_per_cycle'] == N_FAST and d['direction_of_travel'] == '+y'
    assert d['gait']['name'] == 'ripple'
    assert d['gait']['stride_length_m'] == pytest.approx(ripple.spec.stride_length_m)
    assert {c['label'] for c in d['checks']} <= set(['exact', 'sampled', 'approximation',
                                                     'heuristic'])
    assert 'NOT energy' in d['metric_labels']['foot_path_per_m']
    assert 'approximation' in d['metric_labels']['min_static_margin_m']
    json.dumps(gr.jsonable(d))                                   # serialisable


def test_tables_are_byte_identical_across_runs(ripple, inputs, tmp_path):
    cfg = inputs[0]
    a = gr.write_gait_report(ripple, cfg, str(tmp_path / 'a'), plots=False)
    b = gr.write_gait_report(ripple, cfg, str(tmp_path / 'b'), plots=False)
    assert set(a) == {'report', 'samples'}                     # no figures requested
    for key in ('report', 'samples'):
        assert _read(a[key]) == _read(b[key])
    assert not any(n.endswith('.png') for n in os.listdir(tmp_path / 'a' / 'ripple'))


# ------------------------------------------------------------ figures
@pytest.mark.skipif(not gr.plotting_available(), reason='matplotlib not installed')
def test_figures_are_written_as_png(ripple, inputs, tmp_path):
    paths = gr.write_gait_report(ripple, inputs[0], str(tmp_path), plots=True)
    for name in PNGS:
        path = tmp_path / 'ripple' / name
        assert str(path) in paths.values()
        assert _read(path)[:8] == b'\x89PNG\r\n\x1a\n'


def test_missing_matplotlib_skips_figures_but_writes_tables(ripple, inputs, tmp_path,
                                                            monkeypatch):
    monkeypatch.setitem(sys.modules, 'matplotlib', None)       # import -> ImportError
    monkeypatch.setitem(sys.modules, 'matplotlib.pyplot', None)
    assert not gr.plotting_available()
    paths = gr.write_gait_report(ripple, inputs[0], str(tmp_path), plots=True)
    assert set(paths) == {'report', 'samples'}
    assert sorted(os.listdir(tmp_path / 'ripple')) == ['report.json', 'samples.csv']


# ------------------------------------------------------------ CLI
def test_cli_check_only_validates_without_evaluating(tmp_path, capsys):
    out = tmp_path / 'out'
    assert _run('--check-only', '--out', str(out)) == 0
    text = capsys.readouterr().out
    assert 'Configuration valid' in text and 'OFFLINE' in text
    assert not out.exists()


def test_cli_refuses_an_invalid_configuration(tmp_path, capsys):
    bad = tmp_path / 'bad.yaml'
    with open(GAIT_YAML) as f:
        text = f.read()
    assert 'offline_only: true' in text
    bad.write_text(text.replace('offline_only: true', 'offline_only: false'))
    out = tmp_path / 'out'
    assert cli.main(['x', '--config', str(bad), '--config-dir', SRC_CONFIG,
                     '--out', str(out)]) == 2
    assert 'REFUSED' in capsys.readouterr().out
    assert not out.exists()


def test_cli_refuses_an_unknown_gait(tmp_path, capsys):
    assert _run('--check-only', '--gait', 'gallop') == 2
    assert 'REFUSED' in capsys.readouterr().out


def test_cli_run_writes_tables_and_strict_reflects_failures(tmp_path, capsys):
    out = tmp_path / 'out'
    assert _run('--gait', 'trot', '--no-plots', '--strict', '--out', str(out)) == 0
    assert sorted(os.listdir(out / 'trot')) == ['report.json', 'samples.csv']
    assert sorted(os.listdir(out / 'comparison')) == ['report.json', 'summary.csv']
    info = json.loads(_read(out / 'run_info.json'))
    assert info['gaits'] == ['trot'] and info['verdicts'] == {'trot': 'PASS'}
    assert info['plots'] == 'skipped'
    assert 'All evaluated gaits passed' in capsys.readouterr().out

    # tripod_crawl fails static stability (Batch D finding): a result, not an error...
    assert _run('--gait', 'tripod_crawl', '--no-plots', '--out', str(out)) == 0
    # ...unless --strict is given
    assert _run('--gait', 'tripod_crawl', '--no-plots', '--strict', '--out', str(out)) == 1
    report = json.loads(_read(out / 'tripod_crawl' / 'report.json'))
    assert report['verdict'] == 'FAIL' and 'static_stability' in report['failed_checks']
    assert 'FAILED' in capsys.readouterr().out


# ------------------------------------------------------------ cross-gait comparison
@pytest.fixture(scope='module')
def two(inputs):
    # configured resolution: joint_continuity bounds the per-SAMPLE step, so a coarse N_FAST
    # sampling of trot (0.0504 rad at n=40) would fail it while n=200 gives 0.0101 rad
    cfg, geoms, mass = inputs
    return [gm.evaluate_gait(cfg.gait(n), cfg, geoms, mass) for n in ('tripod_crawl', 'trot')]


def test_comparison_tables(two, inputs, tmp_path):
    cfg = inputs[0]
    rows = gr.comparison_rows(two)
    assert rows[0][:4] == ['gait', 'verdict', 'failed_checks', 'requires_static_stability']
    assert [r[0] for r in rows[1:]] == ['tripod_crawl', 'trot']          # configuration order
    by = {r[0]: dict(zip(rows[0], r)) for r in rows[1:]}
    assert by['tripod_crawl']['verdict'] == 'FAIL'
    assert 'static_stability' in by['tripod_crawl']['failed_checks'].split('|')
    assert by['trot']['verdict'] == 'PASS' and by['trot']['failed_checks'] == ''
    assert by['trot']['min_static_margin_m'] == ''                       # no polygon -> empty
    a = gr.write_comparison(cfg, two, str(tmp_path / 'a'), plots=False)
    b = gr.write_comparison(cfg, two, str(tmp_path / 'b'), plots=False)
    assert set(a) == {'summary', 'report'}
    for key in a:
        assert _read(a[key]) == _read(b[key])
    report = json.loads(_read(a['report']))
    assert 'NOT energy' in report['scope'] and 'not walking' in report['scope']
    assert report['verdicts'] == {'tripod_crawl': 'FAIL', 'trot': 'PASS'}
    assert report['metrics']['trot']['min_static_margin_m'] is None


@pytest.mark.skipif(not gr.plotting_available(), reason='matplotlib not installed')
def test_comparison_figure(two, inputs, tmp_path):
    paths = gr.write_comparison(inputs[0], two, str(tmp_path), plots=True)
    assert _read(paths['metrics'])[:8] == b'\x89PNG\r\n\x1a\n'
