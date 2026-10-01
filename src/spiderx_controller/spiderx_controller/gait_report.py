"""M4.5 OFFLINE report writers: per-gait CSV/JSON tables and labelled PNG figures.

Layout (deterministic; default root log/m4_5_gait_analysis/, git-ignored by the repo's `log/`):

    <out>/run_info.json                      config version/date, gaits, samples (no timestamps)
    <out>/<gait>/report.json                 spec, verdict, every check (with honesty label), metrics
    <out>/<gait>/samples.csv                 one row per sample: support, COM, margin, targets, q, IK reason
    <out>/<gait>/phase_diagram.png           stance bars per leg over one cycle
    <out>/<gait>/foot_trajectories.png       body-frame foot offsets y(u), z(u) per leg
    <out>/<gait>/stability_margin.png        quasi-static margin over the cycle (approximation)
    <out>/<gait>/joint_angles.png            hip / thigh / knee angles of every leg
    <out>/comparison/summary.csv             one row per gait: verdict, failed checks, key metrics
    <out>/comparison/report.json             the same, with metric labels and the scope statement
    <out>/comparison/metrics.png             small multiples of the key metrics, one bar per gait

Numbers are written with 10 significant digits so files are byte-identical between runs.
Figures use matplotlib's non-interactive Agg backend and are saved to files only; every figure has
a CSV/JSON table beside it, and leg identity is carried by legend, label and line style, never by
colour alone. If matplotlib is missing, tables are still written and figures are skipped.
"""

import csv
import dataclasses
import json
import os

from spiderx_controller import leg_kinematics as lk

SIG = 10
# Validated categorical order (dataviz reference palette, light surface; CVD/normal checks pass).
# Contrast below 3:1 for slots 3-4 is relieved by legends, line styles and the CSV tables.
LEG_COLORS = {'front_left': '#2a78d6', 'front_right': '#eb6834', 'rear_left': '#1baf7a',
              'rear_right': '#eda100'}
LEG_STYLES = {'front_left': '-', 'front_right': '--', 'rear_left': '-.', 'rear_right': ':'}
LEG_SHORT = {'front_left': 'LF', 'front_right': 'RF', 'rear_left': 'LR', 'rear_right': 'RR'}
INK, INK_2, MUTED, GRID, SURFACE = '#0b0b0b', '#52514e', '#898781', '#e6e5e1', '#fcfcfb'
FAIL_INK = '#d03b3b'
MIN_ANGLE_SPAN_RAD = 0.1


def fmt(v):
    """Deterministic number formatting (10 significant digits); None -> empty."""
    if v is None:
        return ''
    if isinstance(v, bool):
        return str(v).lower()
    if isinstance(v, float):
        return format(v, f'.{SIG}g')
    return str(v)


def jsonable(v):
    """Round floats to 10 significant digits recursively; tuples -> lists."""
    if isinstance(v, float):
        return float(format(v, f'.{SIG}g'))
    if isinstance(v, dict):
        return {str(k): jsonable(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [jsonable(x) for x in v]
    return v


def write_json(path, data):
    with open(path, 'w') as f:
        json.dump(jsonable(data), f, indent=2, sort_keys=True)
        f.write('\n')


def spec_dict(spec):
    d = dataclasses.asdict(spec)
    d['stride_length_m'] = spec.stride_length_m
    return d


def gait_report_dict(ev, cfg):
    return {
        'scope': 'OFFLINE kinematic analysis only - not walking, not a validated gait, not '
                 'hardware. Approximations and heuristic proxies are labelled per check.',
        'config_version': cfg.config_version, 'config_date': cfg.date, 'frame': cfg.frame,
        'direction_of_travel': '+y',
        'samples_per_cycle': ev.kinematics.n,
        'gait': spec_dict(ev.spec),
        'verdict': 'PASS' if ev.passed else 'FAIL',
        'failed_checks': ev.failed_checks,
        'checks': [c.as_dict() for c in ev.checks],
        'metrics': ev.metrics,
        'metric_labels': METRIC_LABELS,
    }


METRIC_LABELS = {
    'min_static_margin_m': 'approximation (quasi-static, URDF CAD masses)',
    'fraction_statically_stable': 'approximation (quasi-static, URDF CAD masses)',
    'max_joint_speed_rad_s': 'sampled; compare with the 0.5 rad/s SIMULATION_PLACEHOLDER',
    'foot_path_per_m': 'heuristic proxy, NOT energy',
    'joint_travel_rad_per_m': 'heuristic proxy, NOT energy',
    'lift_work_proxy_j_per_m': 'heuristic proxy sum(m_leg g dz+), NOT measured work or power',
    'steps_per_m': 'exact (definition: 4 / stride)',
    'max_transition_acceleration_jump_m_s2': 'sampled (information only)',
}


def samples_rows(ev):
    kin, series = ev.kinematics, ev.series
    header = ['k', 'u', 't_s', 'support_count', 'support_legs', 'static_margin_m',
              'com_x_m', 'com_y_m', 'com_z_m']
    for leg in lk.ALL_LEGS:
        s = LEG_SHORT[leg]
        header += [f'{s}_phase', f'{s}_target_x_m', f'{s}_target_y_m', f'{s}_target_z_m',
                   f'{s}_q_hip_rad', f'{s}_q_thigh_rad', f'{s}_q_knee_rad', f'{s}_ik_reason']
    rows = [header]
    for k, sample in enumerate(kin.samples):
        com = series['com_m'][k] or (None, None, None)
        row = [k, sample['u'], sample['t'], series['support_count'][k],
               '|'.join(LEG_SHORT[leg] for leg in sample['support']),
               series['static_margin_m'][k], *com]
        for leg in lk.ALL_LEGS:
            fs, ls = sample['feet'][leg], kin.legs[leg]
            q = ls.q[k] or (None, None, None)
            row += [fs.phase, *fs.target, *q, ls.reason[k]]
        rows.append([fmt(v) if not isinstance(v, str) else v for v in row])
    return rows


def write_csv(path, rows):
    with open(path, 'w', newline='') as f:
        csv.writer(f, lineterminator='\n').writerows(rows)


# ---------------------------------------------------------------- figures
def _pyplot():
    """matplotlib.pyplot with the non-interactive Agg backend, or None if not installed."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        return None
    plt.rcParams.update({
        'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
        'axes.edgecolor': MUTED, 'axes.labelcolor': INK_2, 'xtick.color': MUTED,
        'ytick.color': MUTED, 'text.color': INK, 'axes.grid': True, 'grid.color': GRID,
        'grid.linewidth': 0.8, 'axes.spines.top': False, 'axes.spines.right': False,
        'font.size': 9, 'axes.titlesize': 10, 'legend.frameon': False, 'lines.linewidth': 1.6,
        'svg.hashsalt': 'spiderx-m4.5',
    })
    return plt


def _save(plt, fig, path):
    fig.savefig(path, dpi=150, metadata={'Software': None})
    plt.close(fig)


def _subtitle(ev):
    s = ev.spec
    return (f'{s.name}: beta {s.duty_factor:g}, T {s.cycle_period_s:.2f} s, L {s.step_length_m*1e3:.0f} '
            f'mm, h {s.step_height_m*1e3:.0f} mm, v {s.body_speed_m_s*1e3:.1f} mm/s - '
            f'{"PASS" if ev.passed else "FAIL " + ", ".join(ev.failed_checks)}')


def plot_phase_diagram(plt, ev, path):
    spec, n = ev.spec, ev.kinematics.n
    fig, ax = plt.subplots(figsize=(7.0, 2.6))
    for row, leg in enumerate(reversed(lk.ALL_LEGS)):
        phases = [s['feet'][leg].phase for s in ev.kinematics.samples]
        k = 0
        while k < n:
            if phases[k] == 'stance':
                j = k
                while j < n and phases[j] == 'stance':
                    j += 1
                ax.broken_barh([(k / n, (j - k) / n)], (row - 0.3, 0.6),
                               facecolors=LEG_COLORS[leg], edgecolor=SURFACE, linewidth=2)
                k = j
            else:
                k += 1
    ax.set_axisbelow(True)
    ax.set_yticks(range(4))
    ax.set_yticklabels([LEG_SHORT[leg] for leg in reversed(lk.ALL_LEGS)])
    ax.set_xlim(0, 1)
    ax.set_xlabel('normalised cycle time u = t / T')
    ax.set_title(f'Footfall diagram (bars = stance, gaps = swing)\n{_subtitle(ev)}', loc='left')
    ax.grid(axis='y', visible=False)
    fig.tight_layout()
    _save(plt, fig, path)


def plot_foot_trajectories(plt, ev, path):
    us = [s['u'] for s in ev.kinematics.samples]
    fig, (ax_y, ax_z) = plt.subplots(2, 1, figsize=(8.0, 4.6), sharex=True)
    for leg in lk.ALL_LEGS:
        offs = [s['feet'][leg].offset for s in ev.kinematics.samples]
        kw = dict(color=LEG_COLORS[leg], linestyle=LEG_STYLES[leg], label=LEG_SHORT[leg])
        ax_y.plot(us, [o[1] * 1000 for o in offs], **kw)
        ax_z.plot(us, [o[2] * 1000 for o in offs], **kw)
    ax_y.set_ylabel('y offset, mm (+ front)')
    ax_z.set_ylabel('z offset, mm (+ up)')
    ax_z.set_xlabel('normalised cycle time u = t / T')
    ax_y.legend(loc='upper left', bbox_to_anchor=(1.01, 1.0))
    ax_y.set_title('Foot offsets from the CAD-neutral tip, base_link (body frame); legs that share a '
                   'phase overlap\n' + _subtitle(ev), loc='left')
    fig.tight_layout()
    _save(plt, fig, path)


def plot_stability(plt, ev, path, threshold):
    us = [s['u'] for s in ev.kinematics.samples]
    margins = [None if m is None else m * 1000 for m in ev.series['static_margin_m']]
    fig, ax = plt.subplots(figsize=(9.0, 3.6))
    n = len(us)
    for k, count in enumerate(ev.series['support_count']):
        if count < 3:
            ax.axvspan(k / n, (k + 1) / n, color=GRID, linewidth=0)
    ax.plot(us, [float('nan') if m is None else m for m in margins], color=LEG_COLORS['front_left'],
            label='COM-to-support-edge margin')
    ax.axhline(threshold * 1000, color=INK_2, linestyle='--', linewidth=1.0,
               label=f'required {threshold*1000:.0f} mm' + ('' if ev.spec.requires_static_stability
                                                            else ' (not required: information)'))
    ax.axhline(0.0, color=MUTED, linewidth=0.8)
    ax.set_xlim(0, 1)
    ax.set_xlabel('normalised cycle time u = t / T (grey: < 3 stance feet, no support polygon)')
    ax.set_ylabel('margin, mm (+ inside)')
    ax.legend(loc='upper left', bbox_to_anchor=(1.01, 1.0))
    ax.set_title('Quasi-static stability margin - APPROXIMATION\n(URDF CAD masses, point feet at the '
                 'derived tips, flat ground, no dynamics)\n' + _subtitle(ev), loc='left')
    fig.tight_layout()
    _save(plt, fig, path)


def plot_joint_angles(plt, ev, path):
    us = [s['u'] for s in ev.kinematics.samples]
    fig, axes = plt.subplots(3, 1, figsize=(8.0, 5.8), sharex=True)
    for j, (ax, role) in enumerate(zip(axes, ('hip', 'thigh', 'knee ("foot" joint)'))):
        for leg in lk.ALL_LEGS:
            q = ev.kinematics.legs[leg].q
            ax.plot(us, [float('nan') if v is None else v[j] for v in q], color=LEG_COLORS[leg],
                    linestyle=LEG_STYLES[leg], label=LEG_SHORT[leg])
        ax.set_ylabel(f'{role}, rad')
        lo, hi = ax.get_ylim()
        if hi - lo < MIN_ANGLE_SPAN_RAD:          # do not magnify round-off (e.g. hip ~ 1e-16)
            mid = (lo + hi) / 2
            ax.set_ylim(mid - MIN_ANGLE_SPAN_RAD / 2, mid + MIN_ANGLE_SPAN_RAD / 2)
    axes[0].legend(loc='upper left', bbox_to_anchor=(1.01, 1.0))
    axes[-1].set_xlabel('normalised cycle time u = t / T')
    axes[0].set_title('Joint angles from the M3/M4 IK (URDF limits minus 0.05 rad enforced)\n'
                      + _subtitle(ev), loc='left')
    fig.tight_layout()
    _save(plt, fig, path)


# ---------------------------------------------------------------- writers
def write_gait_report(ev, cfg, out_dir, plots=True):
    """Write one gait's tables (always) and figures (if plots and matplotlib). Returns paths."""
    d = os.path.join(out_dir, ev.spec.name)
    os.makedirs(d, exist_ok=True)
    paths = {'report': os.path.join(d, 'report.json'), 'samples': os.path.join(d, 'samples.csv')}
    write_json(paths['report'], gait_report_dict(ev, cfg))
    write_csv(paths['samples'], samples_rows(ev))
    plt = _pyplot() if plots else None
    if plt is not None:
        for name, fn in (('phase_diagram', plot_phase_diagram),
                         ('foot_trajectories', plot_foot_trajectories),
                         ('joint_angles', plot_joint_angles)):
            paths[name] = os.path.join(d, f'{name}.png')
            fn(plt, ev, paths[name])
        paths['stability_margin'] = os.path.join(d, 'stability_margin.png')
        plot_stability(plt, ev, paths['stability_margin'], cfg.analysis.min_static_margin_m)
    return paths


def write_run_info(cfg, evaluations, out_dir, plots_written):
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, 'run_info.json')
    write_json(path, {
        'tool': 'spiderx_controller m4_5_gait_analysis (offline)',
        'config_version': cfg.config_version, 'config_date': cfg.date,
        'samples_per_cycle': cfg.analysis.samples_per_cycle,
        'gaits': [ev.spec.name for ev in evaluations],
        'verdicts': {ev.spec.name: 'PASS' if ev.passed else 'FAIL' for ev in evaluations},
        'plots': 'written' if plots_written else 'skipped',
    })
    return path


# ---------------------------------------------------------------- cross-gait comparison
COMPARISON_METRICS = (
    'duty_factor', 'cycle_period_s', 'stride_length_m', 'min_support_feet',
    'min_static_margin_m', 'fraction_statically_stable', 'max_joint_speed_rad_s',
    'min_singularity_margin_rad', 'min_joint_limit_margin_rad', 'max_joint_step_rad',
    'steps_per_m', 'foot_path_per_m', 'joint_travel_rad_per_m', 'lift_work_proxy_j_per_m',
)
# (metric, scale, panel title) - small multiples, one hue, one bar per gait
COMPARISON_PANELS = (
    ('min_static_margin_m', 1000.0, 'min static margin, mm\n(approximation)'),
    ('fraction_statically_stable', 100.0, 'statically stable samples, %\n(approximation)'),
    ('max_joint_speed_rad_s', 1.0, 'max joint speed, rad/s\n(sampled; 0.5 = placeholder)'),
    ('joint_travel_rad_per_m', 1.0, 'joint travel, rad per m\n(heuristic proxy)'),
    ('lift_work_proxy_j_per_m', 1.0, 'lift work, J per m\n(heuristic proxy, NOT energy)'),
    ('foot_path_per_m', 1.0, 'foot path, m per m\n(heuristic proxy)'),
)


def comparison_rows(evaluations):
    header = ['gait', 'verdict', 'failed_checks', 'requires_static_stability', *COMPARISON_METRICS]
    rows = [header]
    for ev in evaluations:
        row = [ev.spec.name, 'PASS' if ev.passed else 'FAIL', '|'.join(ev.failed_checks),
               ev.spec.requires_static_stability, *(ev.metrics[k] for k in COMPARISON_METRICS)]
        rows.append([fmt(v) if not isinstance(v, str) else v for v in row])
    return rows


def comparison_dict(cfg, evaluations):
    return {
        'scope': 'OFFLINE cross-gait comparison of kinematic metrics - not walking, not a ranking '
                 'of validated gaits, not hardware. Stability values are quasi-static '
                 'approximations; path/travel/lift values are heuristic proxies, NOT energy.',
        'config_version': cfg.config_version, 'config_date': cfg.date,
        'samples_per_cycle': cfg.analysis.samples_per_cycle,
        'gaits': [ev.spec.name for ev in evaluations],
        'verdicts': {ev.spec.name: 'PASS' if ev.passed else 'FAIL' for ev in evaluations},
        'failed_checks': {ev.spec.name: ev.failed_checks for ev in evaluations},
        'metrics': {ev.spec.name: {k: ev.metrics[k] for k in COMPARISON_METRICS}
                    for ev in evaluations},
        'metric_labels': METRIC_LABELS,
    }


def plot_comparison(plt, evaluations, path, cfg):
    names = [f'{ev.spec.name} (beta {ev.spec.duty_factor:g})'
             + ('' if ev.passed else '  FAIL') for ev in evaluations]
    rows = list(range(len(evaluations)))[::-1]                 # configuration order, top down
    fig, axes = plt.subplots(2, 3, figsize=(11.0, 5.8), sharey=True)
    for ax, (key, scale, title) in zip(axes.flat, COMPARISON_PANELS):
        values = [ev.metrics[key] for ev in evaluations]
        for row, v in zip(rows, values):
            if v is None:
                ax.text(0, row, ' n/a', va='center', ha='left', color=MUTED, fontsize=8)
                continue
            ax.barh(row, v * scale, height=0.6, color=LEG_COLORS['front_left'])
            ax.text(v * scale, row, f' {v * scale:.3g}', va='center',
                    ha='left' if v >= 0 else 'right', color=INK_2, fontsize=8)
        ax.axvline(0.0, color=MUTED, linewidth=0.8)
        if key == 'min_static_margin_m':
            ax.axvline(cfg.analysis.min_static_margin_m * scale, color=INK_2, linestyle='--',
                       linewidth=1.0)
        if key == 'max_joint_speed_rad_s':
            ax.axvline(cfg.analysis.joint_speed_reference_rad_s, color=INK_2, linestyle='--',
                       linewidth=1.0)
        ax.margins(x=0.25)
        ax.set_axisbelow(True)
        ax.grid(axis='y', visible=False)
        ax.set_title(title, loc='left', fontsize=9)
    for ax in axes[:, 0]:
        ax.set_yticks(rows)
        ax.set_yticklabels(names, color=INK_2)
    fig.suptitle('Cross-gait comparison - OFFLINE kinematic analysis, equal body speed; '
                 'FAIL = failed a required check (see comparison/report.json)\n'
                 'dashed lines: required static margin / joint-speed placeholder; '
                 'n/a: < 3 stance feet, no support polygon',
                 x=0.01, ha='left', fontsize=10)
    fig.tight_layout()
    _save(plt, fig, path)


def write_comparison(cfg, evaluations, out_dir, plots=True):
    """Write comparison/{summary.csv, report.json} (always) and metrics.png (if plots)."""
    d = os.path.join(out_dir, 'comparison')
    os.makedirs(d, exist_ok=True)
    paths = {'summary': os.path.join(d, 'summary.csv'), 'report': os.path.join(d, 'report.json')}
    write_csv(paths['summary'], comparison_rows(evaluations))
    write_json(paths['report'], comparison_dict(cfg, evaluations))
    plt = _pyplot() if plots else None
    if plt is not None:
        paths['metrics'] = os.path.join(d, 'metrics.png')
        plot_comparison(plt, evaluations, paths['metrics'], cfg)
    return paths


def plotting_available():
    return _pyplot() is not None


__all__ = ['write_gait_report', 'write_run_info', 'write_comparison', 'comparison_rows',
           'gait_report_dict', 'samples_rows', 'fmt', 'jsonable', 'plotting_available',
           'LEG_COLORS']
