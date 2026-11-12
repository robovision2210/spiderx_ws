#!/usr/bin/env python3
"""Offline response-law analyzer for a COMPLETED S2 bag: plant identification and tracking.

This is an offline plant-validity check. It reads a closed bag after the run and cannot stop a
run in progress; the online stop conditions stay with the driver and the node.

Plant law per controller update k (gz_ros2_control position interface; see
docs/S2_SIM_ACTUATOR_MODEL_DESIGN.md):

    x[k+1] - x[k] = g * (c[k] - x[k])

c[k] is the command written at update k, x[k] the position read at update k.

Pairs are built only from RECORDED commands at IDENTIFIED update times, never from commands or
hold timing inferred with an assumed gain:
  * c[k] is controller_state.output, the command read back from the command interface (its
    reference when output is empty), or action feedback.desired. Two sources for the same update
    must agree, otherwise that update is dropped.
  * t_k for action feedback is its header stamp, accepted only if feedback.actual equals
    /joint_states at that stamp exactly.
  * t_k for controller_state is the unique /joint_states sample, within WINDOW_NS of its stamp,
    whose positions equal controller_state.feedback exactly. Its header stamp is the controller
    node's /clock time and trails the update. Ambiguous (rest) and unmatched samples are excluded
    and counted.
  * x[k] and x[k+1] are /joint_states at t_k and t_k + T. A pair is never formed across a
    missing update.
The command applied over (t_k, t_k + T] is by construction the one written at t_k, so a goal
start, a cancellation or a hold between two updates needs no further accounting. Joints are
matched by name in every message. Simulation time (header stamps), receipt time (bag) and wall
time (not in the bag) are kept apart.

The gain is a closed-form least-squares fit over the pairs: no search interval, so no value is
excluded a priori. It is fitted pooled, per joint, and per segment (feedback windows of goals vs
the holds after them), independently of the expected gain, which is only compared afterwards.
Tracking (P2 basis: action-feedback error) is reported separately from plant identification.

Identification thresholds, fixed before any new run and justified by the 2026-10-29 bag
(g = 0.1, 1200 joint-steps, residual RMS 1.1e-12 and max 1.9e-12 rad) and by synthetic data:
  GAIN_TOLERANCE           1e-6  acceptance: |g_hat - g_expected| < 1e-6 (owner requirement)
  MIN_PAIRS                20    pairs per joint (or segment) before a gain is reported
  MIN_EXCITATION_RAD       1e-6  max |c - x|; holds show ~1e-11, tracking >= 1e-4
  MIN_EXCITATION_NORM_RAD  1e-4  sqrt(sum (c - x)^2); with residual <= 1e-11 rad, the gain's
                                 standard error is <= 1e-11 / 1e-4 = 1e-7, a tenth of the tolerance
  MAX_RESIDUAL_RAD         1e-9  max |dx - g_hat (c - x)|: 500x the observed floor, while a
                                 one-update ordering error leaves >= 3e-5 rad in the same data
Below these, a joint is UNMEASURED, never assigned a gain.

Plant verdict:
  * CONTRADICTED: any identified fit (pooled, per joint, per segment) has |g_hat - expected| >=
    tolerance, or a residual above MAX_RESIDUAL_RAD.
  * INSUFFICIENT_EVIDENCE: otherwise, if the pooled fit or any required joint is unidentified.
  * SUPPORTED: otherwise.
A good pooled fit never excuses an identified joint's mismatch.

    response_law.py --bag BAG --expected-gain G --out DIR [options]
    Exit: 0 SUPPORTED, 1 CONTRADICTED, 2 INSUFFICIENT_EVIDENCE, 3 input error.
Stdlib only, except reading a bag, which needs the ROS 2 toolchain (rclpy) and opens the bag with
SQLite mode=ro&immutable=1, recording its SHA-256 before and after.
"""
import argparse
import collections
import dataclasses
import hashlib
import json
import math
import os
import sys

SCHEMA = 'spiderx.response_law/1'
UPDATE_PERIOD_NS = 10_000_000
WINDOW_NS = 100_000_000
GAIN_TOLERANCE = 1e-6
MIN_PAIRS = 20
MIN_EXCITATION_RAD = 1e-6
MIN_EXCITATION_NORM_RAD = 1e-4
MAX_RESIDUAL_RAD = 1e-9
TRACKING_LIMIT_RAD = 0.02
NS = 1_000_000_000
EXIT = {'SUPPORTED': 0, 'CONTRADICTED': 1, 'INSUFFICIENT_EVIDENCE': 2}

TOPIC_JS = '/joint_states'
TOPIC_CS = '/leg_trajectory_controller/controller_state'
TOPIC_FB = '/leg_trajectory_controller/follow_joint_trajectory/_action/feedback'
TOPIC_STATUS = '/spiderx/locomotion/status'


@dataclasses.dataclass(frozen=True)
class JointStateSample:
    stamp_ns: int            # simulation time (header)
    receipt_ns: int          # bag receipt time
    names: tuple
    positions: tuple


@dataclasses.dataclass(frozen=True)
class CommandSample:
    source: str              # 'controller_state' | 'feedback'
    stamp_ns: int            # header stamp (controller_state: NOT the update time)
    receipt_ns: int
    names: tuple
    command: tuple           # output (or reference) / feedback.desired
    measured: tuple          # controller_state.feedback / feedback.actual
    error: tuple = ()        # recorded error (desired - actual)
    reference: tuple = ()    # controller_state.reference (to compare with output)
    goal_id: str = None      # feedback only


@dataclasses.dataclass(frozen=True)
class StatusSample:
    receipt_ns: int
    data: dict


def _finite(values):
    return all(isinstance(v, float) and math.isfinite(v) for v in values)


def _reorder(names, values, order):
    """values (in names order) in the canonical order, or None if the names do not match."""
    if len(names) != len(values) or len(set(names)) != len(names) or set(names) != set(order):
        return None
    index = {n: i for i, n in enumerate(names)}
    return tuple(float(values[index[n]]) for n in order)


def _stats(values):
    if not values:
        return {'n': 0, 'rms': None, 'max_abs': None}
    return {'n': len(values), 'rms': math.sqrt(sum(v * v for v in values) / len(values)),
            'max_abs': max(abs(v) for v in values)}


def fit_gain(e, dx, expected):
    """Least squares dx = g e through the origin, with the identifiability checks."""
    n = len(e)
    sum_e2 = sum(v * v for v in e)
    norm = math.sqrt(sum_e2)
    max_e = max((abs(v) for v in e), default=0.0)
    out = {'pairs': n, 'max_abs_excitation_rad': max_e, 'excitation_norm_rad': norm}
    reasons = []
    if n < MIN_PAIRS:
        reasons.append(f'{n} pairs < {MIN_PAIRS}')
    if max_e < MIN_EXCITATION_RAD:
        reasons.append(f'max |c - x| {max_e:.3g} rad < {MIN_EXCITATION_RAD:g}')
    if norm < MIN_EXCITATION_NORM_RAD:
        reasons.append(f'excitation norm {norm:.3g} rad < {MIN_EXCITATION_NORM_RAD:g}')
    if reasons or sum_e2 == 0.0:
        out.update({'status': 'UNMEASURED', 'reason': '; '.join(reasons) or 'no excitation',
                    'gain': None})
        return out
    g = sum(a * b for a, b in zip(e, dx)) / sum_e2
    out.update({
        'status': 'IDENTIFIED', 'gain': g, 'gain_minus_expected': g - expected,
        'residual_at_gain': _stats([b - g * a for a, b in zip(e, dx)]),
        'residual_at_expected_gain': _stats([b - expected * a for a, b in zip(e, dx)])})
    out['gain_standard_error'] = out['residual_at_gain']['rms'] / norm
    out['gain_matches_expected'] = abs(g - expected) < GAIN_TOLERANCE
    out['residual_ok'] = out['residual_at_gain']['max_abs'] <= MAX_RESIDUAL_RAD
    return out


def analyze(joint_states, commands, statuses=(), expected_gain=1.0, required_joints=None,
            update_period_ns=UPDATE_PERIOD_NS, window_ns=WINDOW_NS,
            tracking_limit_rad=TRACKING_LIMIT_RAD):
    """Plant identification and tracking from recorded samples. Returns the report dict."""
    excluded = collections.Counter()
    if not math.isfinite(expected_gain):
        raise ValueError('expected gain must be finite')
    order = None
    for c in commands:                       # the controller's joint order, for reporting
        if len(set(c.names)) == len(c.names) and c.names:
            order = tuple(c.names)
            break
    if order is None and joint_states:
        order = tuple(sorted(joint_states[0].names))
    if order is None:
        raise ValueError('no joint names in the input')

    # ------------------------------------------------ joint states, by simulation time
    js = {}
    receipt = {}
    for s in joint_states:
        pos = _reorder(s.names, s.positions, order)
        if pos is None:
            excluded['joint_states_name_mismatch'] += 1
            continue
        if not _finite(pos):
            excluded['joint_states_non_finite'] += 1
            continue
        if s.stamp_ns in js:
            if js[s.stamp_ns] != pos:
                excluded['joint_states_conflicting_duplicate_stamp'] += 1
                js[s.stamp_ns] = None
            else:
                excluded['joint_states_identical_duplicate'] += 1
            continue
        js[s.stamp_ns] = pos
        receipt[s.stamp_ns] = s.receipt_ns
    js = {t: p for t, p in js.items() if p is not None}
    stamps = sorted(js)
    diffs = collections.Counter(b - a for a, b in zip(stamps, stamps[1:]))
    gaps = [(a, b) for a, b in zip(stamps, stamps[1:]) if b - a != update_period_ns]
    by_receipt = sorted(stamps, key=lambda t: receipt[t])
    receipt_inversions = sum(1 for a, b in zip(by_receipt, by_receipt[1:]) if b < a)
    by_position = collections.defaultdict(list)
    for t in stamps:
        by_position[js[t]].append(t)

    # ------------------------------------------------ commands at identified update times
    at_update = {}
    cs_offsets = collections.Counter()
    output_vs_reference = collections.Counter()
    for c in commands:
        cmd = _reorder(c.names, c.command, order)
        meas = _reorder(c.names, c.measured, order)
        if cmd is None or meas is None:
            excluded[f'{c.source}_name_mismatch'] += 1
            continue
        if not (_finite(cmd) and _finite(meas)):
            excluded[f'{c.source}_non_finite'] += 1
            continue
        if c.source == 'feedback':
            if js.get(c.stamp_ns) != meas:
                excluded['feedback_without_identical_joint_state_at_its_stamp'] += 1
                continue
            t = c.stamp_ns
        else:
            cands = [t for t in by_position.get(meas, ()) if abs(t - c.stamp_ns) <= window_ns]
            if not cands:
                excluded['controller_state_no_exact_joint_state_match'] += 1
                continue
            if len(cands) > 1:
                excluded['controller_state_ambiguous_match_at_rest'] += 1
                continue
            t = cands[0]
            cs_offsets[round((c.stamp_ns - t) / 1e6, 3)] += 1
            if c.reference:
                ref = _reorder(c.names, c.reference, order)
                output_vs_reference['equal' if ref == cmd else 'different'] += 1
        prev = at_update.get(t)
        if prev is None:
            at_update[t] = (c.source, cmd)
        elif prev is not False and prev[1] == cmd:
            at_update[t] = (prev[0] if prev[0] == c.source else 'both', cmd)
        elif prev is not False:
            excluded['conflicting_commands_for_one_update'] += 1
            at_update[t] = False
    at_update = {t: v for t, v in at_update.items() if v is not False}

    # ------------------------------------------------ goals and segments (feedback windows)
    goals = collections.OrderedDict()
    for c in sorted((c for c in commands if c.source == 'feedback'), key=lambda c: c.stamp_ns):
        g = goals.setdefault(c.goal_id, {'goal_id': c.goal_id, 'first_stamp_ns': c.stamp_ns,
                                         'first_receipt_ns': c.receipt_ns, 'samples': []})
        g['last_stamp_ns'], g['last_receipt_ns'] = c.stamp_ns, c.receipt_ns
        g['samples'].append(c)
    goal_list = list(goals.values())
    status_goals, mapping_note = _status_goals(statuses)
    if status_goals and len(status_goals) == len(goal_list):
        for g, s in zip(goal_list, status_goals):
            g['node'] = s
    elif statuses:
        mapping_note = (f'{len(status_goals)} goals in the node status vs {len(goal_list)} '
                        'feedback goal ids: not mapped' if status_goals else mapping_note)

    def segment(t):
        last = None
        for i, g in enumerate(goal_list):
            if g['first_stamp_ns'] <= t <= g['last_stamp_ns']:
                return f'goal_{i + 1}', 'tracking'
            if g['last_stamp_ns'] < t:
                last = i
        if last is None:
            return 'before_first_goal', 'idle'
        cancelled = goal_list[last].get('node', {}).get('cancel_sent')
        return (f'after_goal_{last + 1}',
                'hold_after_cancel' if cancelled else 'hold_after_goal')

    # ------------------------------------------------ pairs and fits
    pairs = []
    for t in sorted(at_update):
        nxt = t + update_period_ns
        if nxt not in js:
            excluded['next_update_missing'] += 1
            continue
        source, cmd = at_update[t]
        seg, kind = segment(t)
        x0, x1 = js[t], js[nxt]
        pairs.append({'t': t, 'source': source, 'segment': seg, 'kind': kind,
                      'e': [c - x for c, x in zip(cmd, x0)],
                      'dx': [b - a for a, b in zip(x0, x1)]})

    def fit(selected, joint=None):
        if joint is None:
            e = [v for p in selected for v in p['e']]
            dx = [v for p in selected for v in p['dx']]
        else:
            e = [p['e'][joint] for p in selected]
            dx = [p['dx'][joint] for p in selected]
        return fit_gain(e, dx, expected_gain)

    pooled = fit(pairs)
    per_joint = {n: fit(pairs, j) for j, n in enumerate(order)}
    kinds = sorted({p['kind'] for p in pairs})
    per_kind = {k: fit([p for p in pairs if p['kind'] == k]) for k in kinds}

    contradictions, insufficient = [], []
    for label, f in [('pooled', pooled)] + [(f'joint {n}', f) for n, f in per_joint.items()] + \
            [(f'segment {k}', f) for k, f in per_kind.items()]:
        if f['status'] != 'IDENTIFIED':
            continue
        if not f['gain_matches_expected']:
            contradictions.append(f'{label}: g_hat {f["gain"]!r} differs from {expected_gain!r} '
                                  f'by {f["gain_minus_expected"]:.3g} (>= {GAIN_TOLERANCE:g})')
        if not f['residual_ok']:
            contradictions.append(f'{label}: residual max {f["residual_at_gain"]["max_abs"]:.3g} '
                                  f'rad > {MAX_RESIDUAL_RAD:g}')
    required = list(order) if required_joints is None else list(required_joints)
    unmeasured = [n for n in required if per_joint.get(n, {}).get('status') != 'IDENTIFIED']
    if pooled['status'] != 'IDENTIFIED':
        insufficient.append(f'pooled fit unidentified: {pooled.get("reason")}')
    if unmeasured:
        insufficient.append(f'required joints unmeasured: {unmeasured}')
    verdict = ('CONTRADICTED' if contradictions else
               'INSUFFICIENT_EVIDENCE' if insufficient else 'SUPPORTED')

    return {
        'schema': SCHEMA,
        'scope': 'offline plant-validity check of a completed bag; not an online stop condition',
        'expected_gain': expected_gain,
        'thresholds': {'gain_tolerance': GAIN_TOLERANCE, 'min_pairs': MIN_PAIRS,
                       'min_excitation_rad': MIN_EXCITATION_RAD,
                       'min_excitation_norm_rad': MIN_EXCITATION_NORM_RAD,
                       'max_residual_rad': MAX_RESIDUAL_RAD, 'update_period_ns': update_period_ns,
                       'controller_state_match_window_ns': window_ns},
        'joint_order': list(order),
        'plant': {'verdict': verdict, 'contradictions': contradictions,
                  'insufficient': insufficient, 'unmeasured_required_joints': unmeasured,
                  'pooled': pooled, 'per_joint': per_joint, 'per_segment_kind': per_kind},
        'evidence': {
            'pairs': len(pairs),
            'pairs_by_source': dict(collections.Counter(p['source'] for p in pairs)),
            'pairs_by_segment': dict(collections.Counter(p['segment'] for p in pairs)),
            'updates_with_recorded_command': len(at_update),
            'excluded': dict(excluded),
            'controller_state_header_minus_update_ms': dict(sorted(cs_offsets.items())),
            'controller_state_output_vs_reference': dict(output_vs_reference)},
        'timing': {
            'simulation': {'joint_states': len(stamps),
                           'first_s': stamps[0] / NS if stamps else None,
                           'last_s': stamps[-1] / NS if stamps else None,
                           'stamp_differences_ns': dict(diffs.most_common(5)),
                           'gaps': len(gaps),
                           'gap_examples_s': [[a / NS, b / NS] for a, b in gaps[:10]]},
            'receipt': {'first_ns': min(receipt.values()) if receipt else None,
                        'last_ns': max(receipt.values()) if receipt else None,
                        'joint_states_received_out_of_stamp_order': receipt_inversions},
            'wall': 'not in the bag (driver and node logs carry CLOCK_MONOTONIC)'},
        'goals': [{'index': i + 1, 'goal_id': g['goal_id'], 'feedback_samples': len(g['samples']),
                   'first_feedback_s': g['first_stamp_ns'] / NS,
                   'last_feedback_s': g['last_stamp_ns'] / NS,
                   'first_feedback_receipt_ns': g['first_receipt_ns'],
                   'last_feedback_receipt_ns': g['last_receipt_ns'],
                   'node': g.get('node')} for i, g in enumerate(goal_list)],
        'goal_mapping_note': mapping_note,
        'tracking': _tracking(commands, order, goal_list, tracking_limit_rad),
    }


def _status_goals(statuses):
    """Per-goal records from the node status (seq order); cancel_sent if ever reported."""
    seqs = collections.OrderedDict()
    for s in sorted(statuses, key=lambda s: s.receipt_ns):
        g = (s.data or {}).get('goal')
        if not isinstance(g, dict) or 'seq' not in g:
            continue
        rec = seqs.setdefault(g['seq'], {k: g.get(k) for k in (
            'seq', 'fingerprint', 'kind', 'leg', 'level', 'phase', 'direction', 'b_from',
            'b_to', 'duration_s')})
        rec['cancel_sent'] = bool(rec.get('cancel_sent') or g.get('cancel_sent'))
        m = g.get('max_tracking_error_rad')
        if isinstance(m, (int, float)):
            prior = rec.get('node_max_tracking_error_rad', 0.0)
            rec['node_max_tracking_error_rad'] = max(prior, m)
    note = None if seqs else ('no goal entries in the node status' if statuses else
                              'no node status recorded')
    return list(seqs.values()), note


def _tracking(commands, order, goal_list, limit):
    """P2 basis: the recorded action-feedback error. Separate from plant identification."""
    fb = sorted((c for c in commands if c.source == 'feedback'), key=lambda c: c.stamp_ns)
    if not fb:
        return {'basis': 'action feedback error', 'verdict': 'NO_DATA', 'samples': 0}
    identity_mismatch = skipped = 0
    rows = []
    for c in fb:
        err = _reorder(c.names, c.error, order) if c.error else None
        des = _reorder(c.names, c.command, order)
        act = _reorder(c.names, c.measured, order)
        if err is None or des is None or act is None or not _finite(err):
            skipped += 1
            continue
        if any(e != d - a for e, d, a in zip(err, des, act)):
            identity_mismatch += 1
        j = max(range(len(order)), key=lambda i: abs(err[i]))
        rows.append((c.stamp_ns, c.goal_id, order[j], err[j], c.receipt_ns))
    above = [r for r in rows if abs(r[3]) > limit]
    worst = max(rows, key=lambda r: abs(r[3])) if rows else None
    per_goal = []
    for i, g in enumerate(goal_list):
        gr = [r for r in rows if r[1] == g['goal_id']]
        w = max(gr, key=lambda r: abs(r[3])) if gr else None
        per_goal.append({'index': i + 1, 'samples': len(gr),
                         'max_abs_error_rad': abs(w[3]) if w else None,
                         'joint': w[2] if w else None, 'at_s': w[0] / NS if w else None,
                         'samples_above_limit': sum(1 for r in gr if abs(r[3]) > limit)})
    return {
        'basis': 'action feedback error (desired - actual), the S2 P2 basis', 'limit_rad': limit,
        'samples': len(rows), 'samples_skipped_unusable': skipped,
        'error_identity_mismatches': identity_mismatch,
        'max_abs_error_rad': abs(worst[3]) if worst else None,
        'max_at_s': worst[0] / NS if worst else None, 'max_joint': worst[2] if worst else None,
        'samples_above_limit': len(above),
        'first_crossing': None if not above else {
            's': above[0][0] / NS, 'joint': above[0][2], 'error_rad': above[0][3],
            'receipt_ns': above[0][4]},
        'per_goal': per_goal,
        'verdict': 'PASS' if not above else 'FAIL'}


# ======================================================================== bag input
def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def read_bag(path):
    """Samples from a rosbag2 SQLite file, opened read-only; needs the ROS 2 toolchain."""
    import sqlite3
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    before = sha256(path)
    con = sqlite3.connect(f'file:{path}?mode=ro&immutable=1', uri=True)
    topics = {r[0]: (r[1], r[2]) for r in con.execute('SELECT id, name, type FROM topics')}
    wanted = {TOPIC_JS, TOPIC_CS, TOPIC_FB, TOPIC_STATUS}
    types = {i: get_message(t) for i, (n, t) in topics.items() if n in wanted}
    js, cmds, status, counts = [], [], [], collections.Counter()
    for tid, ts, data in con.execute('SELECT topic_id, timestamp, data FROM messages '
                                     'ORDER BY timestamp, id'):
        name = topics[tid][0]
        if name not in wanted:
            continue
        counts[name] += 1
        m = deserialize_message(data, types[tid])
        if name == TOPIC_JS:
            js.append(JointStateSample(_ns(m.header.stamp), ts, tuple(m.name),
                                       tuple(m.position)))
        elif name == TOPIC_CS:
            out = tuple(m.output.positions)
            ref = tuple(m.reference.positions)
            cmds.append(CommandSample('controller_state', _ns(m.header.stamp), ts,
                                      tuple(m.joint_names), out if out else ref,
                                      tuple(m.feedback.positions), tuple(m.error.positions),
                                      ref if out else ()))
        elif name == TOPIC_FB:
            f = m.feedback
            cmds.append(CommandSample('feedback', _ns(f.header.stamp), ts, tuple(f.joint_names),
                                      tuple(f.desired.positions), tuple(f.actual.positions),
                                      tuple(f.error.positions), (),
                                      bytes(m.goal_id.uuid).hex()))
        else:
            try:
                status.append(StatusSample(ts, json.loads(m.data)))
            except ValueError:
                counts['status_not_json'] += 1
    con.close()
    after = sha256(path)
    meta = {'path': os.path.abspath(path), 'sha256_before': before, 'sha256_after': after,
            'unchanged': before == after, 'open_mode': 'sqlite3 mode=ro&immutable=1',
            'messages': dict(counts)}
    return js, cmds, status, meta


def _ns(stamp):
    return stamp.sec * NS + stamp.nanosec


def summary_text(report):
    p, t = report['plant'], report['tracking']
    lines = [f"plant verdict: {p['verdict']} (expected gain {report['expected_gain']!r})"]
    if p['pooled']['status'] == 'IDENTIFIED':
        pooled = p['pooled']
        lines.append(f"pooled g_hat {pooled['gain']!r} over {pooled['pairs']} joint-updates, "
                     f"residual max {pooled['residual_at_gain']['max_abs']:.3g}")
    for n, f in p['per_joint'].items():
        lines.append(f"  {n}: " + (f"g_hat {f['gain']!r}, residual max "
                                   f"{f['residual_at_gain']['max_abs']:.3g}"
                                   if f['status'] == 'IDENTIFIED' else
                                   f"UNMEASURED ({f['reason']})"))
    lines += [f'  contradiction: {c}' for c in p['contradictions']]
    lines += [f'  insufficient: {c}' for c in p['insufficient']]
    lines.append(f"tracking ({t.get('basis')}): {t['verdict']}, max |error| "
                 f"{t.get('max_abs_error_rad')} at {t.get('max_at_s')} s ({t.get('max_joint')}), "
                 f"first crossing {t.get('first_crossing')}")
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bag', required=True, help='rosbag2 .db3 file (read-only)')
    ap.add_argument('--expected-gain', type=float, required=True)
    ap.add_argument('--out', required=True, help='new or empty output directory')
    ap.add_argument('--update-period-s', type=float, default=UPDATE_PERIOD_NS / NS)
    ap.add_argument('--tracking-limit-rad', type=float, default=TRACKING_LIMIT_RAD)
    ap.add_argument('--required-joints', default='all',
                    help="'all' (default) or a comma-separated list of joint names")
    a = ap.parse_args(argv)
    if os.path.exists(a.out) and os.listdir(a.out):
        print(f'refusing to write into non-empty {a.out}', file=sys.stderr)
        return 3
    try:
        js, cmds, status, meta = read_bag(a.bag)
        report = analyze(js, cmds, status, a.expected_gain,
                         None if a.required_joints == 'all' else a.required_joints.split(','),
                         round(a.update_period_s * NS), WINDOW_NS, a.tracking_limit_rad)
    except (OSError, ValueError, KeyError) as e:
        print(f'input error: {e}', file=sys.stderr)
        return 3
    report['inputs'] = meta
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, 'response_law.json'), 'w') as f:
        json.dump(report, f, indent=1, sort_keys=True)
    text = summary_text(report)
    with open(os.path.join(a.out, 'summary.txt'), 'w') as f:
        f.write(text + '\n')
    print(text)
    if not meta['unchanged']:
        print('BAG CHANGED WHILE READING', file=sys.stderr)
        return 3
    return EXIT[report['plant']['verdict']]


if __name__ == '__main__':
    sys.exit(main())
