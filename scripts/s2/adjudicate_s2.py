#!/usr/bin/env python3
"""Retrospective S2 assessment under criteria revision r1 (offline; the frozen scorer stays as is).

The frozen scorer (docs/evidence/m55/s2_fixed_base_20261029T100210Z/tools/analyze_s2.py) and its
verdicts are never modified or re-labelled. This tool re-assesses only the two criteria the owner
amended on 2026-10-30 and carries every other verdict over unchanged:

P2 (r1): tracking <= 0.02 rad (unchanged) on every normally executed goal, by action feedback.
  Only the FINAL SCRIPTED ESTOP goal may be "tracking N/A - planned cancellation before
  feedback", and only with proof of: goal identity and acceptance; the scripted estop key; the
  cancel requested before the goal could publish feedback; exactly one cancel; result CANCELED;
  no refusal or unplanned fault before it; no feedback for it. Missing feedback on any other goal
  stays UNMEASURED. No tracking value is invented for an N/A goal.
P4 (r1): an ABSENT command topic counts as zero recorded messages only with independent support:
  the topic was requested from the recorder; recording covered the goal interval with discovery
  demonstrated; the recorder log shows no failure; the node-monitored streams stayed continuous
  (within the node's own staleness limits); the command-owner monitor and node events show no
  foreign command source; and goals, cancels and exercise order match the script. Otherwise the
  term is UNMEASURED. A present topic with any message is FAIL.

Feedback is attributed to goals by time (node events carry no action goal id): bag receipt times
are mapped to the node's CLOCK_MONOTONIC with the status messages that both the recorder and the
driver received (offset = median, tolerance = the largest pair deviation).

Timing terms (m55_rclpy / m55_locomotion): every time is the node's CLOCK_MONOTONIC, seen by the
CLIENT. goal_sent and cancel_sent are stamped when the node calls send_goal_async / requests the
cancel; a cancel requested before the goal handle arrives is QUEUED and sent from the goal-response
callback (which case applied is not logged). goal_response, cancel_response and result are
stamped at the 20 Hz session tick that POLLED them: each callback happened in (previous poll,
stamp]. Server-side acceptance and cancel processing are not recorded and are never inferred.

Usage: adjudicate_s2.py --evidence <run evidence dir> --run-root <run root> --out <new dir>"""
import argparse
import collections
import csv
import gzip
import json
import os
import re
import statistics
import sys

REVISION = 'r1'
LABEL = 'Retrospective assessment under criteria revision r1'
SCHEMA = 'spiderx.s2_adjudication/1'
TRACK_MAX_RAD = 0.02                      # design section 4 P2, unchanged
EXPECTED_GOALS, EXPECTED_CANCELS = 59, 1
EXERCISES = [(0, 1, 1), (1, 0, -1), (0, 1, 1), (1, 0, -1), (0, 1, 1)]   # = the frozen scorer
STATUS_CANCELED = 5
COMMAND_TOPIC = '/leg_trajectory_controller/joint_trajectory'
TOPIC_JS = '/joint_states'
TOPIC_POSE = '/spiderx/sim/world_poses'
TOPIC_STATUS = '/spiderx/locomotion/status'
TOPIC_FB = '/leg_trajectory_controller/follow_joint_trajectory/_action/feedback'
TOPIC_CS = '/leg_trajectory_controller/controller_state'


def verdict(ok, measured):
    return 'PASS' if measured and ok else ('FAIL' if measured else 'UNMEASURED')


# ------------------------------------------------------------------ P2 (r1)
def assess_p2(goals, feedback, keys, cancels, faults, refusals, tol_s, feedback_period_s):
    """goals: [{seq, sent, accepted, response_polled_t, result_polled_t, status, b_from, b_to,
    direction, level, cancel_response_code, cancel_response_polled_t}];
    feedback: {goal_id: [(t, max_abs_error)]}; keys: [{key, t, why}]; cancels: [{seq, t}];
    faults: [{reason, t}]; refusals: [{request, reason, t}]. Times: node CLOCK_MONOTONIC s."""
    out = {'criterion': 'P2', 'revision': REVISION, 'limit_rad': TRACK_MAX_RAD, 'problems': []}
    by_seq = {g['seq']: g for g in goals}
    mapped, unmapped = collections.defaultdict(list), []
    for gid, samples in sorted(feedback.items(), key=lambda kv: kv[1][0][0]):
        lo, hi = samples[0][0], samples[-1][0]
        hits = [g['seq'] for g in goals if g.get('sent') is not None
                and g.get('result_polled_t') is not None
                and g['sent'] - tol_s <= lo and hi <= g['result_polled_t'] + tol_s]
        if len(hits) == 1:
            mapped[hits[0]].append(gid)
        else:
            unmapped.append({'goal_id': gid, 'candidates': hits})
    out['feedback_ids'] = len(feedback)
    out['feedback_ids_unattributed'] = unmapped
    if unmapped:
        out['problems'].append(f'{len(unmapped)} feedback goal id(s) not attributable to one goal')
    for seq, ids in mapped.items():
        if len(ids) > 1:
            out['problems'].append(f'goal {seq}: {len(ids)} feedback goal ids')
    last_seq = max(by_seq) if by_seq else None
    rows, fail, unmeasured = [], False, bool(unmapped) or any(len(v) > 1 for v in mapped.values())
    for g in sorted(goals, key=lambda g: g['seq']):
        samples = [s for gid in mapped.get(g['seq'], []) for s in feedback[gid]]
        row = {'seq': g['seq'], 'feedback_samples': len(samples)}
        if samples:
            row['max_abs_error_rad'] = max(e for _, e in samples)
            row['verdict'] = 'PASS' if row['max_abs_error_rad'] <= TRACK_MAX_RAD else 'FAIL'
            fail |= row['verdict'] == 'FAIL'
        elif g['seq'] == last_seq:
            na = estop_na_proof(g, goals, feedback, mapped, keys, cancels, faults, refusals,
                                tol_s, feedback_period_s)
            row['estop_proof'] = na
            if na['qualifies']:
                row['verdict'] = 'N/A'
                row['note'] = 'tracking N/A - planned cancellation before feedback'
            else:
                row['verdict'] = 'UNMEASURED'
                unmeasured = True
                out['problems'].append(f'goal {g["seq"]}: no feedback and not a qualifying '
                                       f'planned estop: {na["failed"]}')
        else:
            row['verdict'] = 'UNMEASURED'
            unmeasured = True
            out['problems'].append(f'goal {g["seq"]}: no feedback (normally executed goal)')
        rows.append(row)
    out['goals'] = rows
    measured = [r['max_abs_error_rad'] for r in rows if 'max_abs_error_rad' in r]
    out['max_abs_error_rad'] = max(measured) if measured else None
    out['goals_with_feedback'] = len(measured)
    out['goals_na'] = [r['seq'] for r in rows if r['verdict'] == 'N/A']
    out['verdict'] = 'FAIL' if fail else ('UNMEASURED' if unmeasured or not rows else 'PASS')
    return out


def estop_na_proof(g, goals, feedback, mapped, keys, cancels, faults, refusals, tol_s,
                   feedback_period_s):
    """Every condition for 'tracking N/A - planned cancellation before feedback' on goal g."""
    seq = g['seq']
    t_sent, t_resp, t_res = g.get('sent'), g.get('response_polled_t'), g.get('result_polled_t')
    t_cresp = g.get('cancel_response_polled_t')
    xkeys = [k for k in keys if k['key'] == 'x' and 'estop exercise' in (k.get('why') or '')]
    x_in = [k for k in xkeys if t_sent is not None and t_res is not None
            and t_sent <= k['t'] <= t_res]
    c_seq = [c for c in cancels if c['seq'] == seq]
    t_x = x_in[0]['t'] if len(x_in) == 1 else None
    t_c = c_seq[0]['t'] if len(c_seq) == 1 else None
    estop_faults = [f for f in faults if f['reason'] == 'estop']
    planned = [f for f in estop_faults if t_x is not None and f['t'] >= t_x]
    others = [f for f in faults if f not in planned]
    early_refusals = [r for r in refusals if t_res is None or r['t'] <= t_res]
    fb_in_window = [gid for gid, s in feedback.items()
                    if any(t_sent - tol_s <= t <= t_res + tol_s for t, _ in s)] \
        if t_sent is not None and t_res is not None else ['window unknown']
    ex = (g.get('b_from'), g.get('b_to'), g.get('direction'))
    checks = {
        'is_final_scripted_goal': seq == EXPECTED_GOALS and len(goals) == EXPECTED_GOALS
        and seq == max(gg['seq'] for gg in goals),
        'identity_matches_estop_exercise': ex == EXERCISES[-1] and g.get('level') == 0,
        'accepted': g.get('accepted') is True,
        'scripted_estop_key_during_goal': len(x_in) == 1 and len(xkeys) == 1,
        'estop_fault_follows_key': t_x is not None and len(estop_faults) == 1
        and t_x <= estop_faults[0]['t'] <= (t_res if t_res is not None else -1),
        'exactly_one_cancel_overall': len(cancels) == EXPECTED_CANCELS and len(c_seq) == 1,
        'cancel_after_key': t_c is not None and t_x is not None and t_c >= t_x,
        'early_cancel_request': t_c is not None and t_resp is not None
        and (t_c <= t_resp or (t_c - t_resp) < feedback_period_s),
        'cancel_processed_by_server': g.get('cancel_response_code') == 0
        and t_cresp is not None and t_res is not None and t_cresp <= t_res,
        'result_canceled': g.get('status') == STATUS_CANCELED,
        'no_unplanned_fault_before': not [f for f in others if f['t'] <= (t_res or 0.0)],
        'no_refusal_before_result': not early_refusals,
        'no_feedback_for_goal': not mapped.get(seq) and not fb_in_window,
    }
    timing = {'goal_sent_t': t_sent, 'estop_key_t (driver)': t_x,
              'cancel_requested_t (queued if no goal handle yet)': t_c,
              'goal_response_polled_t (accepted; callback in (prev poll, t])': t_resp,
              'cancel_response_polled_t': t_cresp, 'result_polled_t': t_res,
              'cancel_requested_after_send_s': None if t_c is None or t_sent is None
              else t_c - t_sent,
              'server_active_interval_upper_bound_s (cancel response polled - sent)':
              None if t_cresp is None or t_sent is None else t_cresp - t_sent,
              'observed_feedback_period_s': feedback_period_s, 'alignment_tolerance_s': tol_s,
              'not_recorded': 'server-side acceptance and cancel-processing times'}
    return {'qualifies': all(checks.values()), 'checks': checks,
            'failed': [k for k, v in checks.items() if not v], 'timing': timing}


# ------------------------------------------------------------------ P4 (r1)
def assess_p4(frozen_p4, topic_present, topic_count, requested_topics, recorder, interval,
              continuity, owner, events_check, captures):
    """frozen_p4: the frozen scorer's P4 fields. recorder: {started_t, stopped_t, errors,
    discovered_after_start}; interval: (t0, t1) of the goals; continuity: {topic: {max_gap_s,
    limit_s}}; owner: {samples, max_publishers, max_foreign, max_graph_age_s, graph_stale_s,
    max_status_gap_s}; events_check: {owner_refusals}; captures: {before, after}
    publisher counts."""
    out = {'criterion': 'P4', 'revision': REVISION, 'problems': []}
    terms = {
        'goals_sent_59': frozen_p4.get('goals_sent') == EXPECTED_GOALS,
        'cancels_1': frozen_p4.get('cancels_sent') == EXPECTED_CANCELS,
        'foreign_clients_0': frozen_p4.get('max_foreign_clients') == 0,
        'command_topic_publishers_0': frozen_p4.get('max_command_topic_publishers') == 0,
        'exercise_order_ok': frozen_p4.get('exercise_order_ok') is True,
    }
    out['frozen_terms'] = terms
    if topic_present:
        out['command_topic'] = {'present_in_bag': True, 'messages': topic_count}
        jt = 'PASS' if topic_count == 0 else 'FAIL'
        if jt == 'FAIL':
            out['problems'].append(f'{topic_count} message(s) on {COMMAND_TOPIC}')
    else:
        t0, t1 = interval
        support = {
            'requested_from_recorder': COMMAND_TOPIC in requested_topics,
            'recording_covers_goal_interval': recorder.get('started_t') is not None
            and recorder.get('stopped_t') is not None
            and recorder['started_t'] <= t0 and recorder['stopped_t'] >= t1,
            'discovery_demonstrated_after_start': bool(recorder.get('discovered_after_start')),
            'recorder_log_without_failure': recorder.get('errors') == [],
            'node_monitored_streams_continuous': bool(continuity) and all(
                c['max_gap_s'] is not None and c['max_gap_s'] <= c['limit_s']
                for c in continuity.values()),
            'owner_monitor_zero_throughout': owner.get('samples', 0) > 0
            and owner.get('samples_without_graph') == 0
            and owner.get('max_publishers') == 0 and owner.get('max_foreign') == 0,
            'owner_monitor_fresh': owner.get('max_graph_age_s') is not None
            and owner['max_graph_age_s'] <= owner['graph_stale_s']
            and owner.get('max_status_gap_s') is not None
            and owner['max_status_gap_s'] <= owner['graph_stale_s'],
            'no_owner_refusal_in_node_events': events_check.get('owner_refusals') == [],
            'captures_before_after_zero_publishers': captures.get('before') == 0
            and captures.get('after') == 0,
            'script_counts_and_order_match': all(terms.values()),
        }
        out['command_topic'] = {'present_in_bag': False, 'support': support,
                                'failed': [k for k, v in support.items() if not v]}
        jt = 'PASS' if all(support.values()) else 'UNMEASURED'
        if jt == 'UNMEASURED':
            out['problems'].append('absent command topic without adequate independent '
                                   f'coverage: {out["command_topic"]["failed"]}')
    out['command_topic']['term'] = jt
    if not all(terms.values()) or jt == 'FAIL':
        out['verdict'] = 'FAIL'
        out['problems'] += [f'frozen term failed: {k}' for k, v in terms.items() if not v]
    else:
        out['verdict'] = 'PASS' if jt == 'PASS' else 'UNMEASURED'
    out['sampling_limitations'] = [
        'the node graph monitor samples once per graph_period_s (1.0 s); a publisher that '
        'exists only between samples is not seen by it',
        'the recorder subscribes only after discovering a publisher; messages sent by a '
        'publisher that appears and disappears before discovery are not recorded',
        'captures of the command topic exist only before and after the script']
    return out


def overall(frozen_p, p2, p4, plant_verdict):
    p = {k: dict(v) for k, v in frozen_p.items()}
    p['P2'] = {'verdict': p2['verdict'], 'revision': REVISION}
    p['P4'] = {'verdict': p4['verdict'], 'revision': REVISION}
    vs = [v['verdict'] for v in p.values()]
    if any(v == 'FAIL' for v in vs) or plant_verdict == 'CONTRADICTED':
        res = 'FAIL'
    elif all(v == 'PASS' for v in vs) and plant_verdict == 'SUPPORTED':
        res = 'PASS'
    else:
        res = 'INCONCLUSIVE'
    return {'P': {k: p[k]['verdict'] for k in sorted(p)}, 'plant': plant_verdict, 'result': res}


# ------------------------------------------------------------------ inputs from a run
def align(driver_statuses, bag_statuses):
    """Offset (bag receipt realtime s - CLOCK_MONOTONIC s) from statuses received by both."""
    index = collections.defaultdict(list)
    for t, data in bag_statuses:
        index[json.dumps(data, sort_keys=True)].append(t)
    offs = [index[k][0] - mono for mono, k in
            ((m, json.dumps(d, sort_keys=True)) for m, d in driver_statuses)
            if len(index.get(k, ())) == 1]
    if len(offs) < 10:
        return None
    off = statistics.median(offs)
    return {'offset_s': off, 'pairs': len(offs), 'of_driver_statuses': len(driver_statuses),
            'tolerance_s': max(abs(o - off) for o in offs),
            'p05_p95_ms': [round((sorted(offs)[int(0.05 * len(offs))] - off) * 1e3, 2),
                           round((sorted(offs)[int(0.95 * len(offs))] - off) * 1e3, 2)]}


def max_gap(times, t0, t1):
    inside = sorted(t for t in times if t0 <= t <= t1)
    if not inside:
        return None
    edges = [t0] + inside + [t1]
    return max(b - a for a, b in zip(edges, edges[1:]))


def load_inputs(evidence, run_root, bag):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import response_law as rl
    import sqlite3
    js, cmds, statuses, meta = rl.read_bag(bag)
    con = sqlite3.connect(f'file:{bag}?mode=ro&immutable=1', uri=True)
    topics = {r[0]: r[1] for r in con.execute('SELECT id, name FROM topics')}
    receipts = collections.defaultdict(list)
    for tid, ts in con.execute('SELECT topic_id, timestamp FROM messages'):
        receipts[topics[tid]].append(ts / 1e9)
    con.close()
    after = rl.sha256(bag)
    ev = [json.loads(ln) for ln in open(os.path.join(evidence, 'node', 'events.jsonl'))
          if ln.strip()]
    drv_path = os.path.join(evidence, 'driver', 'driver.jsonl.gz')
    drv = [json.loads(ln) for ln in gzip.open(drv_path, 'rt')]
    al = align([(d['mono'], d['status']) for d in drv if 'status' in d],
               [(s.receipt_ns / 1e9, s.data) for s in statuses])
    if al is None:
        raise SystemExit('cannot align bag receipt time to CLOCK_MONOTONIC')
    off = al['offset_s']
    goals = {}
    for e in ev:
        s = e.get('seq')
        if e['event'] == 'goal_sent':
            goals[s] = {'seq': s, 'sent': e['wall'], 'b_from': e['b_from'], 'b_to': e['b_to'],
                        'direction': e['direction'], 'level': e['level'], 'kind': e['kind'],
                        'fingerprint': e['fingerprint']}
        elif e['event'] == 'goal_response':
            goals[s].update(accepted=e['accepted'], response_polled_t=e['wall'])
        elif e['event'] == 'cancel_response':
            goals[s].update(cancel_response_code=e['code'], cancel_response_polled_t=e['wall'])
        elif e['event'] == 'result':
            goals[s].update(status=e['status'], result_polled_t=e['wall'])
    fb = collections.defaultdict(list)
    for c in cmds:
        if c.source == 'feedback':
            fb[c.goal_id].append((c.receipt_ns / 1e9 - off, max(abs(x) for x in c.error)))
    gaps = sorted(b[0] - a[0] for v in fb.values() for a, b in zip(v, v[1:]))
    keys = [{'key': d['key'].strip("'"), 't': d['mono'], 'why': d.get('why')}
            for d in drv if 'key' in d]
    cancels = [{'seq': e['seq'], 't': e['wall']} for e in ev if e['event'] == 'cancel_sent']
    faults = [{'reason': e['reason'], 't': e['wall']} for e in ev if e['event'] == 'fault']
    refusals = [{'request': e['request'], 'reason': e['reason'], 't': e['wall']}
                for e in ev if e['event'] == 'refused']
    gl = [goals[k] for k in sorted(goals)]
    t0, t1 = gl[0]['sent'], gl[-1]['result_polled_t']
    # recorder: requested topics (the harness), log, coverage
    script = open(os.path.join(evidence, 'tools', 'phase_c_run.sh')).read()
    rec_cmd = re.search(r'ros2 bag record(.*?)>', script, re.S).group(1)
    requested = re.findall(r'(/[A-Za-z0-9_/]+)', rec_cmd.replace('\\\n', ' '))
    log_path = os.path.join(run_root, 'run', 'bag_record.log')
    log = [re.sub(r'\x1b\[[0-9;]*m', '', ln) for ln in open(log_path)]
    stamp = lambda ln: float(re.search(r'\[(\d+\.\d+)\]', ln).group(1)) - off  # noqa: E731
    started = [stamp(ln) for ln in log if 'Recording...' in ln]
    stopped = [stamp(ln) for ln in log if 'Recording stopped' in ln]
    subs = [(stamp(ln), re.search(r"topic '([^']+)'", ln).group(1)) for ln in log
            if 'Subscribed to topic' in ln]
    errors = [ln.strip() for ln in log if re.search(r'\[(WARN|ERROR|FATAL)\]', ln)]
    recorder = {'started_t': started[0] if started else None,
                'stopped_t': stopped[0] if stopped else None, 'errors': errors,
                'subscriptions': [{'t': t, 'topic': n} for t, n in subs],
                'discovered_after_start': [n for t, n in subs if started and t > started[0]]}
    mono = {k: [t - off for t in v] for k, v in receipts.items()}
    continuity = {TOPIC_JS: {'max_gap_s': max_gap(mono.get(TOPIC_JS, []), t0, t1),
                             'limit_s': 0.5, 'limit_source': 'monitor.joint_states_stale_s'},
                  TOPIC_POSE: {'max_gap_s': max_gap(mono.get(TOPIC_POSE, []), t0, t1),
                               'limit_s': 1.0, 'limit_source': 'monitor.pose_stale_s'}}
    info_gaps = {k: max_gap(mono.get(k, []), t0, t1) for k in (TOPIC_CS, TOPIC_STATUS)}
    st_in = [(s.receipt_ns / 1e9 - off, s.data) for s in statuses
             if t0 <= s.receipt_ns / 1e9 - off <= t1]
    graphs = [d['monitors'].get('graph') or {} for _, d in st_in]
    owner = {'samples': len(st_in), 'samples_without_graph': sum(1 for g in graphs if not g),
             'max_publishers': max((g.get('command_topic_publishers', -1) for g in graphs),
                                   default=None),
             'max_foreign': max((g.get('foreign_action_clients', -1) for g in graphs),
                                default=None),
             'max_graph_age_s': max((d['monitors'].get('graph_age_s', 1e9) for _, d in st_in),
                                    default=None),
             'graph_stale_s': 3.0, 'graph_period_s': 1.0,
             'max_status_gap_s': max_gap([t for t, _ in st_in], t0, t1)}
    owner_ref = [r for r in refusals if 'owner' in r['reason'] or 'foreign' in r['reason']]

    def pubs(name):
        txt = open(os.path.join(evidence, 'run', 'captures', name)).read()
        m = re.search(re.escape(COMMAND_TOPIC) + r'.*?Publisher count: (\d+)', txt, re.S)
        return int(m.group(1)) if m else None
    # context around the final goal (not a tracking measurement)
    last = gl[-1]
    w0, w1 = last['sent'], last['result_polled_t']
    cs_in = [c for c in cmds if c.source == 'controller_state'
             and w0 <= c.receipt_ns / 1e9 - off <= w1]
    js_in = [j for j in js if w0 - 0.1 <= j.receipt_ns / 1e9 - off <= w1 + 0.3]
    context = {
        'window_s': [w0, w1],
        'controller_state_samples_in_window': len(cs_in),
        'controller_state_max_abs_reference_minus_feedback_rad':
            max((max(abs(x) for x in c.error) for c in cs_in), default=None),
        'joint_state_samples_window_minus_0_1_plus_0_3_s': len(js_in),
        'joint_motion_max_abs_rad': max((max(abs(a - b) for a, b in
                                             zip(j.positions, js_in[0].positions))
                                         for j in js_in), default=None),
        'controller_state_max_gap_s_in_window': max_gap(
            [c.receipt_ns / 1e9 - off for c in cs_in], w0, w1),
        'note': 'context only: the controller_state reference after a cancel is the hold, not '
                'necessarily goal 59; this is not a P2 measurement'}
    return {'goals': gl, 'feedback': dict(fb), 'keys': keys, 'cancels': cancels,
            'faults': faults, 'refusals': refusals, 'alignment': al,
            'feedback_period_s': statistics.median(gaps) if gaps else None,
            'interval': (t0, t1), 'requested_topics': requested, 'recorder': recorder,
            'continuity': continuity, 'informational_max_gaps_s': info_gaps, 'owner': owner,
            'events_check': {'owner_refusals': owner_ref},
            'captures': {'before': pubs('before_topics.txt'), 'after': pubs('after_topics.txt')},
            'topic_present': COMMAND_TOPIC in receipts or COMMAND_TOPIC in topics.values(),
            'topic_count': len(receipts.get(COMMAND_TOPIC, [])),
            'final_goal_context': context,
            'bag': {**meta, 'sha256_after_topic_scan': after,
                    'unchanged': meta['unchanged'] and after == meta['sha256_before']}}


def main(argv=None):
    a = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    a.add_argument('--evidence', required=True)
    a.add_argument('--run-root', required=True)
    a.add_argument('--bag', default=None)
    a.add_argument('--out', required=True)
    o = a.parse_args(argv)
    if os.path.isdir(o.out) and os.listdir(o.out):
        print(f'refusing to write into non-empty {o.out}', file=sys.stderr)
        return 3
    bag = o.bag or os.path.join(o.run_root, 'bag', 'bag_0.db3')
    frozen = json.load(open(os.path.join(o.evidence, 'summary.json')))
    plant = json.load(open(os.path.join(o.evidence, 'response_law', 'response_law.json')))
    x = load_inputs(o.evidence, o.run_root, bag)
    p2 = assess_p2(x['goals'], x['feedback'], x['keys'], x['cancels'], x['faults'],
                   x['refusals'], x['alignment']['tolerance_s'], x['feedback_period_s'])
    frozen_rows = {int(r['seq']): r['max_abs_tracking_error_rad'] for r in
                   csv.DictReader(open(os.path.join(o.evidence, 'tracking.csv')))}
    p2['per_goal_max_equals_frozen_tracking_csv'] = all(
        (not frozen_rows[r['seq']]) if r.get('max_abs_error_rad') is None
        else bool(frozen_rows[r['seq']]) and r['max_abs_error_rad'] == float(frozen_rows[r['seq']])
        for r in p2['goals'])
    p2['final_goal_context'] = x['final_goal_context']
    p4 = assess_p4(frozen['P']['P4'], x['topic_present'], x['topic_count'],
                   x['requested_topics'], x['recorder'], x['interval'], x['continuity'],
                   x['owner'], x['events_check'], x['captures'])
    p4['evidence'] = {k: x[k] for k in ('requested_topics', 'recorder', 'continuity',
                                        'informational_max_gaps_s', 'owner', 'captures')}
    res = {'schema': SCHEMA, 'label': LABEL, 'revision': REVISION,
           'frozen': {'scorer': 'analyze_s2.py (sha256 e9c2efcb...)', 'overall': frozen['overall'],
                      'P': {k: v['verdict'] for k, v in frozen['P'].items()},
                      'note': 'the original frozen result; unchanged and still authoritative '
                              'for the original criteria'},
           'alignment': x['alignment'], 'feedback_period_s': x['feedback_period_s'],
           'P2': p2, 'P4': p4,
           'retrospective': overall(frozen['P'], p2, p4, plant['plant']['verdict']),
           'bag': x['bag']}
    os.makedirs(o.out, exist_ok=True)
    with open(os.path.join(o.out, 'assessment.json'), 'w') as f:
        json.dump(res, f, indent=1, sort_keys=True, default=str)
    lines = [LABEL + ' (NOT the original frozen result)',
             f"frozen (original criteria): {res['frozen']['overall']} {res['frozen']['P']}",
             f"r1: P2 {p2['verdict']} (N/A goals {p2['goals_na']}, max "
             f"{p2['max_abs_error_rad']} rad over {p2['goals_with_feedback']} goals); "
             f"P4 {p4['verdict']} (command topic term {p4['command_topic']['term']})",
             f"r1 retrospective: {res['retrospective']}",
             f"bag unchanged: {x['bag']['unchanged']}"]
    with open(os.path.join(o.out, 'summary.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    return 0


if __name__ == '__main__':
    sys.exit(main())
