#!/usr/bin/env python3
"""SUPPLEMENTARY, read-only analysis of the ONE S4c level-2 LIVE attempt (no ROS graph; the bag
opened read-only/immutable). It is the S4b live supplementary analysis, unchanged but for this
docstring: the verdict is the committed score_s4c.py --level 2 output (score/summary.json),
never this file. It records RTF, node-observed ages, transitions, controller traffic and exits.
Usage: analyze_s4c_live_supplementary.py <run root> -> prints JSON."""
import collections, glob, gzip, json, math, os, re, sqlite3, sys

R = sys.argv[1]
RUN = os.path.join(R, 'run')
JS_LIMIT = 0.5


def db(path):
    f = sorted(glob.glob(os.path.join(path, '*.db3')))
    return f[0] if f else None


def rows(dbf, wanted):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    con = sqlite3.connect(f'file:{dbf}?mode=ro&immutable=1', uri=True)
    topics = {r[0]: (r[1], r[2]) for r in con.execute('SELECT id, name, type FROM topics')}
    types = {i: get_message(t) for i, (n, t) in topics.items() if n in wanted}
    counts = collections.Counter(n for (n,) in con.execute(
        'SELECT t.name FROM messages m JOIN topics t ON m.topic_id = t.id'))
    out = collections.defaultdict(list)
    for tid, ts, data in con.execute('SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp, id'):
        n = topics[tid][0]
        if n in wanted:
            out[n].append((ts * 1e-9, deserialize_message(data, types[tid])))
    return {n: t for n, t in topics.values()}, counts, out


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def poses(msgs, cfg):
    from spiderx_controller.m55_rclpy import body_pose_sample
    res = []
    for recv, m in msgs:
        codes, p = body_pose_sample(m.transforms, cfg)
        if p is not None:
            st = max((stamp(t.header) for t in m.transforms), default=None)
            res.append((recv, st, p))
    return res


def js_stats(msgs, t0=None, t1=None):
    m = [(r, stamp(x.header), x) for r, x in msgs if (t0 is None or r >= t0) and (t1 is None or r <= t1)]
    if len(m) < 2:
        return None
    gaps = [b[0] - a[0] for a, b in zip(m, m[1:])]
    sgaps = [b[1] - a[1] for a, b in zip(m, m[1:])]
    q0 = dict(zip(m[0][2].name, m[0][2].position))
    motion = collections.defaultdict(float)
    for _, _, x in m:
        for n, q in zip(x.name, x.position):
            motion[n] = max(motion[n], abs(q - q0[n]))
    # RTF in 10 s wall windows
    win, i = [], 0
    for j in range(len(m)):
        while m[j][0] - m[i][0] > 10.0:
            i += 1
        if m[j][0] - m[i][0] >= 9.5:
            win.append((m[j][1] - m[i][1]) / (m[j][0] - m[i][0]))
    return {'samples': len(m), 'wall_s': m[-1][0] - m[0][0], 'sim_s': m[-1][1] - m[0][1],
            'rtf': (m[-1][1] - m[0][1]) / (m[-1][0] - m[0][0]),
            'rtf_10s_windows_min': min(win) if win else None,
            'rtf_10s_windows_max': max(win) if win else None,
            'max_receive_gap_s': max(gaps), 'max_stamp_gap_s': max(sgaps),
            'max_joint_motion_rad': max(motion.values()),
            'max_joint_motion_joint': max(motion, key=motion.get)}


def main():
    from spiderx_controller import m55_locomotion as loc
    cfg = loc.load_config()
    res = {}
    # ---- main bag
    T_ST, T_JS, T_WP = '/spiderx/locomotion/status', '/joint_states', '/spiderx/sim/world_poses'
    T_CS = '/leg_trajectory_controller/controller_state'
    T_JT = '/leg_trajectory_controller/joint_trajectory'
    T_FB = '/leg_trajectory_controller/follow_joint_trajectory/_action/feedback'
    tmap, counts, out = rows(db(os.path.join(R, 'bag')), {T_ST, T_JS, T_CS})
    res['bag_topics'] = {n: {'type': tmap.get(n), 'messages': counts.get(n, 0)}
                         for n in (T_ST, T_JS, T_WP, '/cmd_vel', T_CS, T_JT, T_FB)}
    res['bag_topics'][T_JT]['present_in_bag'] = T_JT in tmap
    res['bag_topics'][T_FB]['present_in_bag'] = T_FB in tmap
    sts = [(r, json.loads(m.data)) for r, m in out[T_ST]]
    ages = collections.defaultdict(float)
    states, last = [], None
    for r, st in sts:
        mon = st.get('monitors') or {}
        for k in ('joint_states_age_s', 'body_pose_age_s', 'graph_age_s'):
            if isinstance(mon.get(k), (int, float)):
                ages[k] = max(ages[k], mon[k])
        if st['state'] != last:
            states.append({'bag_receipt_unix_s': round(r, 3),
                           'sim_s_node_joint_state_stamp': mon.get('sim_time_s'),
                           'state': st['state']})
            last = st['state']
    gts = [st['ground_truth'] for _, st in sts if st.get('ground_truth')]
    res['status'] = {
        'messages': len(sts), 'modes': sorted({(st.get('mode'), st.get('dispatch_gate')) for _, st in sts}, key=str),
        'state_changes_seen_in_status': states,
        'max_ages_s': dict(ages), 'joint_states_margin_to_0.5s': JS_LIMIT - ages['joint_states_age_s'],
        'final': {k: sts[-1][1].get(k) for k in ('state', 'faults', 'command_rejections', 'goals_recorded', 'cycles', 'phases_completed', 'boundary')},
        'body': {'z_min': min(g['z'] for g in gts), 'z_max': max(g['z'] for g in gts),
                 'tilt_max': max(g['tilt'] for g in gts),
                 'xy_first': gts[0]['xy'], 'xy_last': gts[-1]['xy'],
                 'xy_travel_m': math.hypot(gts[-1]['xy'][0] - gts[0]['xy'][0], gts[-1]['xy'][1] - gts[0]['xy'][1]),
                 'yaw_change_rad': gts[-1]['yaw'] - gts[0]['yaw']}}
    res['joint_states_main'] = js_stats(out[T_JS])
    cs = out[T_CS]
    if cs:
        r0 = list(cs[0][1].reference.positions)
        dev = max(max((abs(a - b) for a, b in zip(m.reference.positions, r0)), default=0.0) for _, m in cs)
        res['controller_state'] = {'messages': len(cs), 'reference_points_nonempty': bool(r0),
                                   'max_reference_change_rad': dev}
    # ---- (no settle bag in this run: one recorder; settling is premotion/settle_check.json and
    #      the driver's own settle record)
    # ---- node evidence
    ev = [json.loads(l) for l in open(os.path.join(RUN, 'node', 'events.jsonl'))]
    c = collections.Counter(e['event'] for e in ev)
    res['node_events'] = {'counts': dict(c),
                          'transitions': [[e['previous'], e['state'], e.get('why')] for e in ev if e['event'] == 'transition'],
                          'cancel': [e for e in ev if e['event'] in ('cancel_sent', 'cancel_response')],
                          'results_not_succeeded': [e for e in ev if e['event'] == 'result' and e.get('status') != 4],
                          'faults': [e for e in ev if e['event'] == 'fault'],
                          'refused': [e for e in ev if e['event'] == 'refused']}
    for f in ('shadow_goals.json', 'final_status.json'):
        p = os.path.join(RUN, 'node', f)
        if os.path.exists(p):
            d = json.load(open(p))
            res['node_' + f] = (len(d) if isinstance(d, list) else {k: d.get(k) for k in ('state', 'mode', 'dispatch_gate', 'goals_recorded', 'faults', 'command_rejections', 'cycles')})
    # ---- launch log
    lg = open(os.path.join(RUN, 'launch.log'), errors='replace').read()
    pat = {'jtc_goal_received': r'\[leg_trajectory_controller\]: Received new action goal',
           'jtc_goal_accepted': r'\[leg_trajectory_controller\]: Accepted new action goal',
           'jtc_cancel_request': r'\[leg_trajectory_controller\]: Got request to cancel goal',
           'jtc_canceling': r'\[leg_trajectory_controller\]: Canceling active action goal',
           'terminate_called': r'terminate called', 'what': r'what\(\):', 'aborted': r'Aborted',
           'signal_sent_by': r'Signal sent by', 'process_died': r'process has died',
           'sigterm': r'SIGTERM', 'sigkill': r'SIGKILL', 'sending_signal': r'sending signal',
           'exception': r'[Ee]xception', 'error_lines': r'\[ERROR\]'}
    res['launch_log'] = {k: len(re.findall(v, lg)) for k, v in pat.items()}
    res['launch_log']['process_exits'] = re.findall(r'\[INFO\] \[([^\]]+)\]: process has finished cleanly \[pid \d+\]', lg)
    res['launch_log']['process_died_lines'] = re.findall(r'.*process has died.*', lg)
    res['launch_log']['error_lines'] = re.findall(r'.*\[ERROR\].*', lg)[:20]
    # ---- controller traffic and exits (no verdict here)
    def action_clients(f):
        try:
            t = open(os.path.join(RUN, 'captures', f)).read()
        except OSError:
            return None
        m = re.search(r'^Action clients: (\d+)$(.*?)^Action servers:', t, re.M | re.S)
        return None if m is None else {'count': int(m.group(1)),
                                       'client_nodes': re.findall(r'^    (\S+)$', m.group(2), re.M)}
    res['controller_traffic'] = {
        'bag_joint_trajectory_messages': res['bag_topics'][T_JT]['messages'],
        'bag_action_feedback_messages': res['bag_topics'][T_FB]['messages'],
        'controller_reference_max_change_rad': (res.get('controller_state') or {}).get('max_reference_change_rad'),
        'jtc_log_goal_or_cancel_lines': sum(res['launch_log'][k] for k in ('jtc_goal_received', 'jtc_goal_accepted', 'jtc_cancel_request', 'jtc_canceling')),
        'action_clients_before': action_clients('before_action_info.txt'),
        'action_clients_after': action_clients('after_action_info.txt')}
    res['exits'] = {
        'locomotion_node_lines': re.findall(r'.*\[m55_locomotion_node[^\]]*\].*(?:process has finished cleanly|process has died).*', lg),
        'server_exit_strace': open(os.path.join(RUN, 'server_exit_strace.txt')).read().strip().splitlines() if os.path.exists(os.path.join(RUN, 'server_exit_strace.txt')) else None}
    print(json.dumps(res, indent=1, sort_keys=True, default=str))


main()
