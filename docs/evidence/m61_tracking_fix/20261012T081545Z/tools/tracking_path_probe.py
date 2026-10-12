#!/usr/bin/env python3
"""READ-ONLY rehearsal of the M6.1 in-flight tracking path against a running simulation.

DISABLED build only (refuses unless both live gates are False). No goal exists at any point: the
probe node has subscriptions only - no action client, no publisher of its own - and the sessions
it drives have an in-memory stub transport with no ROS object at all.

What runs on LIVE data:
  - /joint_states through the production adapter conversion (m6_live_adapter
    RclpyLiveTransport._on_joint_state), in arrival order;
  - a FEEDBACK PROXY: there is no action feedback without a goal, so each live
    /leg_trajectory_controller/controller_state message is copied into a FollowJointTrajectory
    feedback message (header, joint_names, desired = reference, actual = feedback, error) and
    converted by the production adapter (RclpyLiveTransport._on_feedback);
  - those events, after a pseudo acceptance at the sim time of the first joint state 2 s into
    the window, through M61Session._on_event (feedback capture, m61_tracking anchor, reference,
    in-flight comparison) and, as a CONTROL, the same session class with the unchanged M6.0-D
    reference rule.
The robot holds neutral (nothing is commanded), so once the approved reference leaves neutral by
more than 0.05 rad the in-flight check MUST trip: that is a genuinely bad track, and the expected
instant is computed independently. A trip calls only the stub's in-memory cancel counter.

Not shown here (needs a goal): the controller's real action feedback during a goal, the
controller's start relative to acceptance in this launch, goal tolerances, any motion.
Usage: tracking_path_probe.py --domain-id 0 --duration 40 --out <dir>
"""
import argparse
import json
import os
import sys
import time

from spiderx_controller import m6_gait_replay as gr
from spiderx_controller import m6_live_contract as lc
from spiderx_controller import m6_live_playback as lpb
from spiderx_controller import m61_live_contract as c61
from spiderx_controller import m61_tracking as trk
from spiderx_controller import m61_trot_cycle as tc

SCHEMA = 'spiderx.m61.tracking_path_probe/1'
NODE = 'm61_tracking_path_probe'
JS_TOPIC = '/joint_states'
CS_TOPIC = '/leg_trajectory_controller/controller_state'
STATUS_TOPIC = '/leg_trajectory_controller/follow_joint_trajectory/_action/status'
CMD_TOPIC = '/leg_trajectory_controller/joint_trajectory'
SETTLE_S = 2.0


class Holder:
    """Stands in for the transport object the adapter conversions append to."""

    def __init__(self):
        self._events, self._last_stamp = [], None


class StubTransport:
    """In-memory only: the pseudo acceptance time and a cancel COUNTER. No ROS object."""

    def __init__(self, accept):
        self.t, self.accept, self.cancels = 0.0, accept, 0

    def wall_now(self):
        return self.t

    def sim_now(self):
        return self.accept

    def cancel_goal(self):
        self.cancels += 1

    def poll(self, timeout_s):
        return []


class M6dRuleSession(gr.M61Session):
    """CONTROL ONLY: M61Session with the unchanged M6.0-D reference start (the pre-fix rule)."""
    _ref_time = lpb.LiveSession._ref_time
    _start_estimate = lpb.LiveSession._start_estimate


def secs(t):
    return t.sec + t.nanosec * 1e-9


def collect(domain_id, duration):
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from control_msgs.action import FollowJointTrajectory
    from control_msgs.msg import JointTrajectoryControllerState
    from rclpy.qos import qos_profile_action_status_default
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory

    from spiderx_controller import m6_live_adapter as la

    if os.environ.get('ROS_DOMAIN_ID', '0') != str(domain_id):
        raise SystemExit(f'ROS_DOMAIN_ID {os.environ.get("ROS_DOMAIN_ID")} != --domain-id')
    holder = Holder()
    raw = {'joint_states': 0, 'controller_state': 0, 'status_msgs': 0, 'max_goals_in_status': 0,
           'command_msgs': 0, 'cs_points': set(), 'cs_stamps': []}
    rclpy.init()
    node = rclpy.create_node(NODE)
    try:
        def on_js(msg):
            raw['joint_states'] += 1
            la.RclpyLiveTransport._on_joint_state(holder, msg)

        def on_cs(msg):
            raw['controller_state'] += 1
            for f in ('desired', 'reference'):
                p = getattr(msg, f)
                raw['cs_points'].add((f, len(p.positions), secs(p.time_from_start)))
            raw['cs_stamps'].append(secs(msg.header.stamp))
            fbm = FollowJointTrajectory.Impl.FeedbackMessage()
            fbm.feedback.header = msg.header
            fbm.feedback.joint_names = list(msg.joint_names)
            fbm.feedback.desired = msg.reference
            fbm.feedback.actual = msg.feedback
            fbm.feedback.error = msg.error
            la.RclpyLiveTransport._on_feedback(holder, fbm)

        def on_status(msg):
            raw['status_msgs'] += 1
            raw['max_goals_in_status'] = max(raw['max_goals_in_status'], len(msg.status_list))

        def on_cmd(msg):
            raw['command_msgs'] += 1

        node.create_subscription(JointState, JS_TOPIC, on_js, 50)
        node.create_subscription(JointTrajectoryControllerState, CS_TOPIC, on_cs, 50)
        node.create_subscription(GoalStatusArray, STATUS_TOPIC, on_status,
                                 qos_profile_action_status_default)
        node.create_subscription(JointTrajectory, CMD_TOPIC, on_cmd, 10)
        t_start = time.monotonic()
        while time.monotonic() - t_start < duration:
            rclpy.spin_once(node, timeout_sec=0.05)
        own = {'publishers': sorted(n for n, _ in node.get_publisher_names_and_types_by_node(
                   NODE, '/')),
               'service_clients': sorted(n for n, _ in node.get_client_names_and_types_by_node(
                   NODE, '/')),
               'subscriptions': sorted(n for n, _ in
                                       node.get_subscriber_names_and_types_by_node(NODE, '/'))}
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raw['cs_points'] = sorted([list(x) for x in raw['cs_points']])
    return holder._events, raw, own, t_start


def drive(cls, plan, traj, events, accept):
    st = StubTransport(accept)
    s = cls(st, plan, lambda: None, lambda: '')
    s._t0, s._trajectory, s.state = 0.0, traj, lpb.GOAL_PENDING
    mon = lpb.StreamMonitor(0.0)
    s._on_event(('goal_response', True, 'pseudo-acceptance-no-goal'), mon)
    trip, first_check = None, None
    for ev in events:
        st.t = ev[1] if ev[0] == 'joint_state' else st.t
        before = s._max_inflight_error
        s._on_event(ev, mon)
        if ev[0] == 'joint_state' and first_check is None and s._max_inflight_error is not None \
                and before is None:
            first_check = {'stamp_s': ev[2], 'ref_time_s': round(s._ref_time(ev[2]), 6)}
        if st.cancels and trip is None:
            trip = {'stamp_s': ev[2], 'ref_time_s': round(s._ref_time(ev[2]), 6),
                    'max_inflight_error_rad': s._max_inflight_error, 'reason': s.reason}
    return {'session_goals_sent': s.goals_sent, 'stub_cancel_calls_in_memory': st.cancels,
            'state': s.state, 'first_tracked_check': first_check, 'first_trip': trip,
            'max_inflight_error_rad': s._max_inflight_error,
            'start_estimate_s': s._start_estimate(),
            'estimate_minus_acceptance_s': (None if s._start_estimate() is None
                                            else round(s._start_estimate() - accept, 6)),
            'tracking_reference': (s.tracking_reference() if cls is gr.M61Session else None),
            'fallback_events': [e[2] for e in s.events if trk.FALLBACK_NOTE in e[2]]}


def predicted_trip(traj, names, q_obs):
    """First reference time (1 ms grid) at which the approved reference leaves the static
    observation by more than the 0.05 rad abort limit (independent of the session code)."""
    k = 3000
    while k <= 7000:
        ref = tc.reference_positions(traj, k / 1000.0)
        if ref is not None:
            e = max(abs(q_obs[n] - r) for n, r in zip(traj['joint_names'], ref))
            if e > 0.05:
                return k / 1000.0, e
        k += 1
    return None, None


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument('--domain-id', type=int, required=True)
    p.add_argument('--duration', type=float, default=40.0)
    p.add_argument('--out', required=True)
    a = p.parse_args(argv)
    if c61.M61_LIVE_DISPATCH_ENABLED or lc.LIVE_DISPATCH_ENABLED:
        print('REFUSED: the tracking-path probe runs on the DISABLED build only')
        return 3
    plan = tc.load_plan()
    traj = tc.build_trajectory(plan)
    events, raw, own, t_start = collect(a.domain_id, a.duration)
    js = [e for e in events if e[0] == 'joint_state']
    fbk = [e for e in events if e[0] == 'feedback']
    rec = {'schema': SCHEMA, 'gates': gr._gate_state(), 'domain_id': a.domain_id,
           'duration_wall_s': a.duration, 'trajectory_id': traj['trajectory_id'],
           'goals_sent': 0, 'action_clients_created': 0, 'raw': raw, 'own_graph': own,
           'non_claims': ['no goal was sent; no motion; the action feedback is a PROXY built '
                          'from controller_state, not feedback the controller sent for a goal']}
    try:
        from spiderx_controller import m6_live_preflight as lpf
        rec['versions'] = lpf.collect_versions()
    except Exception as e:  # noqa: BLE001
        rec['versions'] = {'error': str(e)}
    if not js or not fbk:
        rec['verdict'] = 'NO DATA'
        _write(a.out, rec)
        return 1
    names = list(traj['joint_names'])
    neutral = dict(zip(plan.sources.joint_names, plan.neutral))
    dev = max(abs(v - neutral[n]) for e in js for n, v in zip(e[3], e[4]) if n in neutral)
    first_wall = js[0][1]
    acc_ev = next(e for e in js if e[1] - first_wall >= SETTLE_S)
    accept = acc_ev[2]
    i0 = events.index(acc_ev)
    after = events[i0 + 1:]
    q_obs = dict(zip(acc_ev[3], acc_ev[4]))
    rt_pred, e_pred = predicted_trip(traj, names, q_obs)
    js_after = [e for e in after if e[0] == 'joint_state']
    fb_after = [e for e in after if e[0] == 'feedback']
    sim_span = js[-1][2] - js[0][2]
    wall_span = js[-1][1] - js[0][1]
    rec.update({
        'joint_state_events': len(js), 'feedback_proxy_events': len(fbk),
        'adapter_feedback_time_from_start_values': sorted({e[1]['desired_time_from_start']
                                                           for e in fbk}),
        'max_abs_deviation_from_neutral_rad': dev,
        'sim_span_s': round(sim_span, 6), 'wall_span_s': round(wall_span, 6),
        'real_time_factor': round(sim_span / wall_span, 4) if wall_span > 0 else None,
        'pseudo_acceptance_sim_s': accept,
        'events_after_acceptance': {'joint_state': len(js_after), 'feedback': len(fb_after)},
        'sim_after_acceptance_s': round(js_after[-1][2] - accept, 6) if js_after else 0.0,
        'predicted_trip_ref_time_s': rt_pred, 'predicted_trip_error_rad': e_pred,
        'm61_rule': drive(gr.M61Session, plan, traj, after, accept),
        'm6d_rule_control': drive(M6dRuleSession, plan, traj, after, accept),
    })
    new, old = rec['m61_rule'], rec['m6d_rule_control']
    checks = {
        'no_goal_in_action_status': raw['max_goals_in_status'] == 0,
        'no_command_messages': raw['command_msgs'] == 0,
        'probe_has_no_action_client': not any('_action/' in c for c in own['service_clients']),
        'robot_held_neutral': dev < 1e-3,
        'live_time_from_start_zero': rec['adapter_feedback_time_from_start_values'] == [0.0],
        'anchor_is_acceptance': (new['tracking_reference'] or {}).get('t0_sim_s') == accept,
        'fallback_recorded_once': len(new['fallback_events']) == 1 and
        (new['tracking_reference'] or {}).get('fallback_time_from_start_zero') is True,
        'tracked_from_3s': (new['first_tracked_check'] or {}).get('ref_time_s', -1) >= 3.0 and
        (new['first_tracked_check'] or {}).get('ref_time_s', 99) < 3.05,
        'non_moving_robot_trips_as_predicted': (
            new['first_trip'] is not None and rt_pred is not None and
            abs(new['first_trip']['ref_time_s'] - rt_pred) < 0.05),
        'm6d_rule_estimate_drifted': (old['estimate_minus_acceptance_s'] or 0.0) >
        lc.FEEDBACK_OFFSET_LIMIT_S,
    }
    rec['checks'] = checks
    rec['verdict'] = 'PASS' if all(checks.values()) else 'FAIL'
    _write(a.out, rec)
    print(f'verdict {rec["verdict"]}; ' + ', '.join(f'{k}={v}' for k, v in checks.items()))
    return 0 if rec['verdict'] == 'PASS' else 1


def _write(out, rec):
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, 'tracking_path_probe.json'), 'w') as f:
        json.dump(rec, f, indent=1, allow_nan=False, default=str)
        f.write('\n')


if __name__ == '__main__':
    sys.exit(main())
