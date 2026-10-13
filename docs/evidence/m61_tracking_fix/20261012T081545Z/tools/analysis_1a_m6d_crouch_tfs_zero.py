#!/usr/bin/env python3
"""READ-ONLY MODEL (question 1a): the UNCHANGED M6.0-D reference rule on the M6.0-D crouch.

No ROS graph, nothing sent; a model, not a measurement. It replays the approved M6.0-D crouch
(44f0a7ad52e5c330) with PERFECT tracking from acceptance, 100 Hz joint states and 20 Hz action
feedback, through the M6.0-D rule (m6_live_playback.LiveSession._ref_time: the median of
stamp - desired.time_from_start replaces acceptance when it differs by more than 0.1 s), twice:
  - with feedback time_from_start filled in (joint_trajectory_controller >= 2.49.0, as on the
    owner's PC, 2.54.0, where the crouch ran live and SUCCEEDED, 0.00603 rad in flight);
  - with time_from_start = 0 (2.48.0, as in this Cloud image).
"""
from spiderx_controller import m6_action_client as ac
from spiderx_controller import m6_live_contract as lc
from spiderx_controller import m6_trajectory as m6t

traj = m6t.build_trajectory(m6t.load_sources())


def run(tfs_mode):
    acc, offs, worst, first_abort = 0.0, [], 0.0, None
    t = 0.0
    while t <= 9.5:
        t = round(t + 0.01, 6)                                  # 100 Hz joint states, sim time
        if abs(t * 20 - round(t * 20)) < 1e-9:                  # 20 Hz action feedback
            tfs = 0.0 if tfs_mode == 'zero' else t
            offs.append(t - tfs)
        q = ac.reference_positions(traj, t) or traj['points'][0]['positions']   # perfect
        est = sorted(offs)[len(offs) // 2] if offs else None
        t0 = est if (est is not None and abs(est - acc) > lc.FEEDBACK_OFFSET_LIMIT_S) else acc
        ref = ac.reference_positions(traj, t - t0)
        if ref is None:
            continue
        e = max(abs(a - b) for a, b in zip(q, ref))
        worst = max(worst, e)
        if e > lc.CLIENT_TRACKING_ABORT_RAD and first_abort is None:
            first_abort = (t, round(t - t0, 3), round(e, 4))
    return worst, first_abort


print(f'trajectory {traj["trajectory_id"]}; abort limit {lc.CLIENT_TRACKING_ABORT_RAD} rad; '
      f'offset limit {lc.FEEDBACK_OFFSET_LIMIT_S} s')
for mode, label in (('correct', 'feedback time_from_start filled (>= 2.49.0)'),
                    ('zero', 'feedback time_from_start = 0 (2.48.0)')):
    worst, first = run(mode)
    print(f'{label}: max |err| {worst:.4f} rad; first abort (stamp s, ref time s, err rad): '
          f'{first}')
