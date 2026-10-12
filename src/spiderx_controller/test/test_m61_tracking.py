"""colcon test: the M6.1 in-flight tracking reference (m61_tracking + M61Session). Fakes only.

The second authorized Cloud cycle (docs/evidence/m61_cloud_one_cycle/20261012T054741Z) was
cancelled by the client's in-flight tracking check while the joints followed the approved
trajectory within 0.0108 rad: joint_trajectory_controller 2.48.0 reports
desired.time_from_start = 0 in its action feedback, and the unchanged M6.0-D rule then takes the
median feedback STAMP as the trajectory start, which drifts late. M6.1 now anchors the reference
at goal acceptance and lets only informative feedback (time_from_start > 0) move it; the
fallback is recorded. No threshold changes.

Covers: the anchor rule and its boundaries; the abort threshold boundary; a fake controller that
reports time_from_start = 0; a replay of the recorded cycle (its /joint_states and a recorded
controller-stream proxy for its feedback) that must not abort, while the unchanged M6.0-D rule on
the same replay does; genuinely bad tracks that still cancel; both gate modes (monkeypatched, in-
memory fakes). Nothing here reaches a ROS graph, a simulator or a controller.
"""
import hashlib
import inspect
import json
import math
import os

import pytest

from spiderx_controller import m6_gait_replay as gr
from spiderx_controller import m6_live_contract as lc
from spiderx_controller import m6_live_playback as lpb
from spiderx_controller import m61_evidence as evd
from spiderx_controller import m61_live_contract as c61
from spiderx_controller import m61_mock
from spiderx_controller import m61_tracking as trk
from spiderx_controller import m61_trot_cycle as tc
from spiderx_controller.config_check import load_urdf

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..')
SRC_CONFIG = os.path.join(PKG, 'config')
FIXTURE = os.path.join(HERE, 'data', 'm61_cycle_20261012T054741Z_tracking_replay.json')
EVIDENCE = os.path.join(PKG, '..', '..', 'docs', 'evidence', 'm61_cloud_one_cycle',
                        '20261012T054741Z')
TRAJECTORY_ID = '241760e7dfd5ef12'
CONTENT_SHA256 = '94a492c43fcc046125d6bc71ee7f1d9a8a5d8466f1a1bdd9958a16044dd58e64'
WORD = c61.CONFIRMATION_WORD + '\n'


@pytest.fixture(scope='module')
def plan():
    p = tc.load_plan(SRC_CONFIG, load_urdf())
    assert p.errors == (), p.errors
    return p


@pytest.fixture(scope='module')
def traj(plan):
    return tc.build_trajectory(plan)


@pytest.fixture(scope='module')
def fx():
    with open(FIXTURE) as f:
        return json.load(f)


@pytest.fixture
def disabled(monkeypatch):
    monkeypatch.setattr(c61, 'M61_LIVE_DISPATCH_ENABLED', False)      # tests only


@pytest.fixture
def enabled(monkeypatch):
    monkeypatch.setattr(c61, 'M61_LIVE_DISPATCH_ENABLED', True)       # tests only


# ==================================================================== thresholds (unchanged)
def test_no_threshold_was_loosened(plan):
    assert plan.limits.client_tracking_abort_rad == 0.05
    assert lc.CLIENT_TRACKING_ABORT_RAD == 0.05
    assert lc.FEEDBACK_OFFSET_LIMIT_S == 0.1
    default = inspect.signature(trk.tracking_anchor).parameters['offset_limit_s'].default
    assert default == lc.FEEDBACK_OFFSET_LIMIT_S
    assert plan.limits.path_position_tolerance_rad == 0.05
    assert plan.limits.goal_position_tolerance_rad == 0.05


def test_the_m6d_rule_is_unchanged():
    """M6.0-D keeps its own rule (median of stamp - time_from_start over ALL feedback)."""
    s = lpb.LiveSession.__new__(lpb.LiveSession)
    s._t_accept_sim = 10.0
    s._feedback_offsets = [10.0 + 0.1 * k for k in range(21)]     # stamps, time_from_start 0
    assert s._start_estimate() == pytest.approx(11.0)
    assert s._ref_time(14.0) == pytest.approx(3.0)                 # the drift M6.1 no longer has


# ==================================================================== the anchor rule
def fb_zero(stamps):
    return [(st, 0.0) for st in stamps]


def test_before_acceptance_there_is_no_anchor():
    t0, rec = trk.tracking_anchor(None, fb_zero([1.0, 2.0]))
    assert t0 is None and rec['anchor'] is None and rec['t0_sim_s'] is None


def test_no_feedback_anchors_at_acceptance_without_fallback():
    t0, rec = trk.tracking_anchor(10.0, [])
    assert t0 == 10.0 and rec['anchor'] == trk.ACCEPTANCE
    assert rec['fallback_time_from_start_zero'] is False and rec['note'] is None
    assert trk.tracking_anchor(10.0, None)[0] == 10.0


@pytest.mark.parametrize('elapsed', [0.05, 0.5, 3.216, 7.0, 60.0])
def test_zero_time_from_start_never_moves_the_anchor(elapsed):
    stamps = [10.0 + elapsed * k / 50 for k in range(51)]
    t0, rec = trk.tracking_anchor(10.0, fb_zero(stamps))
    assert t0 == 10.0 and rec['anchor'] == trk.ACCEPTANCE
    assert rec['feedback_estimate_s'] is None and rec['estimate_offset_s'] is None
    assert rec['fallback_time_from_start_zero'] is True and rec['note'] == trk.FALLBACK_NOTE
    assert (rec['feedback_samples'], rec['informative_samples'],
            rec['non_informative_samples']) == (51, 0, 51)


@pytest.mark.parametrize('tfs, informative', [
    (0.0, False), (-0.0, False), (trk.TFS_MIN_S, False), (-0.5, False), (math.nan, False),
    (math.inf, False), (-math.inf, False), (None, False), ('1.0', False),
    (2 * trk.TFS_MIN_S, True), (0.02, True), (3.0, True),
])
def test_informative_boundaries(tfs, informative):
    assert trk.informative(tfs) is informative
    t0, rec = trk.tracking_anchor(10.0, [(15.0, tfs)])
    assert rec['informative_samples'] == (1 if informative else 0)
    assert rec['fallback_time_from_start_zero'] is (not informative)
    if not informative:
        assert t0 == 10.0                                    # never an estimate


def test_a_non_finite_stamp_is_never_an_estimate():
    for st in (math.nan, math.inf, None):
        t0, rec = trk.tracking_anchor(10.0, [(st, 1.0)])
        assert t0 == 10.0 and rec['informative_samples'] == 0
        assert rec['fallback_time_from_start_zero'] is True


def test_informative_feedback_agreeing_with_acceptance_keeps_acceptance():
    fbk = [(10.0 + 0.05 * k - 0.015, 0.05 * k) for k in range(1, 40)]  # started 15 ms earlier
    t0, rec = trk.tracking_anchor(10.0, fbk)
    assert t0 == 10.0 and rec['anchor'] == trk.ACCEPTANCE
    assert rec['feedback_estimate_s'] == pytest.approx(9.985)
    assert rec['fallback_time_from_start_zero'] is False and rec['note'] is None


def test_informative_feedback_reporting_another_start_moves_the_anchor():
    fbk = [(10.5 + 0.05 * k, 0.05 * k) for k in range(1, 40)]          # controller: +0.5 s
    t0, rec = trk.tracking_anchor(10.0, fbk)
    assert t0 == pytest.approx(10.5) and rec['anchor'] == trk.FEEDBACK_ESTIMATE
    assert rec['estimate_offset_s'] == pytest.approx(0.5)


def test_offset_limit_boundary_is_exclusive():
    # binary-exact values: an estimate exactly at the limit keeps acceptance
    assert trk.tracking_anchor(2.0, [(3.125, 1.0)], 0.125)[1]['anchor'] == trk.ACCEPTANCE
    assert trk.tracking_anchor(2.0, [(2.875, 1.0)], 0.125)[1]['anchor'] == trk.ACCEPTANCE
    assert trk.tracking_anchor(2.0, [(3.25, 1.0)], 0.125)[1]['anchor'] == trk.FEEDBACK_ESTIMATE
    assert trk.tracking_anchor(2.0, [(2.75, 1.0)], 0.125)[1]['anchor'] == trk.FEEDBACK_ESTIMATE
    # the default limit (0.1 s), clearly inside and outside
    assert trk.tracking_anchor(10.0, [(11.09, 1.0)])[1]['anchor'] == trk.ACCEPTANCE
    assert trk.tracking_anchor(10.0, [(11.11, 1.0)])[1]['anchor'] == trk.FEEDBACK_ESTIMATE


def test_mixed_feedback_uses_only_the_informative_messages():
    zeros = fb_zero([10.0 + 0.05 * k for k in range(100)])             # would drift 2.5 s
    good = [(10.0 + 0.05 * k, 0.05 * k) for k in range(1, 4)]
    t0, rec = trk.tracking_anchor(10.0, zeros + good)
    assert t0 == 10.0 and rec['feedback_estimate_s'] == pytest.approx(10.0)
    assert (rec['informative_samples'], rec['non_informative_samples']) == (3, 100)
    assert rec['fallback_time_from_start_zero'] is False


def test_median_is_the_m6d_upper_median():
    fbk = [(11.0, 1.0), (13.0, 1.0), (12.0, 1.0), (14.0, 1.0)]         # estimates 10..13
    assert trk.tracking_anchor(0.0, fbk)[1]['feedback_estimate_s'] == 12.0


# ==================================================================== session harness (fakes)
class StubTransport:
    """The goal is accepted at sim `accept`; records cancels. Nothing is sent anywhere."""

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


def accepted_session(plan, traj, accept, cls=gr.M61Session):
    st = StubTransport(accept)
    s = cls(st, plan, lambda: None, lambda: '')
    s._t0, s._trajectory, s.state = 0.0, traj, lpb.GOAL_PENDING
    mon = lpb.StreamMonitor(0.0)
    s._on_event(('goal_response', True, 'goal-1'), mon)
    assert s.state == lpb.ACTIVE and s._t_accept_sim == accept
    return s, st, mon


def feed(s, st, mon, events):
    """events: [(stamp, event)] in arrival order; returns the stamp of the first cancel."""
    cancel_at = None
    for stamp, ev in events:
        st.t = stamp
        s._on_event(ev, mon)
        if st.cancels and cancel_at is None:
            cancel_at = stamp
    return cancel_at


def js(traj, stamp, q):
    return (stamp, ('joint_state', stamp, stamp, list(traj['joint_names']), list(q)))


def feedback(stamp, tfs):
    return (stamp, ('feedback', {'stamp': stamp, 'desired_time_from_start': tfs}))


def test_abort_threshold_boundary_at_the_first_tracked_instant(plan, traj):
    zero = [0.0] * 12
    assert tc.reference_positions(traj, 3.0) == pytest.approx(zero, abs=0.0)
    assert tc.reference_positions(traj, 2.999) is None             # lead-in: untracked
    for err, cancels in ((0.05, 0), (math.nextafter(0.05, 1.0), 1)):
        s, st, mon = accepted_session(plan, traj, 100.0)
        feed(s, st, mon, [feedback(100.5, 0.0), js(traj, 102.999, [1.0] * 12),
                          js(traj, 103.0, [err] + zero[1:])])
        assert st.cancels == cancels and s.cancels_sent == cancels
        assert s._max_inflight_error == err
        if cancels:
            assert s.reason == 'tracking_error' and s.state == lpb.CANCEL_REQUESTED


def test_zero_time_from_start_no_longer_postpones_the_check(plan, traj):
    """With time_from_start = 0 the M6.0-D estimate sits at about half the elapsed time, so a
    real error at the first tracked instant goes unchecked; acceptance checks it."""
    events = [feedback(100.0 + 0.05 * k, 0.0) for k in range(61)]       # up to 103.0
    events.append(js(traj, 103.0, [0.06] + [0.0] * 11))
    s, st, mon = accepted_session(plan, traj, 100.0)
    assert feed(s, st, mon, events) == 103.0 and s.reason == 'tracking_error'
    old, ost, omon = accepted_session(plan, traj, 100.0, M6dRuleSession)
    assert feed(old, ost, omon, events) is None and old._max_inflight_error is None


def test_fallback_is_recorded_once_and_counted(plan, traj):
    s, st, mon = accepted_session(plan, traj, 100.0)
    events = []
    for k in range(1, 700):                                   # lead-in and tracked part
        t = 100.0 + 0.01 * k
        if k % 5 == 0:
            events.append(feedback(t, 0.0))
        events.append(js(traj, t, tc.reference_positions(traj, t - 100.0) or [0.0] * 12))
    assert feed(s, st, mon, events) is None and st.cancels == 0
    assert s._max_inflight_error is not None and s._max_inflight_error < 1e-9
    notes = [e for e in s.events if trk.FALLBACK_NOTE in e[2]]
    assert len(notes) == 1 and notes[0][0] == pytest.approx(100.05)  # first use, not before
    rec = s.outcome()['tracking_reference']
    assert rec['fallback_time_from_start_zero'] is True and rec['fallback_logged'] is True
    assert rec['fallback_inflight_checks'] == 699 - 4                 # every check after fb #1
    assert rec['anchor'] == trk.ACCEPTANCE and rec['t0_sim_s'] == 100.0
    assert rec['unfiltered_m6d_estimate_s'] > 100.0 + lc.FEEDBACK_OFFSET_LIMIT_S
    assert rec['abort_threshold_rad'] == 0.05
    assert s.outcome()['tracking']['feedback_start_estimate_s'] is None
    json.dumps(s.outcome(), allow_nan=False)


def test_informative_feedback_needs_no_fallback(plan, traj):
    s, st, mon = accepted_session(plan, traj, 100.0)
    events = []
    for k in range(1, 400):
        t = 100.0 + 0.01 * k
        if k % 5 == 0:
            events.append(feedback(t, t - 100.0))
        events.append(js(traj, t, tc.reference_positions(traj, t - 100.0) or [0.0] * 12))
    assert feed(s, st, mon, events) is None
    rec = s.outcome()['tracking_reference']
    assert rec['fallback_time_from_start_zero'] is False and rec['fallback_logged'] is False
    assert rec['fallback_inflight_checks'] == 0 and rec['informative_samples'] == 79
    assert not [e for e in s.events if trk.FALLBACK_NOTE in e[2]]
    assert s.outcome()['tracking']['feedback_offset_s'] == pytest.approx(0.0, abs=1e-9)


def test_feedback_that_turns_informative_keeps_the_fallback_record(plan, traj):
    s, st, mon = accepted_session(plan, traj, 100.0)
    feed(s, st, mon, [feedback(100.05, 0.0), js(traj, 100.06, [0.0] * 12),
                      feedback(100.10, 0.10), js(traj, 100.11, [0.0] * 12)])
    rec = s.tracking_reference()
    assert rec['fallback_time_from_start_zero'] is False                # now informative
    assert rec['fallback_logged'] is True and rec['fallback_inflight_checks'] == 1


def test_unreadable_feedback_fields_are_counted_not_used(plan, traj):
    assert all(math.isnan(v) for v in gr.feedback_timing({}))
    assert all(math.isnan(v) for v in gr.feedback_timing(None))
    s, st, mon = accepted_session(plan, traj, 100.0)
    feed(s, st, mon, [(100.1, ('feedback', {'stamp': 100.1})),
                      (100.2, ('feedback', {'stamp': 'x', 'desired_time_from_start': 1.0})),
                      (100.3, ('feedback', {'stamp': 100.3, 'desired_time_from_start': None})),
                      js(traj, 100.4, [0.0] * 12)])
    rec = s.tracking_reference()
    assert rec['feedback_samples'] == 3 and rec['informative_samples'] == 0
    assert rec['t0_sim_s'] == 100.0 and rec['fallback_time_from_start_zero'] is True


# ==================================================================== replay of the recorded cycle
def test_fixture_is_the_recorded_cycle(fx, traj):
    assert fx['trajectory_id'] == traj['trajectory_id'] == TRAJECTORY_ID
    assert fx['trajectory_content_sha256'] == CONTENT_SHA256
    assert fx['joint_names'] == list(traj['joint_names'])
    rec = fx['recorded']
    assert (rec['state'], rec['reason'], rec['goals_sent'], rec['cancels_sent']) == \
        (lpb.TRACKING_FAILED, 'tracking_error', 1, 1)
    assert rec['feedback_start_estimate_s'] == 33.03 and rec['last_joint_stamp_s'] == 36.03
    assert len(fx['joint_states']) == 624
    fbk = fx['feedback_proxy']['samples']
    assert len(fbk) == 255 and {t for _, t in fbk} == {0.0}
    lo, hi = fx['feedback_proxy']['window_s']
    assert all(lo <= st <= hi for st, _ in fbk)


def test_fixture_matches_the_committed_evidence(fx):
    if not os.path.isdir(EVIDENCE):
        pytest.skip('evidence tree not present (installed test run)')
    for rel, digest in fx['source_sha256'].items():
        with open(os.path.join(EVIDENCE, rel), 'rb') as f:
            assert hashlib.sha256(f.read()).hexdigest() == digest, rel


def replay_events(fx, joint_rows=None, feedback_rows=None):
    names = fx['joint_names']
    rows = fx['joint_states'] if joint_rows is None else joint_rows
    fbk = fx['feedback_proxy']['samples'] if feedback_rows is None else feedback_rows
    evs = [(r[0], 1, ('joint_state', r[0], r[0], names, r[1:])) for r in rows]
    evs += [(st, 0, ('feedback', {'stamp': st, 'desired_time_from_start': t})) for st, t in fbk]
    evs.sort(key=lambda e: (e[0], e[1]))
    return [(st, ev) for st, _, ev in evs]


def replay(plan, traj, fx, cls=gr.M61Session, **kw):
    s, st, mon = accepted_session(plan, traj, fx['recorded']['accept_sim_s'], cls)
    return s, st, feed(s, st, mon, replay_events(fx, **kw))


def test_replay_of_the_recorded_cycle_does_not_abort(plan, traj, fx):
    s, st, cancel_at = replay(plan, traj, fx)
    assert cancel_at is None and st.cancels == 0 and s.cancels_sent == 0
    assert s.state == lpb.ACTIVE and s.reason is None
    assert s._max_inflight_error == pytest.approx(0.010756, abs=1e-6)  # offline analysis
    rec = s.outcome()['tracking_reference']
    assert rec['anchor'] == trk.ACCEPTANCE and rec['t0_sim_s'] == fx['recorded']['accept_sim_s']
    assert rec['fallback_time_from_start_zero'] is True and rec['fallback_inflight_checks'] > 600
    assert len([e for e in s.events if trk.FALLBACK_NOTE in e[2]]) == 1
    rows = s.evidence_rows()['commanded_vs_observed']
    assert rows and rows[0][1] == pytest.approx(3.0, abs=0.01)         # tracked from 3.0 s
    assert rows[-1][1] == pytest.approx(6.216, abs=1e-6)


def test_replay_with_the_m6d_rule_reproduces_the_defect(plan, traj, fx):
    """Control: the unchanged M6.0-D rule on the same replay cancels a good track. (The proxy
    is not the recorded feedback stream, so the cancel stamp differs from the recorded 36.03.)"""
    s, st, cancel_at = replay(plan, traj, fx, cls=M6dRuleSession)
    assert st.cancels == 1 and s.reason == 'tracking_error'
    assert cancel_at is not None and cancel_at <= fx['recorded']['last_joint_stamp_s']
    assert s._max_inflight_error > 0.05
    assert s._start_estimate() - fx['recorded']['accept_sim_s'] > lc.FEEDBACK_OFFSET_LIMIT_S


def _rows_with(fx, fn):
    acc = fx['recorded']['accept_sim_s']
    return [[r[0]] + fn(r[0] - acc, list(r[1:])) for r in fx['joint_states']]


@pytest.mark.parametrize('case', ['offset_after_4s', 'one_sample_spike', 'joints_never_move',
                                  'lag_0_5s', 'joint_missing'])
def test_a_genuinely_bad_track_still_cancels(plan, traj, fx, case):
    acc = fx['recorded']['accept_sim_s']
    if case == 'offset_after_4s':
        rows = _rows_with(fx, lambda rt, q: [q[0] + (0.06 if rt >= 4.0 else 0.0)] + q[1:])
    elif case == 'one_sample_spike':
        rows = _rows_with(fx, lambda rt, q: [q[0] + (0.06 if abs(rt - 5.0) < 0.005 else 0.0)]
                          + q[1:])
    elif case == 'joints_never_move':
        rows = _rows_with(fx, lambda rt, q: [0.0] * 12)
    elif case == 'lag_0_5s':
        rows = [[r[0] + 0.5] + r[1:] for r in fx['joint_states']]
    else:
        rows = [r if r[0] - acc < 4.5 else r[:1] + [math.nan] + r[2:] for r in fx['joint_states']]
    s, st, cancel_at = replay(plan, traj, fx, joint_rows=rows)
    assert st.cancels == 1 and s.cancels_sent == 1 and s.reason == 'tracking_error', case
    assert s.state == lpb.CANCEL_REQUESTED and cancel_at is not None
    assert s.tracking_reference()['anchor'] == trk.ACCEPTANCE


# ==================================================================== fake controller, both modes
def word():
    return lambda: WORD


def test_mock_controller_reporting_zero_time_from_start(plan):
    out, session, _ = gr.run_mock(plan, 'feedback_tfs_zero', word())
    assert out['state'] == lpb.SUCCEEDED and out['cancels_sent'] == 0
    assert out['mock_server_goals_received'] == 1 and out['mock_server_cancels_received'] == 0
    rec = out['tracking_reference']
    assert rec['fallback_time_from_start_zero'] is True and rec['fallback_inflight_checks'] > 0
    assert rec['informative_samples'] == 0 and rec['feedback_samples'] > 100
    assert rec['unfiltered_m6d_estimate_s'] - rec['accept_sim_s'] > 1.0    # would have drifted
    assert out['tracking']['max_inflight_error_rad'] < 0.01
    assert len([e for e in out['events'] if trk.FALLBACK_NOTE in e[2]]) == 1


def test_mock_controller_zero_time_from_start_fails_with_the_m6d_rule(plan, monkeypatch):
    """Control: the same fake run with the pre-fix rule ends TRACKING_FAILED (one cancel)."""
    monkeypatch.setattr(gr.M61Session, '_ref_time', lpb.LiveSession._ref_time)
    monkeypatch.setattr(gr.M61Session, '_start_estimate', lpb.LiveSession._start_estimate)
    out, _, _ = gr.run_mock(plan, 'feedback_tfs_zero', word())
    assert out['state'] == lpb.TRACKING_FAILED and out['cancels_sent'] == 1


def test_mock_bad_track_with_zero_time_from_start_still_cancels(plan):
    out, _, _ = gr.run_mock(plan, 'tracking_error_tfs_zero', word())
    assert (out['state'], out['reason']) == (lpb.TRACKING_FAILED, 'tracking_error')
    assert out['goals_sent'] == 1 and out['cancels_sent'] == 1
    assert out['mock_server_cancels_received'] == 1
    assert out['tracking']['max_inflight_error_rad'] > 0.05


def test_mock_informative_feedback_records_no_fallback(plan):
    out, _, _ = gr.run_mock(plan, 'success', word())
    rec = out['tracking_reference']
    assert out['state'] == lpb.SUCCEEDED and rec['fallback_time_from_start_zero'] is False
    assert rec['anchor'] == trk.ACCEPTANCE and rec['informative_samples'] > 100
    assert abs(rec['estimate_offset_s']) <= lc.FEEDBACK_OFFSET_LIMIT_S


class Graph:
    """Read-only observations of a healthy stack (no ROS)."""

    def __init__(self, names, neutral):
        from spiderx_controller import m6_live_preflight as lpf
        self.action_servers = [('/leg_trajectory_controller', [lpf.env.ACTION_TYPE])]
        self.controllers = [('joint_state_broadcaster', 'x', 'active'),
                            ('leg_trajectory_controller', lpf.JTC_TYPE, 'active')]
        self.joint_state_publishers = [('/joint_state_broadcaster', lpf.JOINT_STATE_TYPE)]
        self.joint_state_messages = [(1.0 + 0.01 * i, list(names), list(neutral))
                                     for i in range(4)]
        self.probe_errors = []


class Factories:
    def __init__(self, plan, **kw):
        self.plan, self.kw, self.made = plan, kw, []

    def transport(self, fp, domain_id, fixed_base):
        t = m61_mock.M61FakeTransport(tc.build_trajectory(self.plan), fp, fixed_base=fixed_base,
                                      **self.kw)
        self.made.append(t)
        return t

    def collect(self, transport):
        return lambda: Graph(self.plan.sources.joint_names, self.plan.neutral)


def run_live(plan, tmp_path, f, utc):
    args = gr.parse_args(['--live', '--domain-id', '42', '--out', str(tmp_path)])
    return gr._live_main(args, plan, word(), transport_factory=f.transport,
                         collect_factory=f.collect, utc_stamp=utc)


def test_disabled_gate_refuses_a_zero_tfs_controller_first(disabled, plan, tmp_path):
    f = Factories(plan, feedback_tfs_zero=True)
    assert run_live(plan, tmp_path, f, '20261012T090000Z') == gr.EXIT_DISABLED
    assert f.made == [] and os.listdir(tmp_path) == []
    with pytest.raises(PermissionError):
        gr._run_live(plan, f.transport, f.collect, word(), lpb.InterruptLatch(), 42)
    assert f.made == []


def test_enabled_live_wiring_tracks_a_zero_tfs_controller(enabled, plan, tmp_path):
    f = Factories(plan, feedback_tfs_zero=True)
    assert run_live(plan, tmp_path, f, '20261012T090001Z') == gr.EXIT_OK
    out = json.loads((tmp_path / '20261012T090001Z' / evd.LIVE_OUTCOME).read_text())
    assert out['state'] == lpb.SUCCEEDED and out['goals_sent'] == 1 and out['cancels_sent'] == 0
    assert len(f.made) == 1 and len(f.made[0].sent) == 1 and f.made[0].cancels == 0
    rec = out['tracking_reference']
    assert rec['fallback_time_from_start_zero'] is True and rec['anchor'] == trk.ACCEPTANCE
    assert len([e for e in out['events'] if trk.FALLBACK_NOTE in e[2]]) == 1


def test_enabled_live_wiring_still_cancels_a_bad_track(enabled, plan, tmp_path):
    f = Factories(plan, **m61_mock.SCENARIOS['tracking_error_tfs_zero'])
    assert run_live(plan, tmp_path, f, '20261012T090002Z') == gr.EXIT_FAILED
    out = json.loads((tmp_path / '20261012T090002Z' / evd.LIVE_OUTCOME).read_text())
    assert out['state'] == lpb.TRACKING_FAILED and out['reason'] == 'tracking_error'
    assert len(f.made[0].sent) == 1 and f.made[0].cancels == 1
    assert out['automatic_return_goals'] == 0 and out['retries'] == 0
