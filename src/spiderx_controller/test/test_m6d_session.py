"""colcon test: M6.0-D Batch B - LiveSession state machine against a deterministic fake transport.

No ROS runtime: the fake transport (m6d_fake_transport) simulates the server, the stream and both
clocks. Proves every dispatch gate, at most one goal, at most one cancel, cancel-only/hold, the
result/timeout/interrupt mappings and the D6/D7/D8 stream rules, plus mutation tests showing each
safety check can fail.
"""
import copy
import os

import m6d_fake_transport as ft
import pytest

from spiderx_controller import m6_action_client as ac
from spiderx_controller import m6_goal_fingerprint as gf
from spiderx_controller import m6_live_contract as lc
from spiderx_controller import m6_live_playback as lpb
from spiderx_controller import m6_live_preflight as lp
from spiderx_controller import m6_live_readiness as rd
from spiderx_controller import m6_trajectory as m6t
from spiderx_controller.config_check import load_urdf

SRC_CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config')
NAMES, NEUTRAL = lp.contract_from_config(SRC_CONFIG)
GOOD_INTERFACE = {'action_file_sha256': 'x',
                  'goal_fields': ['goal_time_tolerance', 'goal_tolerance', 'path_tolerance',
                                  'trajectory'],
                  'tolerance_fields': ['acceleration', 'name', 'position', 'velocity'],
                  'error_codes': dict(lp.REQUIRED_ERROR_CODES)}


@pytest.fixture(scope='module')
def sources():
    return m6t.load_sources(SRC_CONFIG, load_urdf())


@pytest.fixture(scope='module')
def traj(sources):
    return m6t.build_trajectory(sources)


@pytest.fixture(scope='module')
def fp(traj, sources):
    return gf.fingerprint(gf.approved_spec(traj, sources))


def preflight_report(**kw):
    o = lp.Observations(
        action_servers=[('/leg_trajectory_controller', [lp.env.ACTION_TYPE])],
        controllers=[('joint_state_broadcaster', 'x', 'active'),
                     ('leg_trajectory_controller', lp.JTC_TYPE, 'active')],
        joint_state_publishers=[('/joint_state_broadcaster', lp.JOINT_STATE_TYPE)],
        joint_state_messages=[(1.0 + 0.01 * i, list(NAMES), [0.0] * 12) for i in range(4)],
        versions=dict(lp.REFERENCE_VERSIONS, rclpy='3.3.19'), interface=dict(GOOD_INTERFACE))
    for k, v in kw.items():
        setattr(o, k, v)
    return lp.evaluate(o, NAMES, NEUTRAL)


def provider(transport, age=0.0, report=None, sequence=None):
    """Readiness observed `age` s before the call (fresh by default)."""
    calls = []

    def provide():
        calls.append(1)
        if sequence is not None:
            item = sequence[min(len(calls), len(sequence)) - 1]
            if item is None:
                return None
            if item == 'ready_but_incompatible':
                return rd.ReadinessResult(ready=True, label=rd.INCOMPATIBLE, reasons=('x',),
                                          failure_codes=(),
                                          observed_monotonic=transport.wall_now())
            a, rep = item
            return rd.assess(rep, transport.wall_now() - a)
        return rd.assess(report or preflight_report(), transport.wall_now() - age)
    provide.calls = calls
    return provide


def confirm(word=lc.CONFIRMATION_WORD):
    return lambda: word + '\n'


def make(sources, traj, fp, *, prov=None, reader=None, latch=None, **tkw):
    latch = latch or lpb.InterruptLatch()
    t = ft.FakeTransport(traj, fp, latch=latch, **tkw)
    s = lpb.LiveSession(t, sources, prov or provider(t), reader or confirm(), latch=latch)
    return s, t


def run(sources, traj, fp, **kw):
    s, t = make(sources, traj, fp, **kw)
    return s, t, s.run()


# ---------------------------------------------------------------- success path
def test_success_path_sends_exactly_one_approved_goal(sources, traj, fp):
    s, t, out = run(sources, traj, fp)
    assert out['state'] == lpb.SUCCEEDED and out['passed']
    assert out['channels'] == {'action': 'passed', 'tracking': 'passed',
                               'goal_tolerances': 'passed'}
    assert len(t.sent) == 1 and t.cancels == 0
    assert out['goals_sent'] == 1 and out['cancels_sent'] == 0
    assert out['retries'] == 0 and out['automatic_return_goals'] == 0
    goal = t.sent[0]
    assert gf.verify_goal(goal, gf.binding(traj), fp) == fp
    assert all(x.velocity == 0.05 for x in goal.goal_tolerance)
    assert (goal.trajectory.header.stamp.sec, goal.trajectory.header.stamp.nanosec) == (0, 0)
    assert out['goal_fingerprint'] == fp
    # only the acceptance-latency misalignment (<= one 0.05 s poll x 0.061 rad/s peak rate)
    assert out['tracking']['max_inflight_error_rad'] < 0.0062
    states = [e[1] for e in out['events']]
    for st in (lpb.NEW, lpb.PREFLIGHTED, lpb.CONFIRMED, lpb.DISPATCHING, lpb.GOAL_PENDING,
               lpb.ACTIVE):
        assert st in states
    assert [r['when'] for r in out['readiness']] == ['before_confirmation', 'after_confirmation']


def test_outcome_is_deterministic(sources, traj, fp):
    a = run(sources, traj, fp)[2]
    b = run(sources, traj, fp)[2]
    assert gf.json.dumps(a, sort_keys=True) == gf.json.dumps(b, sort_keys=True)


def test_session_is_single_use(sources, traj, fp):
    s, t, _ = run(sources, traj, fp)
    with pytest.raises(ac.SecondGoalForbidden):
        s.run()
    with pytest.raises(ac.SecondGoalForbidden):
        s._dispatch(t.sent[0])
    assert len(t.sent) == 1


# ---------------------------------------------------------------- dispatch gates
def refusal_cases(sources, traj, fp):
    bad_sources = m6t.Sources(config_dir='x', errors=(('neutral_source_missing', 'x'),))
    return {
        'offline_preflight': dict(src=bad_sources),
        'readiness_stale': dict(prov_age=10.5),
        'readiness_incompatible': dict(report=preflight_report(action_servers=[])),
        'readiness_not_ready': dict(report=preflight_report(
            joint_state_messages=[(1.0 + i, NAMES, [0.0] * 11 + [0.2]) for i in range(3)])),
        'readiness_missing': dict(sequence=[None]),
        'readiness_incompatible_label': dict(sequence=['ready_but_incompatible']),
        'readiness_stale_after_confirmation': dict(sequence=[(0.0, preflight_report()),
                                                             (11.0, preflight_report())]),
        'readiness_incompatible_after_confirmation': dict(sequence=[
            (0.0, preflight_report()), (0.0, preflight_report(action_servers=[]))]),
        'confirmation_wrong': dict(word='yes'),
        'confirmation_case': dict(word=lc.CONFIRMATION_WORD.lower()),
        'confirmation_empty': dict(word=''),
        'server_not_ready': dict(tkw={'ready': False}),
        'interrupt_before_confirmation': dict(pre_interrupt=True),
        'transport_fingerprint': dict(tkw_fp='0' * 64),
    }


def run_case(sources, traj, fp, case, session_cls=lpb.LiveSession):
    src = case.get('src', sources)
    latch = lpb.InterruptLatch()
    t = ft.FakeTransport(traj, case.get('tkw_fp', fp), latch=latch, **case.get('tkw', {}))
    prov = provider(t, age=case.get('prov_age', 0.0), report=case.get('report'),
                    sequence=case.get('sequence'))
    if case.get('pre_interrupt'):
        latch.trigger()
    s = session_cls(t, src, prov, confirm(case.get('word', lc.CONFIRMATION_WORD)), latch=latch)
    return t, s.run()


def gate_violations(sources, traj, fp, session_cls=lpb.LiveSession):
    out = []
    for name, case in refusal_cases(sources, traj, fp).items():
        t, res = run_case(sources, traj, fp, case, session_cls)
        if t.sent:
            out.append(f'{name}: a goal reached the server')
        if t.cancels:
            out.append(f'{name}: cancel sent')
        if res['state'] != lpb.REFUSED or res['goals_sent'] != 0:
            out.append(f'{name}: state {res["state"]}')
    return out


def test_every_dispatch_gate_refuses_with_no_action_call(sources, traj, fp):
    assert gate_violations(sources, traj, fp) == []


@pytest.mark.parametrize('name, code', [
    ('offline_preflight', 'preflight_refused'), ('readiness_stale', 'readiness_stale'),
    ('readiness_incompatible', 'stack_incompatible'),
    ('readiness_not_ready', 'readiness_not_ready'), ('readiness_missing', 'readiness_missing'),
    ('readiness_incompatible_label', 'stack_incompatible'),
    ('readiness_stale_after_confirmation', 'readiness_stale'),
    ('readiness_incompatible_after_confirmation', 'stack_incompatible'),
    ('confirmation_wrong', 'confirmation_refused'), ('confirmation_case', 'confirmation_refused'),
    ('confirmation_empty', 'confirmation_refused'),
    ('server_not_ready', 'action_server_unavailable'),
    ('interrupt_before_confirmation', 'operator_interrupt'),
    ('transport_fingerprint', 'goal_fingerprint_mismatch'),
])
def test_refusal_codes(name, code, sources, traj, fp):
    t, res = run_case(sources, traj, fp, refusal_cases(sources, traj, fp)[name])
    assert res['reason'] == code and res['state'] == lpb.REFUSED
    assert not any(c[0] == 'send_goal' for c in t.calls) or name == 'transport_fingerprint'
    assert t.sent == []


def test_confirmation_eof_and_interrupt_refuse(sources, traj, fp):
    for exc in (EOFError(), KeyboardInterrupt()):
        def reader(e=exc):
            raise e
        s, t = make(sources, traj, fp, reader=reader)
        assert s.run()['reason'] == 'confirmation_refused' and t.sent == []


def test_warning_classification_allows_dispatch(sources, traj, fp):
    s, t, out = run(sources, traj, fp)
    assert out['readiness'][0]['result']['classification'] == 'warning'
    assert out['state'] == lpb.SUCCEEDED


# ---------------------------------------------------------------- result mapping
@pytest.mark.parametrize('code, tol', [(-1, 'unavailable'), (-2, 'unavailable'),
                                       (-3, 'unavailable'), (-4, 'failed'), (-5, 'failed')])
def test_abort_codes(code, tol, sources, traj, fp):
    s, t, out = run(sources, traj, fp, result='abort', result_code=code, result_at=5.0)
    assert out['state'] == lpb.ABORTED and out['result']['error_code'] == code
    assert out['result']['error_name'] == ac.ERROR_CODES[code]
    assert out['channels']['action'] == 'failed' and out['channels']['goal_tolerances'] == tol
    assert len(t.sent) == 1 and t.cancels == 0


def test_rejection(sources, traj, fp):
    s, t, out = run(sources, traj, fp, accept=False)
    assert out['state'] == lpb.REJECTED and len(t.sent) == 1 and t.cancels == 0


def test_goal_response_timeout(sources, traj, fp):
    s, t, out = run(sources, traj, fp, respond=False)
    assert out['state'] == lpb.TIMED_OUT and out['reason'] == 'goal_response_timeout'
    assert t.cancels == 0


def test_external_cancel_result_is_not_success(sources, traj, fp):
    s, t, out = run(sources, traj, fp, result='canceled', result_at=6.0)
    assert out['state'] == lpb.ABORTED and t.cancels == 0


def test_result_watchdog_cancels_once(sources, traj, fp):
    s, t, out = run(sources, traj, fp, result=None)
    assert out['state'] == lpb.TIMED_OUT and out['cancel_reason'] == 'result_watchdog'
    assert t.cancels == 1 and len(t.sent) == 1


def test_send_failure_is_held_error(sources, traj, fp):
    class Boom(ft.FakeTransport):
        def send_goal(self, goal, binding):
            raise RuntimeError('wire down')
    latch = lpb.InterruptLatch()
    t = Boom(traj, fp, latch=latch)
    out = lpb.LiveSession(t, sources, provider(t), confirm(), latch).run()
    assert out['state'] == lpb.HELD_ERROR and out['goals_sent'] == 0


# ---------------------------------------------------------------- interrupts
def test_first_interrupt_cancels_once_and_confirms(sources, traj, fp):
    s, t, out = run(sources, traj, fp, interrupts_at=(5.0,))
    assert out['state'] == lpb.CANCEL_CONFIRMED and out['cancel_reason'] == 'operator_interrupt'
    assert t.cancels == 1 and len(t.sent) == 1


def test_interrupt_without_cancel_response_is_unconfirmed(sources, traj, fp):
    s, t, out = run(sources, traj, fp, interrupts_at=(5.0,), cancel_response=None)
    assert out['state'] == lpb.CANCEL_UNCONFIRMED and t.cancels == 1


def test_refused_cancel_is_unconfirmed(sources, traj, fp):
    s, t, out = run(sources, traj, fp, interrupts_at=(5.0,), cancel_response=1)
    assert out['state'] == lpb.CANCEL_UNCONFIRMED and t.cancels == 1


def test_second_interrupt_stops_waiting_without_new_command(sources, traj, fp):
    s, t, out = run(sources, traj, fp, interrupts_at=(5.0, 5.5), cancel_response=None)
    assert out['state'] == lpb.CANCEL_UNCONFIRMED
    assert t.cancels == 1 and len(t.sent) == 1


def test_interrupt_while_pending_cancels_on_acceptance(sources, traj, fp):
    s, t, out = run(sources, traj, fp, interrupts_at=(0.01,), response_latency=1.0)
    assert out['state'] == lpb.CANCEL_CONFIRMED and t.cancels == 1


# ---------------------------------------------------------------- stream rules (D6-D8)
@pytest.mark.parametrize('kw, state, reason', [
    (dict(js_stop_at=5.0), lpb.READINESS_LOST, 'joint_states_stale'),
    (dict(stamp_freeze_at=2.0), lpb.READINESS_LOST, 'sim_time_stalled'),
    (dict(gap_at=5.0), lpb.TRACKING_FAILED, 'sample_gap'),
    (dict(extra_pub_at=5.0), lpb.READINESS_LOST, 'extra_joint_state_publisher'),
    (dict(server_lost_at=5.0), lpb.HELD_ERROR, 'controller_lost'),
    (dict(error_after=4.0, error_value=0.051), lpb.TRACKING_FAILED, 'tracking_error'),
])
def test_stream_rules_cancel_once(kw, state, reason, sources, traj, fp):
    s, t, out = run(sources, traj, fp, **kw)
    assert out['state'] == state and out['cancel_reason'] == reason
    assert t.cancels == 1 and len(t.sent) == 1


def test_tracking_error_below_limit_passes(sources, traj, fp):
    s, t, out = run(sources, traj, fp, error_after=4.0, error_value=0.044)
    assert out['state'] == lpb.SUCCEEDED and t.cancels == 0
    assert 0.043 < out['tracking']['max_inflight_error_rad'] < 0.05


def test_sim_stall_needs_more_than_five_seconds(sources, traj, fp):
    assert lc.SIM_STALL_S == 5.0
    s, t, out = run(sources, traj, fp, stamp_freeze_at=1.0)
    ev = [e for e in out['events'] if 'sim_time_stalled' in e[2]]
    assert ev and 6.0 < ev[0][0] < 6.6


def test_transport_poll_error_cancels(sources, traj, fp):
    s, t, out = run(sources, traj, fp, poll_raises_at=5.0)
    assert out['state'] == lpb.HELD_ERROR and t.cancels == 1


def test_feedback_offset_estimate_is_reported(sources, traj, fp):
    s, t, out = run(sources, traj, fp, feedback_offset=-0.3)
    assert abs(out['tracking']['feedback_offset_s'] - 0.3) < 0.11
    assert out['state'] == lpb.SUCCEEDED


def test_stream_monitor_pure_rules():
    m = lpb.StreamMonitor(0.0)
    m.on_sample(0.1, 10.0)
    assert m.check(0.55) is None and m.check(0.61) == 'joint_states_stale'
    m.on_sample(0.62, 10.2)
    m.on_sample(0.63, 10.46)
    assert m.check(0.64) == 'sample_gap'
    assert lpb.graph_violation({'action_server_present': False}) == 'controller_lost'
    assert lpb.graph_violation({'action_server_present': True,
                                'joint_state_publishers': 2}) == 'extra_joint_state_publisher'
    assert lpb.graph_violation({'action_server_present': True,
                                'joint_state_publishers': 1}) is None


# ---------------------------------------------------------------- mutation tests
def test_mutation_removed_freshness_gate(sources, traj, fp, monkeypatch):
    real = rd.dispatch_permitted
    monkeypatch.setattr(rd, 'dispatch_permitted',
                        lambda r, now, max_age_s=0: real(r, r.observed_monotonic if r else now))
    assert any('readiness_stale' in v for v in gate_violations(sources, traj, fp))


def test_mutation_removed_incompatible_gate(sources, traj, fp, monkeypatch):
    real = rd.dispatch_permitted

    def no_label_check(r, now, max_age_s=lc.READINESS_MAX_AGE_S):
        if r is not None and r.fresh(now, max_age_s) and r.ready:
            return True, None
        return real(r, now, max_age_s)
    monkeypatch.setattr(rd, 'dispatch_permitted', no_label_check)
    assert any('incompatible_label' in v for v in gate_violations(sources, traj, fp))


def test_mutation_bypassed_confirmation(sources, traj, fp, monkeypatch):
    monkeypatch.setattr(lc, 'parse_confirmation', lambda reader: True)
    assert any('confirmation' in v for v in gate_violations(sources, traj, fp))


class ChangedGoalSession(lpb.LiveSession):
    """MUTANT: alters the goal after construction (e.g. a different end pose)."""

    def _dispatch(self, goal):
        g = copy.deepcopy(goal)
        g.trajectory.points[2].positions[0] = 0.01
        return super()._dispatch(g)


def test_mutation_changed_fingerprint_is_blocked_by_the_transport(sources, traj, fp):
    latch = lpb.InterruptLatch()
    t = ft.FakeTransport(traj, fp, latch=latch)
    out = ChangedGoalSession(t, sources, provider(t), confirm(), latch).run()
    assert out['reason'] == 'goal_fingerprint_mismatch' and t.sent == []
    # ...and the check is what blocks it: an unverifying transport lets it through
    t2 = ft.FakeTransport(traj, fp, latch=latch, verify=False)
    ChangedGoalSession(t2, sources, provider(t2), confirm(), latch).run()
    with pytest.raises(gf.FingerprintError):
        gf.verify_goal(t2.sent[0], gf.binding(traj), fp)


class RetryingSession(lpb.LiveSession):
    """MUTANT: re-sends after a rejection."""

    def _on_event(self, ev, monitor):
        out = super()._on_event(ev, monitor)
        if self.state == lpb.REJECTED:
            self.transport.send_goal(self.transport.sent[0], gf.binding(self._trajectory))
        return out


class AutoReturnSession(lpb.LiveSession):
    """MUTANT: sends a return-to-neutral goal after a cancel."""

    def _finish_cancel(self, confirmed, why):
        g = copy.deepcopy(self.transport.sent[0])
        del g.trajectory.points[1:]
        self.transport.send_goal(g, gf.binding(self._trajectory))
        return super()._finish_cancel(confirmed, why)


class DuplicateCancelSession(lpb.LiveSession):
    """MUTANT: re-sends the cancel when the first one was not confirmed."""

    def _finish_cancel(self, confirmed, why):
        if not confirmed:
            self.transport.cancel_goal()
        return super()._finish_cancel(confirmed, why)


def single_command_violations(sources, traj, fp, session_cls, **tkw):
    """Across failure scenarios: at most one goal (the approved one) and at most one cancel."""
    out = []
    scenarios = {'rejected': dict(accept=False), 'interrupt': dict(interrupts_at=(5.0, 5.2)),
                 'cancel_lost': dict(interrupts_at=(5.0,), cancel_response=None),
                 'stale': dict(js_stop_at=5.0, interrupts_at=(5.3,)),
                 'tracking': dict(error_after=4.0, error_value=0.06, interrupts_at=(5.5,))}
    for name, kw in scenarios.items():
        latch = lpb.InterruptLatch()
        t = ft.FakeTransport(traj, fp, latch=latch, **{**tkw, **kw})
        try:
            session_cls(t, sources, provider(t), confirm(), latch).run()
        except Exception as e:  # noqa: BLE001
            out.append(f'{name}: raised {type(e).__name__}')
        if len(t.sent) > 1:
            out.append(f'{name}: {len(t.sent)} goals')
        if t.cancels > 1:
            out.append(f'{name}: {t.cancels} cancels')
        for g in t.sent:
            try:
                gf.verify_goal(g, gf.binding(traj), fp)
            except gf.FingerprintError:
                out.append(f'{name}: non-approved goal sent')
    return out


def test_unmutated_session_has_no_extra_commands(sources, traj, fp):
    assert single_command_violations(sources, traj, fp, lpb.LiveSession) == []


@pytest.mark.parametrize('mutant', [RetryingSession, AutoReturnSession, DuplicateCancelSession])
def test_mutation_retry_return_or_duplicate_cancel_is_detected(mutant, sources, traj, fp):
    # with an unprotected transport the mutant's extra command reaches the "server"
    assert single_command_violations(sources, traj, fp, mutant, verify=False,
                                     single_use=False) != []


@pytest.mark.parametrize('mutant', [RetryingSession, AutoReturnSession])
def test_protected_transport_blocks_retry_and_return(mutant, sources, traj, fp):
    v = single_command_violations(sources, traj, fp, mutant)
    assert all('goals' not in x and 'non-approved' not in x for x in v)


def test_mutation_second_dispatch_without_guard(sources, traj, fp):
    s, t, _ = run(sources, traj, fp, single_use=False)
    s._dispatch_attempted = False                          # MUTANT: guard removed
    s._dispatch(t.sent[0])
    assert len(t.sent) == 2                                # detected: two goals at the server
    s2, t2, _ = run(sources, traj, fp)
    with pytest.raises(ac.SecondGoalForbidden):            # real session and transport refuse
        s2._dispatch(t2.sent[0])


def test_session_source_has_no_return_or_retry_path():
    import inspect
    src = inspect.getsource(lpb)
    assert src.count('self.transport.send_goal(') == 1
    assert src.count('self.transport.cancel_goal()') == 1
    for word in ('NEUTRAL_LABEL', 'build_trajectory(self.sources)\n' * 2, 'while retry'):
        assert word not in src
