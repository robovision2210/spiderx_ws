"""colcon test: PR #16 corrective batch 2 - dispatch freshness and an interruptible confirmation.

No ROS graph. A deterministic fake clock (the mock transport's) makes every boundary exact:
  - readiness evidence is aged from the START of its observation; a slow collection, a slow
    action-server wait or a long confirmation pause can only lead to a refusal, never a retry;
  - freshness is re-checked immediately before the send, after every wait;
  - the live confirmation reader answers the interrupt latch without waiting for a newline, and
    EOF, a wrong word, a first or a second interrupt keep the approved refusal/cancel semantics.
"""
import os
import signal
import threading
import time

import pytest

from spiderx_controller import m6_goal_fingerprint as gf
from spiderx_controller import m6_live_contract as lc
from spiderx_controller import m6_live_mock as mock
from spiderx_controller import m6_live_playback as lpb
from spiderx_controller import m6_live_preflight as lp
from spiderx_controller import m6_live_readiness as rd
from spiderx_controller import m6_trajectory as m6t
from spiderx_controller.config_check import load_urdf

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SRC_CONFIG = os.path.join(PKG, 'config')
NAMES, NEUTRAL = lp.contract_from_config(SRC_CONFIG)
WORD = lc.CONFIRMATION_WORD + '\n'


@pytest.fixture(scope='module')
def sources():
    return m6t.load_sources(SRC_CONFIG, load_urdf())


@pytest.fixture(scope='module')
def traj(sources):
    return m6t.build_trajectory(sources)


@pytest.fixture(scope='module')
def fp(traj, sources):
    return gf.fingerprint(gf.approved_spec(traj, sources))


class Graph:
    def __init__(self):
        self.action_servers = [('/leg_trajectory_controller', [lp.env.ACTION_TYPE])]
        self.controllers = [('joint_state_broadcaster', 'x', 'active'),
                            ('leg_trajectory_controller', lp.JTC_TYPE, 'active')]
        self.joint_state_publishers = [('/joint_state_broadcaster', lp.JOINT_STATE_TYPE)]
        self.joint_state_messages = [(1.0 + 0.01 * i, list(NAMES), list(NEUTRAL))
                                     for i in range(4)]
        self.probe_errors = []


class SlowServer(mock.FakeTransport):
    """The action-server wait takes `server_wait_s` of (fake) wall time."""

    def __init__(self, *a, server_wait_s=0.0, **kw):
        super().__init__(*a, **kw)
        self.server_wait_s = server_wait_s

    def server_ready(self, timeout_s):
        self.t += self.server_wait_s
        return super().server_ready(timeout_s)


def session(sources, traj, fp, *, collect_s=(4.0, 4.0), server_wait_s=0.0, pause_s=0.0,
            reader=None, session_cls=lpb.LiveSession, latch=None, transport_cls=SlowServer,
            **tkw):
    """A LiveSession whose readiness collections take collect_s[i] seconds of fake time."""
    latch = latch or lpb.InterruptLatch()
    t = transport_cls(traj, fp, latch=latch, server_wait_s=server_wait_s, **tkw)
    calls = []

    def collect():
        t.t += collect_s[min(len(calls), len(collect_s) - 1)]
        calls.append(t.t)
        return Graph()
    iface = mock.mock_readiness_report(NAMES, NEUTRAL)['interface']
    provide = rd.make_readiness_provider(collect, NAMES, NEUTRAL, t.wall_now,
                                         versions=dict(mock.MOCK_VERSIONS), interface=iface)

    def pausing_reader():
        t.t += pause_s                                   # the operator takes pause_s to type
        return WORD
    s = session_cls(t, sources, provide, reader or pausing_reader, latch=latch)
    return s, t


# ---------------------------------------------------------------- truthful observation timing
@pytest.mark.parametrize('collect_s, ok', [(9.99, True), (10.0, True), (10.01, False),
                                           (11.0, False)])
def test_collection_time_counts_in_the_age(collect_s, ok):
    clock = [100.0]

    def collect():
        clock[0] += collect_s
        return Graph()
    iface = mock.mock_readiness_report(NAMES, NEUTRAL)['interface']
    r = rd.make_readiness_provider(collect, NAMES, NEUTRAL, lambda: clock[0],
                                   versions=dict(mock.MOCK_VERSIONS), interface=iface)()
    assert r.observed_monotonic == 100.0 and r.collection_s == pytest.approx(collect_s)
    assert rd.dispatch_permitted(r, clock[0]) == ((True, None) if ok
                                                  else (False, 'readiness_stale'))


def test_slow_first_collection_refuses_before_the_prompt(sources, traj, fp):
    asked = []
    s, t = session(sources, traj, fp, collect_s=(11.0,),
                   reader=lambda: asked.append(1) or WORD)
    out = s.run()
    assert out['reason'] == 'readiness_stale' and asked == [] and t.sent == []


def test_slow_second_collection_refuses_without_a_new_observation(sources, traj, fp):
    s, t = session(sources, traj, fp, collect_s=(4.0, 11.0))
    out = s.run()
    assert out['reason'] == 'readiness_stale' and t.sent == []
    assert [r['when'] for r in out['readiness']] == ['before_confirmation',
                                                     'after_confirmation']   # no third look


# ---------------------------------------------------------------- freshness at the send
@pytest.mark.parametrize('server_wait_s, sent, age', [(5.0, True, 9.0), (6.0, True, 10.0),
                                                      (6.05, False, 10.05), (7.0, False, 11.0)])
def test_server_wait_ages_the_evidence_until_the_send(sources, traj, fp, server_wait_s, sent,
                                                      age):
    s, t = session(sources, traj, fp, collect_s=(4.0, 4.0), server_wait_s=server_wait_s)
    out = s.run()
    assert ('server_ready', lc.SERVER_WAIT_S) in t.calls          # the wait happened first
    assert out['freshness_at_send']['age_s'] == pytest.approx(age)
    assert out['freshness_at_send']['permitted'] is sent
    if sent:
        assert len(t.sent) == 1 and out['state'] == lpb.SUCCEEDED
    else:
        assert t.sent == [] and out['reason'] == 'readiness_stale'
        assert out['state'] == lpb.REFUSED and out['dispatch']['attempted'] is False
        assert not any(c[0] == 'send_goal' for c in t.calls)


class NoSendCheckSession(lpb.LiveSession):
    """MUTANT: the freshness re-check immediately before the send is removed."""

    def _fresh_at_send(self):
        return True, None


def test_mutation_removed_send_freshness_check_is_detected(sources, traj, fp):
    s, t = session(sources, traj, fp, server_wait_s=7.0, session_cls=NoSendCheckSession)
    s.run()
    assert len(t.sent) == 1                    # detected: a goal on 11 s old evidence
    s2, t2 = session(sources, traj, fp, server_wait_s=7.0)
    assert s2.run()['reason'] == 'readiness_stale' and t2.sent == []


def test_interrupt_during_the_server_wait_refuses_at_the_send(sources, traj, fp):
    latch = lpb.InterruptLatch()

    class WaitInterrupted(SlowServer):
        def server_ready(self, timeout_s):
            latch.trigger()                     # Ctrl+C while waiting for the server
            return super().server_ready(timeout_s)
    s, t = session(sources, traj, fp, latch=latch, transport_cls=WaitInterrupted)
    out = s.run()
    assert out['reason'] == 'operator_interrupt' and t.sent == []


# ---------------------------------------------------------------- long confirmation pause
def test_long_pause_needs_the_fresh_second_observation(sources, traj, fp):
    s, t = session(sources, traj, fp, collect_s=(4.0, 4.0), pause_s=300.0)
    out = s.run()
    assert out['state'] == lpb.SUCCEEDED and len(t.sent) == 1
    assert out['freshness_at_send']['age_s'] == pytest.approx(4.0)   # only readiness #2 counts


def test_long_pause_then_slow_second_observation_refuses(sources, traj, fp):
    s, t = session(sources, traj, fp, collect_s=(4.0, 10.5), pause_s=300.0)
    out = s.run()
    assert out['reason'] == 'readiness_stale' and t.sent == []


# ---------------------------------------------------------------- interruptible reader
def pipe_with(text=None, close=True):
    r, w = os.pipe()
    if text:
        os.write(w, text.encode())
    if close:
        os.close(w)
        w = None
    return r, w


def test_reader_returns_on_a_latched_interrupt_without_a_newline():
    latch = lpb.InterruptLatch()
    r, w = pipe_with('SEND-ONE', close=False)            # partial input, no newline, no EOF
    read = lpb.interruptible_line_reader(latch, fd=r, poll_s=0.02)
    threading.Timer(0.2, latch.trigger).start()
    t0 = time.monotonic()
    with pytest.raises(KeyboardInterrupt):
        read()
    assert time.monotonic() - t0 < 1.0
    assert lc.parse_confirmation(lpb.interruptible_line_reader(latch, fd=r)) is False
    os.close(r)
    os.close(w)


def test_reader_answers_a_real_sigint_while_input_is_blocked():
    latch = lpb.InterruptLatch()
    latch.install()
    r, w = pipe_with(close=False)
    try:
        read = lpb.interruptible_line_reader(latch, fd=r, poll_s=0.02)
        threading.Timer(0.2, os.kill, (os.getpid(), signal.SIGINT)).start()
        t0 = time.monotonic()
        with pytest.raises(KeyboardInterrupt):
            read()
        assert latch.count == 1 and time.monotonic() - t0 < 1.0
    finally:
        latch.uninstall()
        os.close(r)
        os.close(w)


@pytest.mark.parametrize('text, want', [
    (WORD, WORD), ('SEND-ONE-CROUCH-GOAL', 'SEND-ONE-CROUCH-GOAL'), ('', ''),
    ('send-one-crouch-goal\nmore', 'send-one-crouch-goal\n'), ('x' * 1000, 'x' * 256),
])
def test_reader_matches_readline_up_to_the_length_cap(text, want):
    r, _ = pipe_with(text)
    try:
        assert lpb.interruptible_line_reader(lpb.InterruptLatch(), fd=r)() == want
    finally:
        os.close(r)


def scripted_input(steps):
    """select/read fakes: 'wait' = select times out; bytes = one readable byte."""
    polls = []

    def select_fn(r, w, x, timeout):
        polls.append(timeout)
        if steps and steps[0] == 'wait':
            steps.pop(0)
            return [], [], []
        return (r if steps else []), [], []

    def read_fn(fd, n):
        return steps.pop(0)
    return select_fn, read_fn, polls


def test_reader_is_deterministic_with_injected_select_and_read():
    steps = ['wait', b'S', 'wait', 'wait'] + [bytes([c]) for c in b'END-ONE-CROUCH-GOAL\n']
    select_fn, read_fn, polls = scripted_input(steps)
    read = lpb.interruptible_line_reader(lpb.InterruptLatch(), fd=0, poll_s=0.5,
                                         select_fn=select_fn, read_fn=read_fn)
    assert read() == WORD and set(polls) == {0.5} and len(polls) == 3 + len(WORD)

    latch = lpb.InterruptLatch()
    select_fn, read_fn, polls = scripted_input(['wait'] * 10)

    def select_then_interrupt(r, w, x, timeout):
        if len(polls) == 2:
            latch.trigger()                              # Ctrl+C during the third wait
        return select_fn(r, w, x, timeout)
    read = lpb.interruptible_line_reader(latch, fd=0, select_fn=select_then_interrupt,
                                         read_fn=read_fn)
    with pytest.raises(KeyboardInterrupt):
        read()
    assert len(polls) == 3                               # gave up at the next latch check


# ---------------------------------------------------------------- confirmation semantics
@pytest.mark.parametrize('text, triggers, reason', [
    ('', 0, 'confirmation_refused'),                     # EOF
    ('yes\n', 0, 'confirmation_refused'),                # wrong word
    (None, 1, 'operator_interrupt'),                     # first interrupt while blocked
    (None, 2, 'operator_interrupt'),                     # second interrupt while blocked
    (WORD, 1, 'operator_interrupt'),                     # typed, but interrupted: interrupt wins
])
def test_confirmation_refusals_send_nothing(sources, traj, fp, text, triggers, reason):
    latch = lpb.InterruptLatch()
    r, w = pipe_with(text, close=text is not None)
    for _ in range(triggers if text is not None else 0):
        latch.trigger()
    if text is None:
        for i in range(triggers):
            threading.Timer(0.1 + 0.05 * i, latch.trigger).start()
    reader = lpb.interruptible_line_reader(latch, fd=r, poll_s=0.02)
    s, t = session(sources, traj, fp, reader=reader, latch=latch)
    out = s.run()
    assert out['reason'] == reason and out['state'] == lpb.REFUSED
    assert t.sent == [] and t.cancels == 0
    os.close(r)
    if w is not None:
        os.close(w)


# dispatch happens at fake t = 8 s (two 4 s readiness collections); the goal runs to ~17 s
@pytest.mark.parametrize('interrupts, state', [((13.0,), lpb.CANCEL_CONFIRMED),
                                               ((13.0, 13.2), lpb.CANCEL_UNCONFIRMED),
                                               ((13.0, 13.01), lpb.CANCEL_UNCONFIRMED)])
def test_interrupts_after_dispatch_stay_cancel_only(sources, traj, fp, interrupts, state):
    """First interrupt: exactly one cancel. Second: stop waiting, no further command - even when
    both are latched within the same 50 ms poll (pre-existing ordering defect, fixed here)."""
    r, _ = pipe_with(WORD)
    latch = lpb.InterruptLatch()
    reader = lpb.interruptible_line_reader(latch, fd=r)
    kw = {'cancel_response': None} if len(interrupts) > 1 else {}
    s, t = session(sources, traj, fp, reader=reader, latch=latch, interrupts_at=interrupts,
                   **kw)
    out = s.run()
    os.close(r)
    assert out['state'] == state and len(t.sent) == 1 and t.cancels == 1
    assert out['cancel_reason'] == 'operator_interrupt'
    assert out['dispatch']['cancel_attempted'] and out['dispatch']['acceptance'] == 'accepted'
