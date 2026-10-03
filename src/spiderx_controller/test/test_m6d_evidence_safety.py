"""colcon test: PR #16 corrective batch 1 - live evidence persistence and exception safety.

No ROS graph: the gate is set True only inside these tests (monkeypatch; the committed gate stays
False), and every run uses the in-memory mock transport with injected failures at the output
reservation, the initial record, the pre-send checkpoint, the dispatch/acceptance boundary,
polling, cancellation, the graph check, simulation-time access, the final serialization/write and
session setup. Each test asserts what was sent and cancelled and which uncertainty is retained.
"""
import itertools
import json
import os

import pytest

from spiderx_controller import m6_live_adapter as la
from spiderx_controller import m6_live_contract as lc
from spiderx_controller import m6_live_mock as mock
from spiderx_controller import m6_live_playback as lpb
from spiderx_controller import m6_live_preflight as lp
from spiderx_controller import m6_trajectory as m6t
from spiderx_controller.config_check import load_urdf

PKG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SRC_CONFIG = os.path.join(PKG, 'config')
NAMES, NEUTRAL = lp.contract_from_config(SRC_CONFIG)
_STAMPS = itertools.count(1)


@pytest.fixture(scope='module')
def sources():
    return m6t.load_sources(SRC_CONFIG, load_urdf())


@pytest.fixture(autouse=True)
def enabled(monkeypatch):
    monkeypatch.setattr(lc, 'LIVE_DISPATCH_ENABLED', True)        # tests only


class Graph:
    def __init__(self):
        self.action_servers = [('/leg_trajectory_controller', [lp.env.ACTION_TYPE])]
        self.controllers = [('joint_state_broadcaster', 'x', 'active'),
                            ('leg_trajectory_controller', lp.JTC_TYPE, 'active')]
        self.joint_state_publishers = [('/joint_state_broadcaster', lp.JOINT_STATE_TYPE)]
        self.joint_state_messages = [(1.0 + 0.01 * i, list(NAMES), list(NEUTRAL))
                                     for i in range(4)]
        self.probe_errors = []


class Run:
    """One gated --live run against an injected transport class (mock; no ROS)."""

    def __init__(self, sources, tmp_path, transport_cls=mock.FakeTransport, evidence=None,
                 latch=None, **transport_kw):
        self.sources, self.tmp_path = sources, tmp_path
        self.transport_cls, self.transport_kw = transport_cls, transport_kw
        self.evidence, self.latch = evidence, latch
        self.made = []
        self.stamp = f'20261003T{next(_STAMPS):06d}Z'

    def transport(self, fp, domain_id):
        kw = dict(self.transport_kw)
        if self.latch is not None:
            kw['latch'] = self.latch
        t = self.transport_cls(m6t.build_trajectory(self.sources), fp, **kw)
        self.made.append(t)
        return t

    def __call__(self, *extra, reader=None):
        args = lpb.parse_args(['--live', '--domain-id', '0', '--out', str(self.tmp_path),
                               *extra])
        return lpb._live_main(args, self.sources,
                              reader or (lambda: lc.CONFIRMATION_WORD + '\n'),
                              transport_factory=self.transport,
                              collect_factory=lambda t: (lambda: Graph()),
                              latch=self.latch, utc_stamp=self.stamp,
                              evidence_factory=self.evidence)

    @property
    def path(self):
        return self.tmp_path / 'live' / self.stamp / 'live_outcome.json'

    def report(self):
        return json.loads(self.path.read_text())


class FakeEvidence:
    """Records every document; raises on the write numbers listed in fail_on (1-based)."""

    def __init__(self, path, fail_on=(), error=OSError('injected write failure')):
        self.path, self.fail_on, self.error = path, set(fail_on), error
        self.docs, self.closed = [], False

    def write(self, doc):
        self.docs.append(json.loads(json.dumps(doc, default=str)))
        if len(self.docs) in self.fail_on:
            raise self.error

    def checkpoint(self, outcome):
        self.write(dict(outcome, evidence={'path': self.path, 'phase': 'pre_send'}))

    def close(self):
        self.closed = True


def fake_evidence(store, **kw):
    def make(path):
        store.append(FakeEvidence(path, **kw))
        return store[-1]
    return make


def never(*a, **k):
    raise AssertionError('must not be reached')


# ---------------------------------------------------------------- refusals before ROS
def test_no_write_is_refused_with_live_before_anything(sources, tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(m6t, 'load_sources', never)
    args = lpb.parse_args(['--live', '--domain-id', '0', '--no-write', '--out', str(tmp_path)])
    assert lpb._live_main(args, None, never, never, never, evidence_factory=never) == 2
    assert '--no-write is not allowed' in capsys.readouterr().out
    assert os.listdir(tmp_path) == []


def test_existing_evidence_refuses_and_is_untouched(sources, tmp_path):
    r = Run(sources, tmp_path)
    r.path.parent.mkdir(parents=True)
    r.path.write_text('another run\n')
    r.transport = never
    assert r() == lpb.EXIT_REFUSED
    assert r.path.read_text() == 'another run\n'


def test_unreservable_output_refuses_before_ros(sources, tmp_path, capsys):
    blocker = tmp_path / 'not_a_dir'
    blocker.write_text('x')
    r = Run(sources, blocker)                           # makedirs under a file must fail
    r.transport = never
    assert r() == lpb.EXIT_REFUSED
    assert 'cannot reserve live evidence' in capsys.readouterr().out


def test_initial_record_failure_refuses_before_ros(sources, tmp_path, capsys):
    store = []
    r = Run(sources, tmp_path, evidence=fake_evidence(store, fail_on={1}))
    r.transport = never
    assert r() == lpb.EXIT_REFUSED
    assert 'NOT_DISPATCHED evidence could not be saved' in capsys.readouterr().out
    assert store[0].closed and len(store[0].docs) == 1


def test_not_dispatched_record_exists_before_any_ros_work(sources, tmp_path):
    seen = []
    r = Run(sources, tmp_path)
    real = r.transport

    def transport(fp, domain_id):
        seen.append(json.loads(r.path.read_text()))   # on disk before the transport exists
        return real(fp, domain_id)
    r.transport = transport
    assert r() == lpb.EXIT_OK
    first = seen[0]
    assert first['state'] == 'NOT_DISPATCHED' and first['evidence']['phase'] == 'reserved'
    assert first['dispatch']['attempted'] is False and first['goals_sent'] == 0
    final = r.report()
    assert final['evidence']['phase'] == 'final' and final['state'] == lpb.SUCCEEDED


def test_evidence_file_is_exclusive_and_written_through_its_own_descriptor(tmp_path):
    path = str(tmp_path / 'a' / 'live_outcome.json')
    ev = lpb.EvidenceFile.reserve(path)
    with pytest.raises(FileExistsError):
        lpb.EvidenceFile.reserve(path)
    ev.write({'x': 1, 'long': 'y' * 100})
    ev.write({'x': 2})                                   # shorter rewrite truncates
    assert json.loads(open(path).read()) == {'x': 2}
    os.rename(path, path + '.moved')                     # someone moves the file away...
    open(path, 'w').write('other evidence\n')            # ...and another run's file appears
    ev.write({'x': 3})
    assert open(path).read() == 'other evidence\n'       # never overwritten by this run
    assert json.loads(open(path + '.moved').read()) == {'x': 3}
    ev.close()
    with pytest.raises(ValueError):
        ev.write({'x': 4})


# ---------------------------------------------------------------- pre-send checkpoint
def test_pre_send_record_is_saved_before_the_send(sources, tmp_path):
    store = []

    class Checked(mock.FakeTransport):
        def send_goal(self, goal, binding):
            assert store[0].docs[-1]['evidence']['phase'] == 'pre_send'
            assert store[0].docs[-1]['dispatch']['attempted'] is False
            return super().send_goal(goal, binding)
    r = Run(sources, tmp_path, Checked, evidence=fake_evidence(store))
    assert r() == lpb.EXIT_OK
    assert [d['evidence']['phase'] for d in store[0].docs] == ['reserved', 'pre_send', 'final']


def test_pre_send_record_failure_refuses_without_sending(sources, tmp_path):
    store = []
    r = Run(sources, tmp_path, evidence=fake_evidence(store, fail_on={2}))
    assert r() == lpb.EXIT_REFUSED
    final = store[0].docs[-1]
    assert final['reason'] == 'evidence_persistence_failed' and final['goals_sent'] == 0
    assert r.made[0].sent == [] and r.made[0].cancels == 0
    assert final['dispatch']['attempted'] is False
    assert final['errors'][0]['where'] == 'evidence_checkpoint'


# ---------------------------------------------------------------- dispatch / acceptance boundary
def test_send_failure_before_the_server_keeps_acceptance_unknown(sources, tmp_path):
    class Broken(mock.FakeTransport):
        def send_goal(self, goal, binding):
            raise RuntimeError('wire down')
    r = Run(sources, tmp_path, Broken)
    assert r() == lpb.EXIT_FAILED
    d = r.report()
    assert d['state'] == lpb.HELD_ERROR and d['goals_sent'] == 0 and d['cancels_sent'] == 0
    assert d['dispatch']['attempted'] and d['dispatch']['acceptance'] == 'unknown'
    assert d['dispatch']['final_goal_status'] == 'unknown'
    assert d['dispatch']['goal_may_still_be_executing'] is True


def test_send_failure_after_the_server_got_the_goal_is_never_retried(sources, tmp_path):
    class LateBreak(mock.FakeTransport):
        def send_goal(self, goal, binding):
            super().send_goal(goal, binding)             # the goal reached the "server"...
            raise RuntimeError('connection lost before the call returned')
    r = Run(sources, tmp_path, LateBreak)
    assert r() == lpb.EXIT_FAILED
    d = r.report()
    assert len(r.made[0].sent) == 1 and r.made[0].cancels == 0     # no retry, no second goal
    assert d['dispatch']['acceptance'] == 'unknown'
    assert d['dispatch']['goal_may_still_be_executing'] is True     # truthfully uncertain
    assert 'reached the server' in d['events'][-1][2]


def test_sim_time_failure_at_acceptance_cancels_once(sources, tmp_path):
    class NoSimTime(mock.FakeTransport):
        def sim_now(self):
            raise RuntimeError('clock unavailable')
    r = Run(sources, tmp_path, NoSimTime)
    assert r() == lpb.EXIT_FAILED
    d = r.report()
    t = r.made[0]
    assert len(t.sent) == 1 and t.cancels == 1 and d['cancels_sent'] == 1
    assert d['dispatch']['acceptance'] == 'accepted'
    assert d['dispatch']['goal_id'] == 'mock-goal-0001'
    assert d['cancel_reason'] == 'transport_error' and d['state'] == lpb.HELD_ERROR
    assert d['dispatch']['final_result_known'] and d['result']['status'] == 5   # canceled
    assert [e['where'] for e in d['errors']] == ['sim_now']


def test_poll_failure_while_active_cancels_once_and_keeps_status_unknown(sources, tmp_path):
    r = Run(sources, tmp_path, poll_raises_at=5.0)
    assert r() == lpb.EXIT_FAILED
    d = r.report()
    assert r.made[0].cancels == 1 and len(r.made[0].sent) == 1
    assert d['state'] == lpb.HELD_ERROR and d['dispatch']['cancel_attempted']
    assert d['dispatch']['cancel_response_known'] is False
    assert d['dispatch']['final_goal_status'] == 'unknown'
    assert d['dispatch']['goal_may_still_be_executing'] is True
    assert [e['where'] for e in d['errors']] == ['poll', 'poll']


def test_poll_failure_while_pending_never_cancels_without_a_handle(sources, tmp_path):
    r = Run(sources, tmp_path, poll_raises_at=0.0, response_latency=1.0)
    assert r() == lpb.EXIT_FAILED
    d = r.report()
    assert r.made[0].cancels == 0 and len(r.made[0].sent) == 1
    assert d['dispatch']['acceptance'] == 'unknown'
    assert d['dispatch']['goal_may_still_be_executing'] is True


def test_cancel_failure_is_recorded_and_never_retried(sources, tmp_path):
    class CancelBreaks(mock.FakeTransport):
        def cancel_goal(self):
            self.cancels += 1
            raise RuntimeError('cancel service unavailable')
    latch = lpb.InterruptLatch()
    r = Run(sources, tmp_path, CancelBreaks, latch=latch, interrupts_at=(5.0,))
    assert r() == lpb.EXIT_FAILED
    d = r.report()
    t = r.made[0]
    assert t.cancels == 1 and len(t.sent) == 1 and d['cancels_sent'] == 1
    assert d['dispatch']['cancel_attempted'] and 'cancel service unavailable' in \
        d['dispatch']['cancel_error']
    assert d['dispatch']['cancel_response_known'] is False
    # the goal went on to finish: its final status is known and was not CANCELED
    assert d['state'] == lpb.CANCEL_UNCONFIRMED and d['dispatch']['final_result_known']
    assert d['result']['status'] == 4


def test_cancel_and_poll_failure_ends_with_status_unknown(sources, tmp_path):
    class AllBreaks(mock.FakeTransport):
        def cancel_goal(self):
            self.cancels += 1
            self.poll_raises_at = self._rel()            # the transport dies with the cancel
            raise RuntimeError('cancel service unavailable')
    latch = lpb.InterruptLatch()
    r = Run(sources, tmp_path, AllBreaks, latch=latch, interrupts_at=(5.0,))
    assert r() == lpb.EXIT_FAILED
    d = r.report()
    assert r.made[0].cancels == 1 and len(r.made[0].sent) == 1
    assert d['state'] == lpb.HELD_ERROR and d['dispatch']['final_goal_status'] == 'unknown'
    assert d['dispatch']['goal_may_still_be_executing'] is True


def test_graph_check_failure_cancels_once(sources, tmp_path):
    class GraphBreaks(mock.FakeTransport):
        def graph_status(self):
            if self._rel() >= 4.0:
                raise RuntimeError('graph query failed')
            return super().graph_status()
    r = Run(sources, tmp_path, GraphBreaks)
    assert r() == lpb.EXIT_FAILED
    d = r.report()
    assert r.made[0].cancels == 1 and len(r.made[0].sent) == 1
    assert d['cancel_reason'] == 'transport_error' and d['state'] == lpb.HELD_ERROR
    assert [e['where'] for e in d['errors']] == ['graph_status']
    assert d['dispatch']['final_result_known'] and d['result']['status'] == 5


@pytest.mark.parametrize('where', ['before_result', 'after_result'])
def test_exception_escaping_supervision_is_truthful(sources, monkeypatch, where):
    """A failure outside the guarded calls still ends truthfully: at most the one cancel, and
    only while the goal is active with its result unknown."""
    from spiderx_controller import m6_goal_fingerprint as gf
    from spiderx_controller import m6_live_readiness as rd
    traj = m6t.build_trajectory(sources)
    fp = gf.fingerprint(gf.approved_spec(traj, sources))
    t = mock.FakeTransport(traj, fp)
    report = mock.mock_readiness_report(sources.joint_names, list(NEUTRAL))
    session_cls = lpb.LiveSession
    if where == 'before_result':
        real = lpb.StreamMonitor.check

        def check(self, now):
            if t._rel() > 4.0:
                raise RuntimeError('monitor crashed')
            return real(self, now)
        monkeypatch.setattr(lpb.StreamMonitor, 'check', check)
    else:
        class Breaks(lpb.LiveSession):
            def _finish_result(self):
                raise RuntimeError('evaluation crashed')
        session_cls = Breaks
    out = session_cls(t, sources, lambda: rd.assess(report, t.wall_now()),
                      lambda: lc.CONFIRMATION_WORD + '\n').run()
    assert out['state'] == lpb.HELD_ERROR and len(t.sent) == 1
    assert out['errors'][-1]['where'] == 'session'
    if where == 'before_result':
        assert t.cancels == 1 and out['dispatch']['goal_may_still_be_executing'] is True
    else:
        assert t.cancels == 0 and out['dispatch']['final_result_known']
        assert out['dispatch']['goal_may_still_be_executing'] is False


# ---------------------------------------------------------------- final persistence
def test_final_write_failure_emits_stderr_and_never_claims_saved(sources, tmp_path, capsys):
    store = []
    r = Run(sources, tmp_path, evidence=fake_evidence(store, fail_on={3}))
    assert r() == lpb.EXIT_FAILED
    out = capsys.readouterr()
    assert 'Report:' not in out.out and 'EVIDENCE NOT SAVED' in out.err
    fallback = json.loads(out.err[out.err.index('{'):])
    assert fallback['goals_sent'] == 1 and fallback['state'] == lpb.SUCCEEDED
    assert fallback['evidence'] == {'path': store[0].path, 'phase': 'final', 'saved': False,
                                    'persistence_error': 'OSError: injected write failure'}
    assert len(r.made[0].sent) == 1 and r.made[0].cancels == 0      # never retried


def test_final_serialization_failure_keeps_pre_send_record(sources, tmp_path, capsys):
    """Malformed (NaN) feedback makes the outcome unserializable as strict JSON."""
    r = Run(sources, tmp_path, feedback_offset=float('nan'))
    assert r() == lpb.EXIT_FAILED
    out = capsys.readouterr()
    assert 'EVIDENCE NOT SAVED' in out.err and 'NaN' in out.err and 'Report:' not in out.out
    on_disk = r.report()                                 # the last saved record
    assert on_disk['evidence']['phase'] == 'pre_send'   # => a reader must treat status as unknown
    assert len(r.made[0].sent) == 1


def test_setup_failure_is_refused_with_nothing_sent(sources, tmp_path):
    r = Run(sources, tmp_path)

    def broken(fp, domain_id):
        raise la.TransportError('rclpy init failed')
    r.transport = broken
    assert r() == lpb.EXIT_REFUSED
    d = r.report()
    assert d['reason'] == 'session_setup_failed' and d['dispatch']['attempted'] is False
    assert d['errors'][0]['message'] == 'rclpy init failed'


def test_close_failure_is_recorded_and_does_not_hide_the_outcome(sources, tmp_path):
    class CloseBreaks(mock.FakeTransport):
        def close(self):
            raise RuntimeError('destroy failed')
    r = Run(sources, tmp_path, CloseBreaks)
    assert r() == lpb.EXIT_OK
    d = r.report()
    assert d['state'] == lpb.SUCCEEDED
    assert d['transport'] == {'closed': False, 'close_error': 'RuntimeError: destroy failed',
                              'note': lpb.TRANSPORT_CLOSE_NOTE}


def test_unknown_status_warns_that_closing_does_not_stop_the_goal(sources, tmp_path, capsys):
    r = Run(sources, tmp_path, respond=False)          # no goal response at all
    assert r() == lpb.EXIT_FAILED
    d = r.report()
    assert d['state'] == lpb.TIMED_OUT and d['dispatch']['acceptance'] == 'unknown'
    assert d['dispatch']['goal_may_still_be_executing'] is True and r.made[0].cancels == 0
    out = capsys.readouterr().out
    assert 'final goal status is UNKNOWN' in out and 'does not cancel or stop' in out
