"""Read-only observer plugin for test_m6d_isolated_success (diagnosis only; not package source).

Load with `pytest -p diag_plugin` (this directory on PYTHONPATH). It changes no decision logic: it
wraps functions to RECORD their inputs/outputs and writes one JSON file to $DIAG_OUT:
  - every readiness evaluation: the full M6.0-B report (checks, failures with messages, observed),
    the stamps of the /joint_states samples the probe received, their order violations and the
    sample age against the fake stack's simulation clock, monotonic/wall timestamps, collection
    duration, and the node names visible to the probe (ROS-domain peers);
  - the isolation graph snapshots the test itself takes before and after;
  - the fake stack's tick overlaps (a tick starting while another is still running) and the order
    in which it handed /joint_states stamps to publish().
DIAG_STACK=original loads the pre-diagnosis fake stack from ./original_stack instead of the
repository's (A/B comparison). The committed gate is never changed; the test itself patches it
locally exactly as in the repository.
"""
import datetime
import importlib.util
import itertools
import json
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.environ['DIAG_OUT']
REC = {'pid': os.getpid(), 'stack_variant': os.environ.get('DIAG_STACK', 'repo'),
       'loadavg_start': os.getloadavg(), 'wall_start': datetime.datetime.utcnow().isoformat() + 'Z',
       'readiness': [], 'graph_snapshots': [], 'stack': {}}
_STACK = {'obj': None}


def _load_original_stack():
    path = os.path.join(HERE, 'original_stack', 'm6d_isolated_stack.py')
    spec = importlib.util.spec_from_file_location('m6d_isolated_stack', path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules['m6d_isolated_stack'] = mod
    spec.loader.exec_module(mod)
    return mod


def _instrument_stack(mod):
    cls = mod.IsolatedFakeStack
    orig_init, orig_tick, orig_stop = cls.__init__, cls._tick, cls.stop

    def __init__(self, *a, **k):
        self._diag_active = 0
        self._diag_lock = threading.Lock()
        self._diag_overlaps = 0
        self._diag_max_active = 0
        self._diag_ticks = 0
        self._diag_pub = []
        self._diag_seq = itertools.count()
        orig_init(self, *a, **k)
        real_pub = self.js_pub
        stack = self

        class _LoggingPublisher:
            def publish(self, msg):
                stack._diag_pub.append((next(stack._diag_seq),
                                        msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                                        threading.get_ident(), time.monotonic()))
                real_pub.publish(msg)

            def __getattr__(self, name):
                return getattr(real_pub, name)
        self.js_pub = _LoggingPublisher()
        _STACK['obj'] = self

    def _tick(self):
        with self._diag_lock:
            self._diag_active += 1
            self._diag_ticks += 1
            if self._diag_active > 1:
                self._diag_overlaps += 1
            self._diag_max_active = max(self._diag_max_active, self._diag_active)
        try:
            return orig_tick(self)
        finally:
            with self._diag_lock:
                self._diag_active -= 1

    def stop(self):
        pubs = list(self._diag_pub)
        stamps = [p[1] for p in pubs]
        bad = [i for i in range(1, len(stamps)) if stamps[i] <= stamps[i - 1]]
        REC['stack'] = {
            'class_module_file': mod.__file__,
            'ticks': self._diag_ticks, 'tick_overlaps': self._diag_overlaps,
            'max_concurrent_ticks': self._diag_max_active,
            'published': len(pubs), 'publish_threads': len({p[2] for p in pubs}),
            'publish_order_violations': len(bad),
            'violation_examples': [{'index': i, 'prev_stamp': stamps[i - 1],
                                    'stamp': stamps[i], 'delta_s': stamps[i] - stamps[i - 1]}
                                   for i in bad[:10]],
        }
        return orig_stop(self)

    cls.__init__, cls._tick, cls.stop = __init__, _tick, stop


def _instrument_readiness():
    from spiderx_controller import m6_live_preflight as lpf
    from spiderx_controller import m6_live_readiness as rd
    orig_eval, orig_assess, orig_collect = lpf.evaluate, rd.assess, lpf.GraphProbe.collect
    pending = {}

    def collect(self):
        t0 = time.monotonic()
        obs = orig_collect(self)
        pending['peers'] = sorted(f'{ns.rstrip("/")}/{n}' for n, ns in
                                  self.node.get_node_names_and_namespaces())
        pending['probe_collect_s'] = time.monotonic() - t0
        return obs

    def evaluate(obs, names, neutral):
        report = orig_eval(obs, names, neutral)
        msgs = obs.joint_state_messages or []
        stamps = [m[0] for m in msgs]
        bad = [i for i in range(1, len(stamps)) if stamps[i] <= stamps[i - 1]]
        stack = _STACK['obj']
        sim_now = stack.sim_now() if stack is not None else None
        pending['samples'] = {
            'count': len(stamps), 'first_stamp': stamps[0] if stamps else None,
            'last_stamp': stamps[-1] if stamps else None,
            'order_violations': len(bad),
            'violation_examples': [{'index': i, 'prev_stamp': stamps[i - 1], 'stamp': stamps[i],
                                    'delta_s': stamps[i] - stamps[i - 1]} for i in bad[:10]],
            'sim_now_at_evaluation': sim_now,
            'last_sample_age_sim_s': (sim_now - stamps[-1]) if (stamps and sim_now) else None,
            'last_sample_age_wall_s': ((sim_now - stamps[-1]) / stack.rtf
                                       if (stamps and sim_now) else None),
            'span_sim_s': (stamps[-1] - stamps[0]) if len(stamps) > 1 else None,
        }
        pending['report'] = report
        return report

    def assess(report, observed_monotonic, observed_wall_iso=None, collection_s=0.0):
        result = orig_assess(report, observed_monotonic, observed_wall_iso, collection_s)
        REC['readiness'].append({
            'index': len(REC['readiness']) + 1,
            'observed_monotonic': observed_monotonic, 'observed_wall_iso': observed_wall_iso,
            'assessed_monotonic': time.monotonic(), 'collection_s': collection_s,
            'loadavg': os.getloadavg(),
            'ready': result.ready, 'classification': result.label,
            'failure_codes': list(result.failure_codes),
            'failures': report.get('failures'), 'checks': report.get('checks'),
            'observed': {k: v for k, v in (report.get('observed') or {}).items()
                         if k != 'last_joint_state'},
            'samples': pending.pop('samples', None),
            'peers_visible_to_probe': pending.pop('peers', None),
            'probe_collect_s': pending.pop('probe_collect_s', None),
        })
        pending.pop('report', None)
        _dump()
        return result

    lpf.evaluate, rd.assess, lpf.GraphProbe.collect = evaluate, assess, collect


def _dump():
    REC['loadavg_last'] = os.getloadavg()
    with open(os.path.join(OUT, 'diag.json'), 'w') as f:
        json.dump(REC, f, indent=1, default=repr)


def pytest_configure(config):
    os.makedirs(OUT, exist_ok=True)
    if REC['stack_variant'] == 'original':
        mod = _load_original_stack()
    else:
        sys.path.insert(0, os.environ['DIAG_TEST_DIR'])
        import m6d_isolated_stack as mod      # the repository's
    _instrument_stack(mod)
    _instrument_readiness()


def pytest_collection_modifyitems(session, config, items):
    for item in items:
        m = item.module
        if hasattr(m, 'graph_snapshot') and not hasattr(m.graph_snapshot, '_diag'):
            orig = m.graph_snapshot

            def snap(domain, settle_s=1.5, _orig=orig):
                res = _orig(domain, settle_s)
                REC['graph_snapshots'].append({'domain': domain, 't': time.monotonic(),
                                               'result': res})
                return res
            snap._diag = True
            m.graph_snapshot = snap
            REC['domain'] = getattr(m, 'DOMAIN', None)


def pytest_runtest_logreport(report):
    if report.when == 'call':
        REC.setdefault('outcomes', []).append(
            {'nodeid': report.nodeid, 'outcome': report.outcome,
             'duration_s': report.duration,
             'longrepr_tail': str(report.longrepr)[-1500:] if report.failed else None})


def pytest_unconfigure(config):
    REC['wall_end'] = datetime.datetime.utcnow().isoformat() + 'Z'
    _dump()
