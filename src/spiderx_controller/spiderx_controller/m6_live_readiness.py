"""M6.0-D same-process readiness: installed-stack classification and readiness freshness (D3, D4).

Classification (owner decision, M6 plan section 14.9; D4):
  incompatible - a required action/message field, the controller endpoint or type, or the
                 12-joint contract differs or is missing (or was not observed at all);
  warning      - only package versions differ from the cloud reference; contract checks pass;
  compatible   - contract checks pass and every reference version matches.
Only `incompatible` blocks on classification grounds; any other NOT-READY code (controllers not
active, stale or incomplete /joint_states, start pose, probe error) blocks as well.

A readiness result carries the monotonic time it was observed at and expires after 10 s (D3).
Pure Python: the input is the report dict of m6_live_preflight.evaluate(); nothing here touches
a ROS graph.
"""

from dataclasses import dataclass, field

from spiderx_controller import m6_live_contract as lc

COMPATIBLE, WARNING, INCOMPATIBLE = 'compatible', 'warning', 'incompatible'
CLASSES = (COMPATIBLE, WARNING, INCOMPATIBLE)

# failure codes of m6_live_preflight that mean the contract itself is broken
INCOMPATIBLE_CODES = frozenset({
    'interface_contract_mismatch',
    'action_server_missing',
    'action_server_ambiguous',
    'action_type_mismatch',
    'controller_type_mismatch',
    'joint_names_mismatch',
    'joint_states_type_mismatch',
})


def classify(report):
    """(label, reasons) for an m6_live_preflight report dict. Never raises."""
    if not isinstance(report, dict):
        return INCOMPATIBLE, ['no readiness report']
    reasons = []
    checks = report.get('checks') or {}
    not_observed = (None, 'not_run')
    if report.get('verdict') == 'INTERFACE ONLY' or checks.get('action_server') in not_observed:
        reasons.append('endpoint and joint contract not observed (graph mode required)')
    if checks.get('joint_names_contract') in (None, 'not_run'):
        reasons.append('joint contract not observed')
    codes = set(report.get('failure_codes') or [])
    bad = sorted(codes & INCOMPATIBLE_CODES)
    reasons += [f'contract failure: {c}' for c in bad]
    if reasons:
        return INCOMPATIBLE, reasons
    rows = ((report.get('version_check') or {}).get('packages') or {})
    differs = sorted(f'{p}: installed {r.get("installed")}, reference {r.get("reference")} '
                     f'({r.get("status")})'
                     for p, r in rows.items() if r.get('status') != 'matches')
    if not rows:
        return WARNING, ['no package versions recorded']
    if differs:
        return WARNING, differs
    return COMPATIBLE, []


@dataclass(frozen=True)
class ReadinessResult:
    ready: bool                      # the live preflight verdict was READY
    label: str                       # compatible | warning | incompatible
    reasons: tuple
    failure_codes: tuple
    observed_monotonic: float        # time.monotonic() when the observation finished
    observed_wall_iso: object = None
    report: dict = field(default_factory=dict, compare=False)

    def age_s(self, now_monotonic):
        return now_monotonic - self.observed_monotonic

    def fresh(self, now_monotonic, max_age_s=lc.READINESS_MAX_AGE_S):
        age = self.age_s(now_monotonic)
        return 0.0 <= age <= max_age_s

    def to_dict(self, now_monotonic=None):
        d = {'ready': self.ready, 'classification': self.label, 'reasons': list(self.reasons),
             'failure_codes': list(self.failure_codes),
             'observed_wall_iso': self.observed_wall_iso}
        if now_monotonic is not None:
            d['age_s'] = round(self.age_s(now_monotonic), 6)
        return d


def assess(report, observed_monotonic, observed_wall_iso=None):
    label, reasons = classify(report)
    rep = report if isinstance(report, dict) else {}
    return ReadinessResult(ready=bool(rep.get('ready')), label=label, reasons=tuple(reasons),
                           failure_codes=tuple(rep.get('failure_codes') or ()),
                           observed_monotonic=float(observed_monotonic),
                           observed_wall_iso=observed_wall_iso, report=rep)


def dispatch_permitted(result, now_monotonic, max_age_s=lc.READINESS_MAX_AGE_S):
    """(permitted, refusal_code). Every condition must hold; the first failing one is named."""
    if not isinstance(result, ReadinessResult):
        return False, 'readiness_missing'
    if not result.fresh(now_monotonic, max_age_s):
        return False, 'readiness_stale'
    if result.label == INCOMPATIBLE:
        return False, 'stack_incompatible'
    if result.label not in (COMPATIBLE, WARNING):
        return False, 'classification_unknown'
    if not result.ready:
        return False, 'readiness_not_ready'
    return True, None


def make_readiness_provider(collect_graph, names, neutral, monotonic, versions=None,
                            interface=None, wall_iso=None):
    """A same-process readiness provider: () -> ReadinessResult (D3).

    collect_graph() returns read-only graph observations (m6_live_preflight.GraphProbe.collect on
    the dispatching process's own node). versions/interface default to the installed stack. The
    observation time is taken AFTER collection, so the 10 s expiry covers the whole observation.
    """
    from spiderx_controller import m6_live_preflight as lpf

    def provide():
        obs = lpf.Observations()
        try:
            obs.versions = versions if versions is not None else lpf.collect_versions()
            obs.interface = interface if interface is not None else lpf.collect_interface()
        except Exception as e:  # noqa: BLE001 - recorded; evaluate() then fails the contract
            obs.probe_errors.append(f'interface: {type(e).__name__}: {e}')
        try:
            graph = collect_graph()
            for attr in ('action_servers', 'controllers', 'joint_state_publishers',
                         'joint_state_messages'):
                setattr(obs, attr, getattr(graph, attr))
            obs.probe_errors += list(graph.probe_errors)
        except Exception as e:  # noqa: BLE001
            obs.probe_errors.append(f'graph: {type(e).__name__}: {e}')
        report = lpf.evaluate(obs, list(names), list(neutral))
        return assess(report, monotonic(), wall_iso() if wall_iso else None)
    return provide


__all__ = ['make_readiness_provider', 'COMPATIBLE', 'WARNING', 'INCOMPATIBLE', 'CLASSES',
           'INCOMPATIBLE_CODES', 'classify', 'ReadinessResult', 'assess', 'dispatch_permitted']
