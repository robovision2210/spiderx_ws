"""M6.1 in-flight tracking reference: WHEN the approved trajectory started, as the client sees it.

Why. The second authorized Cloud cycle (2026-10-12, docs/M61A_CLOUD_VERIFICATION.md section 13)
was cancelled by the client's in-flight tracking check while the joints followed the approved
trajectory within 0.0108 rad. The unchanged M6.0-D rule (m6_live_playback.LiveSession._ref_time)
takes the median of `feedback stamp - desired.time_from_start` as the trajectory start whenever it
differs from acceptance by more than 0.1 s. joint_trajectory_controller 2.48.0 (this Cloud image)
leaves desired.time_from_start at 0 in its action feedback; 2.49.0 added it ("[JTC] added
time_from_start to action feedback"). With 0, the "estimate" is just the median feedback stamp,
which drifts to about half the elapsed time and holds the reference late. The M6.0-D crouch ran
on 2.54.0 (owner PC), where the field is filled.

Rule (M6.1 only; the M6.0-D module is unchanged):
  - the anchor is the goal ACCEPTANCE (sim time when the accepted response was processed). The
    bag of that cycle shows the controller's own setpoint started 0.015 s before it;
  - only feedback whose desired.time_from_start is INFORMATIVE (finite, > TFS_MIN_S) may form an
    estimate. If it differs from acceptance by more than the M6.0-D limit (0.1 s), the estimate
    becomes the anchor: the controller itself reports that it started elsewhere;
  - feedback with time_from_start at or below TFS_MIN_S (0 on 2.48.0), negative or not finite
    never forms an estimate. When feedback arrived but none of it was informative, the acceptance
    anchor is a FALLBACK and is recorded as such (record['fallback_time_from_start_zero']).
Nothing here changes a threshold: the abort limit stays m61_limits.client_tracking_abort_rad
(0.05 rad) and the offset limit stays m6_live_contract.FEEDBACK_OFFSET_LIMIT_S (0.1 s).
Pure Python.
"""

import math

from spiderx_controller import m6_live_contract as lc

TFS_MIN_S = 1e-6                     # a time_from_start at or below this carries no timing
ACCEPTANCE, FEEDBACK_ESTIMATE = 'acceptance', 'feedback_estimate'
FALLBACK_NOTE = ('feedback desired.time_from_start is 0 (not informative): the reference is '
                 'anchored at goal acceptance, not at a feedback estimate')


def informative(tfs):
    return isinstance(tfs, (int, float)) and math.isfinite(tfs) and tfs > TFS_MIN_S


def tracking_anchor(accept_sim, feedback, offset_limit_s=lc.FEEDBACK_OFFSET_LIMIT_S):
    """(t0, record) for the reference start. feedback: [(stamp, time_from_start)] in arrival
    order. t0 is None before acceptance. Never raises."""
    fb = list(feedback or ())
    good = [st - tfs for st, tfs in fb
            if informative(tfs) and isinstance(st, (int, float)) and math.isfinite(st)]
    est = sorted(good)[len(good) // 2] if good else None
    rec = {'accept_sim_s': accept_sim, 'feedback_samples': len(fb),
           'informative_samples': len(good), 'non_informative_samples': len(fb) - len(good),
           'feedback_estimate_s': est,
           'estimate_offset_s': (None if est is None or accept_sim is None
                                 else round(est - accept_sim, 6)),
           'offset_limit_s': offset_limit_s, 'anchor': None, 't0_sim_s': None,
           'fallback_time_from_start_zero': bool(fb) and not good,
           'note': None}
    if accept_sim is None:
        return None, rec
    t0, anchor = accept_sim, ACCEPTANCE
    if est is not None and abs(est - accept_sim) > offset_limit_s:
        t0, anchor = est, FEEDBACK_ESTIMATE
    rec.update(anchor=anchor, t0_sim_s=t0,
               note=FALLBACK_NOTE if rec['fallback_time_from_start_zero'] else None)
    return t0, rec


__all__ = ['tracking_anchor', 'informative', 'TFS_MIN_S', 'ACCEPTANCE', 'FEEDBACK_ESTIMATE',
           'FALLBACK_NOTE']
