"""M6.1-A link-pose check: where the Gazebo pose entries come from (E8) and whether the leg-link
entries follow the joints (E9). SIMULATION ONLY. Read-only analysis of recorded data.

Pure Python core (no ROS graph, no numpy). `read_bag` imports rosbag2_py and rclpy lazily; only
the CLI uses it. Nothing here publishes, commands or calls a service.

What an entry of /spiderx/sim/world_poses is
--------------------------------------------
Read for this module: gz-sim 6.16.0 Physics.cc and gz-physics 5.3.2 dartsim
SimulationFeatures.cc, the installed versions.
  * After each step the dartsim plugin reports every link whose DART world pose differs from its
    LAST REPORTED pose by more than 1e-6 in any position axis or any quaternion component (all
    links on the first step). An unreported change therefore never accumulates beyond that.
  * For a model's canonical link, UpdateModelPose writes the MODEL pose X_WM = X_WL * inv(X_ML)
    from DART's X_WL and caches it (modelWorldPoses). Nothing else fills that cache. The canonical
    link's own pose X_ML is never written: it stays the SDF value.
  * Every other link is written relative to the cached model pose,
    X_M_link = inv(modelWorldPoses[model]) * X_W_link(DART). Without a cached model pose it is
    skipped with "Internal error: parent model [...] does not have a world pose available".
  * SceneBroadcaster publishes these pose components as pose/info; the bridge maps each one to a
    TransformStamped (child_frame_id = name, stamp 0).

What follows, and what this module computes
-------------------------------------------
E8, body pose. model entry * dummy_link entry * T_dummy_body = X_WL(DART) * T_dummy_body. The X_ML
that Physics divided by is the X_ML multiplied back, so the composition is DART's pose of the
merged body (dummy_link + base_link + lidar_link) and does not depend on the weld or on X_ML.
T_dummy_body is the URDF dummy_joint origin inside that one rigid body. `body_report` compares it
with the weld origin of /robot_description (the specification).

E8, provenance in this launch. A leg-link entry relative to the dummy_link entry is
inv(X_WM * X_ML) * X_W_link. Never written, it is the SDF value FK(0). Written by Physics, it is
DART's relative pose, FK(q) at the joint positions. If every leg-link entry matches FK(q) at the
measured /joint_states, and q is far enough from 0 that FK(q) and FK(0) differ measurably, then
Physics wrote those entries in this launch. By the source above that requires UpdateModelPose to
have written the model entry from DART first. `provenance_report` decides this; the bound is the
reporting threshold, so it works without motion.

E9, motion. Leg-link entries that change and track FK(q(t)) show dynamic link/kinematic
consistency of the pose stream (content liveness). They are relative to the body: they say
nothing independent about the body's global height or tilt. `e9_report`.

Every threshold is a module constant, frozen before any run that uses it, and copied into the
report together with this file's SHA-256.
"""

import argparse
import bisect
from dataclasses import dataclass
import hashlib
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

from spiderx_controller import leg_kinematics as lk
from spiderx_controller import m61a_fixed_base as fb

SCHEMA = 'spiderx.m61a.link_check/1'
MODEL_NAME = 'spiderx'
ROOT_LINK = 'dummy_link'
TOPICS = {'poses': '/spiderx/sim/world_poses', 'joints': '/joint_states', 'clock': '/clock',
          'description': '/robot_description'}

# gz-physics 5.3.2 dartsim SimulationFeatures::Write: re-report threshold per position axis (m)
# and per quaternion component.
REPORT_TOL = 1e-6

# ---- E8 physics provenance (no motion needed). Frozen 2026-10-10, before its observation run.
# Worst unreported change of a link: sqrt(3) * 1e-6 = 1.7e-6 m; about 4e-6 rad. Match tolerance
# = 1e-5 m and 1e-5 rad (>= 2.5x). A sample discriminates FK(q) from FK(0) only if some link
# differs by >= 5e-5 rad between the two (5x the match tolerance).
PROV_MATCH_POSITION_M = 1e-5
PROV_MATCH_ROTATION_RAD = 1e-5
PROV_MIN_SEPARATION_RAD = 5e-5
PROV_STEADY_WINDOW_S = 0.25      # sim s on each side; joints must be steady there
PROV_STEADY_Q_RAD = 1e-6         # max change of any joint within that window
PROV_MIN_SAMPLES = 20

# ---- E9 dynamic link/kinematic consistency (needs motion). Frozen 2026-10-10, before any
# motion run. A pose sample is attributed the latest /clock received before it; it is explained
# if FK(q(t')) for some t' within +-E9_ALIGN_WINDOW_S matches every link within the tolerances.
# The approved cycle moves thigh and foot joints by up to 0.11 rad, at waypoint speeds up to
# 0.12 rad/s (m61_trot_cycle.yaml): a stale stream misses by far more than 5 mrad, a live one by
# about 1e-5 rad.
E9_POSITION_TOL_M = 1e-3
E9_ROTATION_TOL_RAD = 5e-3
E9_ALIGN_WINDOW_S = 0.025
E9_ALIGN_STEP_S = 0.001
E9_MIN_SAMPLES = 100
E9_MAX_GAP_S = 1.0               # sim s between consecutive analysed samples (pose freshness)
E9_MIN_FOOT_SPAN_RAD = 0.02      # each foot link must turn this much relative to the body (4x)
FOOT_LINKS = ('lf_foot_1', 'rf_foot_1', 'lr_foot_1', 'rr_foot_1')

# ---- server log lines that would contradict the provenance argument
LOG_PATTERNS = ('Internal error', 'does not have a world pose')

PASS, FAIL, INCONCLUSIVE, NO_MOTION = 'PASS', 'FAIL', 'INCONCLUSIVE', 'NO_MOTION'


class LinkCheckError(ValueError):
    pass


def frozen_parameters():
    return {
        'report_tol': REPORT_TOL,
        'provenance': {'match_position_m': PROV_MATCH_POSITION_M,
                       'match_rotation_rad': PROV_MATCH_ROTATION_RAD,
                       'min_separation_rad': PROV_MIN_SEPARATION_RAD,
                       'steady_window_s': PROV_STEADY_WINDOW_S,
                       'steady_q_rad': PROV_STEADY_Q_RAD, 'min_samples': PROV_MIN_SAMPLES},
        'e9': {'position_tol_m': E9_POSITION_TOL_M, 'rotation_tol_rad': E9_ROTATION_TOL_RAD,
               'align_window_s': E9_ALIGN_WINDOW_S, 'align_step_s': E9_ALIGN_STEP_S,
               'min_samples': E9_MIN_SAMPLES, 'max_gap_s': E9_MAX_GAP_S,
               'min_foot_span_rad': E9_MIN_FOOT_SPAN_RAD, 'foot_links': list(FOOT_LINKS)},
        'log_patterns': list(LOG_PATTERNS),
    }


def source_sha256():
    with open(os.path.abspath(__file__), 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


# ---------------------------------------------------------------- kinematics
def angle_between(qa, qb):
    """Rotation angle (rad) from unit quaternion qa to qb; accurate for small angles."""
    x, y, z, w = fb.quat_mul((-qa[0], -qa[1], -qa[2], qa[3]), qb)
    return 2.0 * math.atan2(math.sqrt(x * x + y * y + z * z), abs(w))


def residual(a, b):
    """(translation m, rotation rad) between two fb.Pose."""
    return math.dist(a.p, b.p), angle_between(a.q, b.q)


def _axis_angle(axis, angle):
    n = math.sqrt(sum(c * c for c in axis))
    if not n > 0:
        raise LinkCheckError(f'zero joint axis {axis}')
    s = math.sin(angle / 2.0)
    return (axis[0] / n * s, axis[1] / n * s, axis[2] / n * s, math.cos(angle / 2.0))


@dataclass(frozen=True)
class LinkModel:
    """The links Gazebo reports below `root` and how to reach them from it."""
    root: str
    links: tuple                 # children of non-fixed joints (fixed children are lumped)
    joints: tuple                # actuated joints below root, sorted
    order: tuple                 # (joint dict, fixed origin as fb.Pose), parents before children


def link_model(urdf, root=ROOT_LINK):
    """LinkModel from a URDF string or root element. Fixed-joint children are lumped into their
    parent body (sdformat fixed-joint reduction), so only children of revolute or continuous
    joints are separate Gazebo links; their frames are the URDF link frames."""
    el = ET.fromstring(urdf) if isinstance(urdf, (str, bytes)) else urdf
    joints = lk.parse_urdf_joints(el)
    children = {}
    for j in joints.values():
        children.setdefault(j['parent'], []).append(j)
    links, actuated, order, queue = [], [], [], [root]
    while queue:
        link = queue.pop(0)
        for j in sorted(children.get(link, ()), key=lambda j: j['name']):
            if j['type'] in ('revolute', 'continuous'):
                links.append(j['child'])
                actuated.append(j['name'])
            elif j['type'] != 'fixed':
                raise LinkCheckError(f'unsupported joint type {j["type"]} ({j["name"]})')
            order.append((j, fb.Pose.from_xyz_rpy(*j['xyz'], *j['rpy'])))
            queue.append(j['child'])
    if not links:
        raise LinkCheckError(f'no moving links below {root}')
    return LinkModel(root, tuple(sorted(links)), tuple(sorted(actuated)), tuple(order))


def forward_links(model, q):
    """{link: fb.Pose of the link frame in the root frame} at joint positions q (dict)."""
    frames = {model.root: fb.Pose()}
    for j, fixed in model.order:
        T = frames[j['parent']] * fixed
        if j['type'] != 'fixed':
            T = T * fb.Pose((0.0, 0.0, 0.0), _axis_angle(j['axis'], q[j['name']]))
        frames[j['child']] = T
    return {link: frames[link] for link in model.links}


# ---------------------------------------------------------------- samples
@dataclass(frozen=True)
class PoseSample:
    t: float                     # sim time attributed to the message (s), or None
    entries: dict                # name -> fb.Pose (links: relative to the model)
    codes: tuple = ()            # problems with this message (empty = usable)


@dataclass(frozen=True)
class JointSample:
    t: float                     # header stamp (sim s)
    q: dict


def pose_sample(transforms, t, names):
    """PoseSample from TransformStamped-like objects, keeping only `names`. A missing, duplicated,
    non-finite or non-unit entry is a code, never guessed around."""
    found, codes = {}, []
    for tr in transforms:
        n = getattr(tr, 'child_frame_id', None)
        if n not in names:
            continue
        if n in found:
            codes.append(f'ambiguous:{n}')
            continue
        pose, code = fb._entry_pose(tr)
        if code:
            codes.append(f'{code}:{n}')
            continue
        found[n] = pose
    codes.extend(f'missing:{n}' for n in sorted(set(names) - set(found)
                                                - {c.split(':', 1)[1] for c in codes}))
    if t is None:
        codes.append('no_sim_time')
    return PoseSample(t, found, tuple(sorted(set(codes))))


class JointTrack:
    """Joint positions over sim time, linearly interpolated (no extrapolation)."""

    def __init__(self, samples, joint_names):
        by_t = {}
        for s in samples:
            if all(n in s.q and fb._finite(s.q[n]) for n in joint_names) and fb._finite(s.t):
                by_t[s.t] = {n: float(s.q[n]) for n in joint_names}
        self.t = sorted(by_t)
        self.q = [by_t[t] for t in self.t]
        self.names = tuple(joint_names)

    def __len__(self):
        return len(self.t)

    def covers(self, t):
        return bool(self.t) and self.t[0] <= t <= self.t[-1]

    def at(self, t):
        if not self.covers(t):
            return None
        i = bisect.bisect_left(self.t, t)
        if self.t[i] == t:
            return dict(self.q[i])
        t0, t1 = self.t[i - 1], self.t[i]
        a = (t - t0) / (t1 - t0)
        q0, q1 = self.q[i - 1], self.q[i]
        return {n: q0[n] + a * (q1[n] - q0[n]) for n in self.names}

    def span(self, t0, t1):
        """Max over joints of (max - min) of the samples in [t0, t1] plus the interpolated ends."""
        pts = [self.at(t0), self.at(t1)]
        lo, hi = bisect.bisect_left(self.t, t0), bisect.bisect_right(self.t, t1)
        pts += self.q[lo:hi]
        pts = [p for p in pts if p is not None]
        if not pts:
            return None
        return max(max(p[n] for p in pts) - min(p[n] for p in pts) for n in self.names)


def relative_links(sample, model):
    """{link: inv(root entry) * link entry}: each link in the root-link frame."""
    root_inv = sample.entries[model.root].inverse()
    return {link: root_inv * sample.entries[link] for link in model.links}


def compare_links(rel, fk):
    """Per link (translation m, rotation rad) of the observed relative pose vs FK."""
    return {link: residual(rel[link], fk[link]) for link in rel}


def _worst(res):
    link_t = max(res, key=lambda k: res[k][0])
    link_r = max(res, key=lambda k: res[k][1])
    return {'max_translation_m': res[link_t][0], 'max_translation_link': link_t,
            'max_rotation_rad': res[link_r][1], 'max_rotation_link': link_r}


def _within(res, tol_t, tol_r):
    return all(t <= tol_t and r <= tol_r for t, r in res.values())


# ---------------------------------------------------------------- E8: body pose
def body_report(samples, weld, cfg, model_name=MODEL_NAME, root=ROOT_LINK):
    """Composed body pose (physics) against the weld origin of the description, and constancy of
    the model and dummy_link entries. Limits: one third of the configured attachment tolerance
    (the phase-1 rule)."""
    tol_t = cfg.attachment_translation_tol_m / 3.0
    tol_r = cfg.attachment_rotation_tol_rad / 3.0
    usable = [s for s in samples if model_name in s.entries and root in s.entries]
    if not usable:
        return {'verdict': INCONCLUSIVE, 'reason': 'no sample with the model and body entries',
                'samples': 0}
    expected = weld.expected_world_body()
    first_model, first_root = usable[0].entries[model_name], usable[0].entries[root]
    worst = {'translation_m': 0.0, 'rotation_rad': 0.0, 'abs_dz_m': 0.0}
    model_drift = [0.0, 0.0]
    root_drift = [0.0, 0.0]
    for s in usable:
        body = s.entries[model_name] * s.entries[root] * weld.dummy_to_body
        d_t, d_r, dz, _ = fb.deviation(body, expected)
        d_r = angle_between(body.q, expected.q)
        worst['translation_m'] = max(worst['translation_m'], d_t)
        worst['rotation_rad'] = max(worst['rotation_rad'], d_r)
        worst['abs_dz_m'] = max(worst['abs_dz_m'], abs(dz))
        for drift, a, b in ((model_drift, s.entries[model_name], first_model),
                            (root_drift, s.entries[root], first_root)):
            t, r = residual(a, b)
            drift[0], drift[1] = max(drift[0], t), max(drift[1], r)
    ok = (worst['translation_m'] <= tol_t and worst['rotation_rad'] <= tol_r
          and worst['abs_dz_m'] <= tol_t)
    last_body = usable[-1].entries[model_name] * usable[-1].entries[root] * weld.dummy_to_body
    return {'verdict': PASS if ok else FAIL, 'samples': len(usable),
            'limits': {'translation_m': tol_t, 'rotation_rad': tol_r, 'abs_dz_m': tol_t},
            'expected_world_body_xyz_rpy': list(expected.xyz_rpy()),
            'last_world_body_xyz_rpy': list(last_body.xyz_rpy()),
            'max_deviation': worst,
            'model_entry_max_change': {'translation_m': model_drift[0],
                                       'rotation_rad': model_drift[1]},
            'model_entry_first_xyz_rpy': list(first_model.xyz_rpy()),
            'dummy_entry_max_change': {'translation_m': root_drift[0],
                                       'rotation_rad': root_drift[1]},
            'dummy_entry_first_xyz_rpy': list(first_root.xyz_rpy())}


# ---------------------------------------------------------------- E8: provenance
def provenance_report(samples, track, model, log_hits=None):
    """Were the leg-link entries written by Physics in this launch (see the module docstring)?

    PASS: >= PROV_MIN_SAMPLES steady, complete samples; every one matches FK(q) on every link;
    at least one discriminates (FK(q) vs FK(0) >= PROV_MIN_SEPARATION_RAD on some link) and, on
    those, the entries do not match FK(0); no contradicting server-log line (if a log was given).
    FAIL: a mismatch with FK(q), or entries that match FK(0) where FK(q) differs, or a log line.
    INCONCLUSIVE: too few steady samples, or q too close to 0 to discriminate.
    """
    zero = forward_links(model, {n: 0.0 for n in model.joints})
    used, mismatches, discriminating, zero_like = 0, [], 0, []
    worst_q = {}
    worst_sep = 0.0
    min_zero_res = None
    skipped = {'incomplete': 0, 'outside_joint_track': 0, 'not_steady': 0}
    for s in samples:
        if s.codes:
            skipped['incomplete'] += 1
            continue
        q = track.at(s.t)
        if q is None:
            skipped['outside_joint_track'] += 1
            continue
        span = track.span(s.t - PROV_STEADY_WINDOW_S, s.t + PROV_STEADY_WINDOW_S)
        if span is None or span > PROV_STEADY_Q_RAD:
            skipped['not_steady'] += 1
            continue
        used += 1
        rel = relative_links(s, model)
        fk = forward_links(model, q)
        res_q = compare_links(rel, fk)
        res_0 = compare_links(rel, zero)
        sep = {link: angle_between(fk[link].q, zero[link].q) for link in model.links}
        worst_sep = max(worst_sep, max(sep.values()))
        for link, (a, b) in res_q.items():
            t0, r0 = worst_q.get(link, (0.0, 0.0))
            worst_q[link] = (max(t0, a), max(r0, b))
        if not _within(res_q, PROV_MATCH_POSITION_M, PROV_MATCH_ROTATION_RAD):
            mismatches.append({'t': s.t, **_worst(res_q)})
        disc = [link for link in model.links if sep[link] >= PROV_MIN_SEPARATION_RAD]
        if disc:
            discriminating += 1
            zr = min(res_0[link][1] for link in disc)
            min_zero_res = zr if min_zero_res is None else min(min_zero_res, zr)
            if any(res_0[link][1] <= PROV_MATCH_ROTATION_RAD
                   and res_0[link][0] <= PROV_MATCH_POSITION_M for link in disc):
                zero_like.append(s.t)
    hits = list(log_hits or [])
    rep = {'samples_used': used, 'skipped': skipped, 'discriminating_samples': discriminating,
           'max_fk_q_vs_fk_0_rotation_rad': worst_sep,
           'max_residual_vs_fk_q_by_link': {k: {'translation_m': v[0], 'rotation_rad': v[1]}
                                            for k, v in sorted(worst_q.items())},
           'min_residual_vs_fk_0_on_discriminating_links_rad': min_zero_res,
           'mismatch_samples': len(mismatches), 'first_mismatches': mismatches[:5],
           'samples_matching_fk_0': len(zero_like),
           'server_log_checked': log_hits is not None, 'server_log_hits': hits[:20]}
    if mismatches or zero_like or hits:
        rep['verdict'] = FAIL
        rep['reason'] = ('entries do not match FK at the measured joint positions' if mismatches
                         else 'entries equal the SDF initial values FK(0)' if zero_like
                         else 'server log contradicts the provenance argument')
    elif used < PROV_MIN_SAMPLES:
        rep['verdict'] = INCONCLUSIVE
        rep['reason'] = f'{used} steady complete samples < {PROV_MIN_SAMPLES}'
    elif not discriminating:
        rep['verdict'] = INCONCLUSIVE
        rep['reason'] = (f'q too close to 0: FK(q) and FK(0) differ by at most '
                         f'{worst_sep:.2e} rad < {PROV_MIN_SEPARATION_RAD:.0e} rad')
    else:
        rep['verdict'] = PASS
        rep['reason'] = ('every leg-link entry equals FK at the measured joints and differs from '
                         'the SDF initial value: written by Physics in this launch, after '
                         'UpdateModelPose wrote the model entry from DART')
    return rep


# ---------------------------------------------------------------- E9
def _score(res):
    return max(max(a / E9_POSITION_TOL_M, b / E9_ROTATION_TOL_RAD) for a, b in res.values())


def _window_fk(track, model, t):
    """[(shift s, FK links)] for t' = t + shift on the alignment grid inside the joint track."""
    k = int(round(E9_ALIGN_WINDOW_S / E9_ALIGN_STEP_S))
    out = []
    for i in range(-k, k + 1):
        q = track.at(t + i * E9_ALIGN_STEP_S)
        if q is not None:
            out.append((i * E9_ALIGN_STEP_S, forward_links(model, q)))
    return out


def _best(rel, window):
    """(score, shift, residuals) of the best match of rel within the window; score <= 1 passes."""
    best = None
    for shift, fk in window:
        res = compare_links(rel, fk)
        sc = _score(res)
        if best is None or sc < best[0]:
            best = (sc, shift, res)
    return best


def e9_report(samples, track, model):
    """Dynamic link/kinematic consistency: the leg-link entries change and track FK(q(t)).

    PASS: >= E9_MIN_SAMPLES complete samples inside the joint track; every one explained within
    E9_POSITION_TOL_M / E9_ROTATION_TOL_RAD by FK(q(t')) for some |t' - t| <= E9_ALIGN_WINDOW_S;
    gaps <= E9_MAX_GAP_S; every foot link turns >= E9_MIN_FOOT_SPAN_RAD away from its first
    analysed pose relative to the body.
    NO_MOTION: consistent, but the entries did not change enough: E9 not satisfied.
    FAIL: a sample not explained, or a gap. INCONCLUSIVE: too few samples.
    It does not measure the body's global height or tilt.
    """
    used, unexplained, worst, gaps = [], [], None, []
    first_rel = None
    turn = {link: 0.0 for link in FOOT_LINKS if link in model.links}
    stale_worst = 0.0
    skipped = {'incomplete': 0, 'outside_joint_track': 0}
    for s in samples:
        if s.codes:
            skipped['incomplete'] += 1
            continue
        window = _window_fk(track, model, s.t) if track.covers(s.t) else []
        if not window:
            skipped['outside_joint_track'] += 1
            continue
        rel = relative_links(s, model)
        if first_rel is None:
            first_rel = rel
        for link in turn:
            turn[link] = max(turn[link], angle_between(rel[link].q, first_rel[link].q))
        best = _best(rel, window)
        if used and s.t - used[-1] > E9_MAX_GAP_S:
            gaps.append({'from': used[-1], 'to': s.t})
        used.append(s.t)
        w = _worst(best[2])
        if worst is None or best[0] > worst['score']:
            worst = {'score': best[0], 't': s.t, 'shift_s': best[1], **w}
        if best[0] > 1.0:
            unexplained.append({'t': s.t, 'shift_s': best[1], **w})
        # Counterfactual: the same stream frozen at its first analysed sample.
        stale_worst = max(stale_worst, _best(first_rel, window)[0])
    rep = {'samples_used': len(used), 'skipped': skipped,
           'span_s': (used[-1] - used[0]) if used else 0.0,
           'worst': worst, 'unexplained_samples': len(unexplained),
           'first_unexplained': unexplained[:5], 'gaps_over_limit': gaps[:5],
           'foot_link_max_turn_from_first_rad': turn,
           'stale_stream_counterfactual_worst_score': stale_worst,
           'claim_scope': ('dynamic link/kinematic consistency of the pose stream relative to '
                           'the body; not an independent measurement of global body height or '
                           'tilt')}
    if unexplained or gaps:
        rep['verdict'] = FAIL
        rep['reason'] = 'samples not explained by FK(q(t))' if unexplained else 'pose gap'
    elif len(used) < E9_MIN_SAMPLES:
        rep['verdict'] = INCONCLUSIVE
        rep['reason'] = f'{len(used)} complete samples < {E9_MIN_SAMPLES}'
    elif not turn or min(turn.values()) < E9_MIN_FOOT_SPAN_RAD:
        rep['verdict'] = NO_MOTION
        rep['reason'] = ('consistent, but the foot links did not turn by '
                         f'{E9_MIN_FOOT_SPAN_RAD} rad: E9 not satisfied (it needs motion)')
    else:
        rep['verdict'] = PASS
        rep['reason'] = ('every sample explained by FK(q(t)); the entries changed; the stream '
                         f'frozen at its first sample would have scored {stale_worst:.1f} '
                         '(> 1 fails)')
    return rep


# ---------------------------------------------------------------- inputs
def attach_sim_time(pose_msgs, clock_msgs):
    """[(receive_s, transforms)] + [(receive_s, sim_s)] -> [(sim_s or None, transforms)]: each pose
    message gets the latest /clock received at or before it."""
    clocks = sorted(clock_msgs, key=lambda c: c[0])
    recv = [c[0] for c in clocks]
    out = []
    for r, transforms in pose_msgs:
        i = bisect.bisect_right(recv, r) - 1
        out.append((clocks[i][1] if i >= 0 else None, transforms))
    return out


def scan_server_log(text):
    """Lines of a launch log that contain a LOG_PATTERNS string."""
    return [ln.strip() for ln in text.splitlines() if any(p in ln for p in LOG_PATTERNS)]


def read_bag(path, topics=None):
    """{'poses': [(recv_s, transforms)], 'joints': [JointSample], 'clock': [(recv_s, sim_s)],
    'description': str or None} from a rosbag2 directory (read-only)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    topics = dict(TOPICS, **(topics or {}))
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {'poses': [], 'joints': [], 'clock': [], 'description': None,
           'topic_types': {k: types.get(v) for k, v in topics.items()}}
    wanted = {v: k for k, v in topics.items()}
    classes = {}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        key = wanted.get(topic)
        if key is None:
            continue
        cls = classes.setdefault(topic, get_message(types[topic]))
        msg = deserialize_message(data, cls)
        recv = t_ns * 1e-9
        if key == 'poses':
            out['poses'].append((recv, list(msg.transforms)))
        elif key == 'joints':
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            out['joints'].append(JointSample(st, dict(zip(msg.name, msg.position))))
        elif key == 'clock':
            out['clock'].append((recv, msg.clock.sec + msg.clock.nanosec * 1e-9))
        else:
            out['description'] = msg.data
    return out


def analyse(data, urdf, cfg, log_text=None):
    """The full report from read_bag-shaped data."""
    weld = fb.parse_robot_description(urdf, cfg.body_link, cfg.body_frame, cfg.weld_joint)
    model = link_model(urdf, cfg.body_link)
    names = {cfg.model_name, cfg.body_link, *model.links}
    timed = attach_sim_time(data['poses'], data['clock'])
    samples = [pose_sample(tr, t, names) for t, tr in timed]
    track = JointTrack(data['joints'], model.joints)
    hits = scan_server_log(log_text) if log_text is not None else None
    codes = {}
    for s in samples:
        for c in s.codes:
            codes[c] = codes.get(c, 0) + 1
    return {
        'schema': SCHEMA, 'analysis_sha256': source_sha256(), 'frozen': frozen_parameters(),
        'inputs': {'pose_messages': len(samples), 'joint_samples': len(track),
                   'clock_messages': len(data['clock']),
                   'joint_track_s': [track.t[0], track.t[-1]] if len(track) else None,
                   'pose_sample_codes': codes, 'links': list(model.links),
                   'joints': list(model.joints), 'topic_types': data.get('topic_types')},
        'e8_body_pose': body_report(samples, weld, cfg, cfg.model_name, cfg.body_link),
        'e8_provenance': provenance_report(samples, track, model, hits),
        'e9_link_consistency': e9_report(samples, track, model),
    }


def summary_lines(rep):
    b, p, e = rep['e8_body_pose'], rep['e8_provenance'], rep['e9_link_consistency']
    lines = [f'analysis sha256 {rep["analysis_sha256"]}',
             f'inputs: {rep["inputs"]["pose_messages"]} pose messages, '
             f'{rep["inputs"]["joint_samples"]} joint samples, '
             f'{rep["inputs"]["clock_messages"]} clock messages',
             f'E8 body pose (physics composition vs weld origin): {b["verdict"]}']
    if 'max_deviation' in b:
        m = b['max_deviation']
        lines.append(f'  max deviation {m["translation_m"]:.3e} m, {m["rotation_rad"]:.3e} rad, '
                     f'|dz| {m["abs_dz_m"]:.3e} m (limits {b["limits"]["translation_m"]:.3e} m, '
                     f'{b["limits"]["rotation_rad"]:.3e} rad); model entry max change '
                     f'{b["model_entry_max_change"]["translation_m"]:.3e} m')
    lines.append(f'E8 provenance (leg links written by Physics): {p["verdict"]} - {p["reason"]}')
    lines.append(f'  {p["samples_used"]} samples, {p["discriminating_samples"]} discriminating; '
                 f'FK(q) vs FK(0) up to {p["max_fk_q_vs_fk_0_rotation_rad"]:.3e} rad; '
                 f'min residual vs FK(0) on those links '
                 f'{p["min_residual_vs_fk_0_on_discriminating_links_rad"]}')
    if p['max_residual_vs_fk_q_by_link']:
        wt = max(v['translation_m'] for v in p['max_residual_vs_fk_q_by_link'].values())
        wr = max(v['rotation_rad'] for v in p['max_residual_vs_fk_q_by_link'].values())
        lines.append(f'  max residual vs FK(q): {wt:.3e} m, {wr:.3e} rad')
    lines.append(f'E9 link/kinematic consistency: {e["verdict"]} - {e["reason"]}')
    return lines


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--bag', required=True, help='rosbag2 directory (read-only)')
    ap.add_argument('--out', required=True, help='directory for link_check.json / .txt')
    ap.add_argument('--urdf', help='robot description file (default: /robot_description in the '
                                   'bag)')
    ap.add_argument('--server-log', help='launch log to scan for contradicting lines')
    ap.add_argument('--config-dir', help='directory of m61a_fixed_base.yaml')
    a = ap.parse_args(argv)
    try:
        cfg = fb.load_config(a.config_dir)
        data = read_bag(a.bag)
        if a.urdf:
            with open(a.urdf) as f:
                urdf = f.read()
        else:
            urdf = data['description']
        if not urdf:
            raise LinkCheckError('no robot description (none in the bag and no --urdf)')
        log_text = None
        if a.server_log:
            with open(a.server_log, errors='replace') as f:
                log_text = f.read()
        rep = analyse(data, urdf, cfg, log_text)
    except (LinkCheckError, fb.FixedBaseError, OSError, RuntimeError) as e:
        print(f'link check: {e}', file=sys.stderr)
        return 2
    rep['inputs']['bag'] = os.path.abspath(a.bag)
    rep['inputs']['urdf_source'] = a.urdf or 'bag /robot_description'
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, 'link_check.json'), 'w') as f:
        json.dump(rep, f, indent=2, sort_keys=True)
    text = '\n'.join(summary_lines(rep)) + '\n'
    with open(os.path.join(a.out, 'link_check.txt'), 'w') as f:
        f.write(text)
    print(text, end='')
    return 0


if __name__ == '__main__':
    sys.exit(main())
