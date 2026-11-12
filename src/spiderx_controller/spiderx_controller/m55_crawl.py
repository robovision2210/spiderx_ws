"""M5.5 free-base gait: a statically stable crawl with body shift, as start-stop cycle templates.

Pure Python (no ROS graph). Offline it designs and VALIDATES one gait cycle per speed level; at run
time the locomotion session only replays validated templates (m55_locomotion). Nothing here sends
anything.

Why a crawl with body shift (and not trot) for the first free-base walking
-------------------------------------------------------------------------
* SpiderX's legs carry 66 % of its 7.46 kg (CAD masses) and its COM sits on the diagonals of the
  stance rectangle: lifting any one leg leaves the COM within +-4 mm of the support-triangle edge
  (M4.5 found the same: wave/tripod_crawl FAIL static stability). Shifting the body before each
  swing moves the COM well inside the triangle of the three stance feet.
* A trot stands on two feet: its static margin is undefined (zero area) and its stability is
  dynamic. Without balance feedback, an IMU, real actuator data or a measured contact model, a
  slow trot is not shown to be stable by anything in this repository, and slow playback is not a
  proof of balance either. The approved M6.1 trot stays a FIXED-BASE test case only.
* A quasi-static crawl keeps the COM inside the support polygon by design, and each template is
  CHECKED to keep a static margin >= min_margin_m at every sample (every check_dt_s, on the dense
  path and on the controller spline) under explicit assumptions: flat ground, point feet at the
  derived tips, assumed contacts, no slip, CAD masses, a level body, quasi-static motion. That is
  an offline approximation, not a stability proof; the assumptions are what the local free-base
  runs must check (docs/M55_KEYBOARD_WALKING.md).

Cycle (forward, stride L > 0, body frame +y = forward)
------------------------------------------------------
Start and end in the neutral stance (all feet at the derived tips, no sway), at rest:
    S1 W(rear_left) S2 W(front_left) S3 W(rear_right) S4 W(front_right) S5
* shift S_k (four feet down): the body moves, smoothly and with zero end velocity, to
  advance k*L/4 plus a sway chosen so that the COM of the coming three-foot phase lies inside the
  support triangle inset by design_margin; S5 removes the sway (body at +L, feet at +L).
* swing W(leg) (three feet down, body still): the foot moves L forward along a world cycloid with
  zero velocity at lift-off and touch-down and lift_m apex height.
Reverse = the forward cycle played backwards in time (configurations and supports identical, so
the validation holds; the world displacement is -L). Joint velocities are zero at every phase
boundary, so the cycle starts and ends at rest - a natural, bounded dispatch unit.

Validation of a template (every check named, nothing clamped):
  ik            every leg solvable at every dense sample (URDF limits minus 0.05 rad margin)
  joint_speed   |qdot| <= max_joint_speed_rad_s (development bound, NOT an actuator rating)
  static_margin COM-to-support-edge distance >= min_margin_m at every dense sample (exact URDF
                mass model, configuration-dependent COM; 3- and 4-foot phases)
  spline        the controller's cubic-Hermite interpolation of the dispatched waypoints stays
                within spline_tolerance_rad of the dense IK path, and keeps the static margin
Labels: static margin = quasi-static APPROXIMATION (no dynamics, no slip, no contact model).
"""

from dataclasses import dataclass, field
import math

from spiderx_controller import gait_metrics as gm
from spiderx_controller import leg_kinematics as lk

ORDER_FORWARD = ('rear_left', 'front_left', 'rear_right', 'front_right')   # lateral sequence
SCHEMA = 'spiderx.m55.crawl_template/1'


class CrawlError(ValueError):
    """Invalid parameters, an infeasible design, or a failed validation."""


@dataclass(frozen=True)
class CrawlParams:
    stride_m: float                 # |L|: world advance per cycle (direction is separate)
    lift_m: float                   # swing apex height
    min_shift_s: float              # lower bound for each of the 5 shift phases
    min_swing_s: float              # lower bound for each of the 4 swings
    design_margin_m: float          # COM target inset inside each support triangle
    min_margin_m: float             # validation threshold for the static margin
    max_joint_speed_rad_s: float    # development bound (NOT an actuator rating)
    speed_use: float                # phases are timed for speed_use * max_joint_speed (< 1)
    point_dt_s: float               # dispatched waypoint spacing (phase durations are multiples)
    check_dt_s: float               # dense validation spacing (divides point_dt_s)
    spline_tolerance_rad: float     # max |Hermite(waypoints) - dense IK|

    def check(self):
        vals = (self.stride_m, self.lift_m, self.min_shift_s, self.min_swing_s,
                self.design_margin_m, self.min_margin_m, self.max_joint_speed_rad_s,
                self.speed_use, self.point_dt_s, self.check_dt_s, self.spline_tolerance_rad)
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                   and v > 0 for v in vals):
            raise CrawlError('every crawl parameter must be a finite number > 0')
        if self.min_margin_m > self.design_margin_m:
            raise CrawlError('min_margin_m must not exceed design_margin_m')
        if not self.speed_use < 1:
            raise CrawlError('speed_use must be < 1 (headroom below the joint-speed bound)')
        n = self.point_dt_s / self.check_dt_s
        if abs(n - round(n)) > 1e-9:
            raise CrawlError('check_dt_s must divide point_dt_s exactly')


def _smooth(u):
    """Cycloid profile 0 -> 1 with zero slope at both ends."""
    return u - math.sin(2 * math.pi * u) / (2 * math.pi)


@dataclass
class Phase:
    kind: str                       # 'shift' | 'swing'
    t0: float
    t1: float
    body0: tuple                    # body (x, y) at the start (world)
    body1: tuple                    # body (x, y) at the end
    leg: str = None                 # the swinging leg
    foot0: tuple = None             # its world foot position at lift-off
    support: tuple = ()             # stance legs during this phase


@dataclass
class CrawlCycle:
    """One designed FORWARD cycle: world body and feet as functions of time."""
    params: CrawlParams
    tips: dict                       # {leg: neutral derived foot tip (base_link, m)}
    phases: list = field(default_factory=list)
    sway: list = field(default_factory=list)    # designed (x, y) sway per shift 1..4

    @property
    def cycle_s(self):
        return self.phases[-1].t1

    @property
    def speed_m_s(self):
        return self.params.stride_m / self.cycle_s

    def retime(self, durations):
        t = 0.0
        for ph, d in zip(self.phases, durations):
            ph.t0, ph.t1 = t, t + d
            t += d

    # ------------------------------------------------------------ evaluation
    def state(self, t):
        """(body (x, y), {leg: world foot (x, y, z)}, swinging leg or None) at cycle time t."""
        t = min(max(t, 0.0), self.cycle_s)
        feet = {leg: tuple(tip) for leg, tip in self.tips.items()}
        for ph in self.phases:
            if t >= ph.t1 and ph.kind == 'swing':
                f = feet[ph.leg]
                feet[ph.leg] = (f[0], f[1] + self.params.stride_m, f[2])
            if ph.t0 <= t <= ph.t1 and (t < ph.t1 or ph is self.phases[-1]):
                u = (t - ph.t0) / (ph.t1 - ph.t0)
                if ph.kind == 'shift':
                    s = _smooth(u)
                    body = tuple(a + (b - a) * s for a, b in zip(ph.body0, ph.body1))
                    return body, feet, None
                f0 = ph.foot0
                y = f0[1] + self.params.stride_m * _smooth(u)
                z = f0[2] + self.params.lift_m * (1 - math.cos(2 * math.pi * u)) / 2
                feet[ph.leg] = (f0[0], y, z)
                return ph.body0, feet, ph.leg
        return self.phases[-1].body1, feet, None

    def foot_targets(self, t):
        """{leg: foot target in base_link} (no body rotation; body height constant)."""
        body, feet, _ = self.state(t)
        return {leg: (f[0] - body[0], f[1] - body[1], f[2]) for leg, f in feet.items()}


def _inset_triangle(tri, m):
    """The triangle `tri` (3 x (x, y), any orientation) shrunk inward by m (None if empty)."""
    pts = list(tri)
    area2 = ((pts[1][0] - pts[0][0]) * (pts[2][1] - pts[0][1])
             - (pts[1][1] - pts[0][1]) * (pts[2][0] - pts[0][0]))
    if area2 < 0:
        pts = [pts[0], pts[2], pts[1]]
    lines = []
    for i in range(3):
        a, b = pts[i], pts[(i + 1) % 3]
        ex, ey = b[0] - a[0], b[1] - a[1]
        n = math.hypot(ex, ey)
        nx, ny = -ey / n, ex / n                      # inward normal (counter-clockwise)
        lines.append(((a[0] + nx * m, a[1] + ny * m), (ex, ey)))
    out = []
    for i in range(3):
        (p, d), (q, e) = lines[i - 1], lines[i]
        den = d[0] * e[1] - d[1] * e[0]
        s = ((q[0] - p[0]) * e[1] - (q[1] - p[1]) * e[0]) / den
        out.append((p[0] + d[0] * s, p[1] + d[1] * s))
    if gm.stability_margin(((out[0][0] + out[1][0] + out[2][0]) / 3,
                            (out[0][1] + out[1][1] + out[2][1]) / 3), pts) is None:
        return None
    a2 = ((out[1][0] - out[0][0]) * (out[2][1] - out[0][1])
          - (out[1][1] - out[0][1]) * (out[2][0] - out[0][0]))
    return out if a2 > 0 else None


def _closest_in_polygon(p, poly):
    """The point of the convex polygon `poly` closest to p (p itself if inside)."""
    m = gm.stability_margin(p, poly)
    if m is not None and m >= 0:
        return p
    best, bd = None, math.inf
    n = len(poly)
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        ex, ey = b[0] - a[0], b[1] - a[1]
        t = max(0.0, min(1.0, ((p[0] - a[0]) * ex + (p[1] - a[1]) * ey) / (ex * ex + ey * ey)))
        c = (a[0] + t * ex, a[1] + t * ey)
        d = math.hypot(c[0] - p[0], c[1] - p[1])
        if d < bd:
            best, bd = c, d
    return best


class CrawlDesigner:
    """Designs forward crawl cycles for the real geometry and mass model."""

    def __init__(self, geoms, mass):
        self.geoms = geoms
        self.mass = mass
        self.tips = {leg: tuple(geoms[leg].tip0) for leg in lk.ALL_LEGS}
        self.joint_order = [n for leg in lk.ALL_LEGS for n in geoms[leg].joint_names]

    # ------------------------------------------------------------ IK and COM
    def joints(self, targets, reference=None):
        """{joint: q} for {leg: base_link target}, or raise CrawlError (never clamps)."""
        out = {}
        for leg in lk.ALL_LEGS:
            g = self.geoms[leg]
            ref = None if reference is None else [reference[n] for n in g.joint_names]
            r = lk.inverse(g, targets[leg], reference=ref, margin=lk.DEFAULT_MARGIN_RAD)
            if not r['ok']:
                raise CrawlError(f'ik: {leg} target {tuple(round(c, 4) for c in targets[leg])} '
                                 f'unreachable ({r["reason"]})')
            out.update(zip(g.joint_names, r['solution']))
        return out

    def com_world(self, body, targets, reference=None):
        q = self.joints(targets, reference)
        c = self.mass.com(q)
        return (body[0] + c[0], body[1] + c[1]), q

    # ------------------------------------------------------------ design
    def design(self, p):
        """The forward CrawlCycle for parameters p (sway chosen per the module docstring)."""
        p.check()
        L, cyc = p.stride_m, CrawlCycle(p, self.tips)
        feet = {leg: tuple(t) for leg, t in self.tips.items()}
        body, t = (0.0, 0.0), 0.0
        for k, leg in enumerate(ORDER_FORWARD, start=1):
            support = tuple(lg for lg in lk.ALL_LEGS if lg != leg)
            tri = [feet[lg][:2] for lg in support]
            inset = _inset_triangle(tri, p.design_margin_m)
            if inset is None:
                raise CrawlError(f'design: support triangle for {leg} too small for the margin')
            nominal = (0.0, k * L / 4)
            sway = (0.0, 0.0)
            for _ in range(6):                        # fixed point: COM depends on the posture
                b = (nominal[0] + sway[0], nominal[1] + sway[1])
                rel = {lg: (f[0] - b[0], f[1] - b[1], f[2]) for lg, f in feet.items()}
                c_start, _ = self.com_world(b, rel)
                lifted = dict(rel)
                f = feet[leg]
                lifted[leg] = (f[0] - b[0], f[1] + L - b[1], f[2])
                c_end, _ = self.com_world(b, lifted)
                mean = ((c_start[0] + c_end[0]) / 2, (c_start[1] + c_end[1]) / 2)
                target = _closest_in_polygon(mean, inset)
                corr = (target[0] - mean[0], target[1] - mean[1])
                sway = (sway[0] + corr[0], sway[1] + corr[1])
                if math.hypot(*corr) < 1e-7:
                    break
            new_body = (nominal[0] + sway[0], nominal[1] + sway[1])
            cyc.sway.append(sway)
            cyc.phases.append(Phase('shift', t, t + 1.0, body, new_body,
                                    support=tuple(lk.ALL_LEGS)))
            t += 1.0
            body = new_body
            cyc.phases.append(Phase('swing', t, t + 1.0, body, body, leg, feet[leg], support))
            t += 1.0
            f = feet[leg]
            feet[leg] = (f[0], f[1] + L, f[2])
        cyc.phases.append(Phase('shift', t, t + 1.0, body, (0.0, L),
                                support=tuple(lk.ALL_LEGS)))
        self.time_phases(cyc)
        return cyc

    def time_phases(self, cyc, samples=200):
        """Give every phase the shortest duration (a multiple of point_dt_s, at least its minimum)
        whose peak joint speed is <= speed_use * max_joint_speed. On a fixed path with a fixed
        normalized profile, peak |qdot| = max|dq/du| / duration, so this is exact up to the
        sampling of u (the dense validation re-checks the bound itself)."""
        p = cyc.params
        limit = p.speed_use * p.max_joint_speed_rad_s
        durations, ref = [], None
        for ph in cyc.phases:
            peak, prev = 0.0, None
            for i in range(samples + 1):
                t = ph.t0 + (ph.t1 - ph.t0) * i / samples
                q = self.joints(cyc.foot_targets(t), prev or ref)
                if prev is not None:
                    peak = max(peak, max(abs(q[n] - prev[n]) for n in q) * samples)
                prev = q
            ref = prev
            lo = p.min_shift_s if ph.kind == 'shift' else p.min_swing_s
            d = max(lo, peak / limit)
            durations.append(math.ceil(d / p.point_dt_s - 1e-9) * p.point_dt_s)
        cyc.retime(durations)
        cyc.durations = durations


# ---------------------------------------------------------------- templates (what is dispatched)
@dataclass
class CrawlTemplate:
    """A validated cycle as joint waypoints (forward or reverse).

    Phase metadata (every phase starts and ends at rest with four feet down):
      boundaries     waypoint index of each phase boundary: [0, ..., len(points) - 1]
      phase_info     [(kind, leg)] per phase ('shift', None) or ('swing', leg)
      boundary_body  planned body (x, y) in the cycle-start frame at each boundary (m)
    """
    params: CrawlParams
    direction: int                    # +1 forward (+y), -1 reverse
    joint_names: list
    points: list                      # [{time_from_start_s, positions, velocities}]
    report: dict
    boundaries: list = None
    phase_info: list = None
    boundary_body: list = None

    @property
    def displacement_m(self):
        return self.direction * self.params.stride_m

    @property
    def cycle_s(self):
        return self.points[-1]['time_from_start_s']

    def reversed(self):
        T, n = self.cycle_s, len(self.points) - 1
        pts = [{'time_from_start_s': round(T - p['time_from_start_s'], 9),
                'positions': list(p['positions']),
                'velocities': [-v for v in p['velocities']]} for p in reversed(self.points)]
        end = self.boundary_body[-1]
        return CrawlTemplate(self.params, -self.direction, list(self.joint_names), pts,
                             dict(self.report, direction=-self.direction),
                             [n - i for i in reversed(self.boundaries)],
                             list(reversed(self.phase_info)),
                             [(b[0] - end[0], b[1] - end[1])
                              for b in reversed(self.boundary_body)])


def _hermite(pa, pb, t):
    ta, tb = pa['time_from_start_s'], pb['time_from_start_s']
    dt = tb - ta
    s = (t - ta) / dt
    s2, s3 = s * s, s * s * s
    h00, h10, h01, h11 = 2 * s3 - 3 * s2 + 1, s3 - 2 * s2 + s, -2 * s3 + 3 * s2, s3 - s2
    return [h00 * qa + h10 * dt * va + h01 * qb + h11 * dt * vb
            for qa, va, qb, vb in zip(pa['positions'], pa['velocities'], pb['positions'],
                                      pb['velocities'])]


def build_template(designer, p, fd_eps=1e-4):
    """Design, sample and validate the forward template. Raises CrawlError on any failed check."""
    cyc = designer.design(p)
    names = designer.joint_order
    T = cyc.cycle_s
    ref = {n: 0.0 for n in names}

    def q_at(t, reference):
        return designer.joints(cyc.foot_targets(t), reference)

    # dense validation
    n_check = int(round(T / p.check_dt_s))
    dense, worst_margin, worst_speed = [], (math.inf, None), (0.0, None)
    margin_step, prev_m = 0.0, None        # largest change between consecutive samples
    prev = ref
    for i in range(n_check + 1):
        t = i * p.check_dt_s
        q = q_at(t, prev)
        body, feet, swing = cyc.state(t)
        c = designer.mass.com(q)
        com = (body[0] + c[0], body[1] + c[1])
        support = [feet[lg][:2] for lg in lk.ALL_LEGS if lg != swing]
        m = gm.stability_margin(com, support)
        if m is None or m < worst_margin[0]:
            worst_margin = (-math.inf if m is None else m, t)
        if m is not None and prev_m is not None and prev_m[1] == swing:
            margin_step = max(margin_step, abs(m - prev_m[0]))
        prev_m = None if m is None else (m, swing)
        if i:
            for n in names:
                v = abs(q[n] - prev[n]) / p.check_dt_s
                if v > worst_speed[0]:
                    worst_speed = (v, (n, t))
        dense.append((t, q))
        prev = q
    # dispatched waypoints: positions + central-difference velocities (0 at the ends)
    pts, prev = [], ref
    n_pts = int(round(T / p.point_dt_s))
    for i in range(n_pts + 1):
        t = i * p.point_dt_s
        q = q_at(t, prev)
        if i in (0, n_pts):
            v = [0.0] * len(names)
        else:
            a, b = q_at(t - fd_eps, q), q_at(t + fd_eps, q)
            v = [(b[n] - a[n]) / (2 * fd_eps) for n in names]
        pts.append({'time_from_start_s': round(t, 9), 'positions': [q[n] for n in names],
                    'velocities': v})
        prev = q
    # every phase boundary is a rest point (exact zero velocity; validated below as dispatched)
    bounds = [0]
    for d in cyc.durations:
        bounds.append(bounds[-1] + int(round(d / p.point_dt_s)))
    if bounds[-1] != n_pts:
        raise CrawlError(f'internal: phase boundaries end at {bounds[-1]}, not {n_pts}')
    for i in bounds:
        pts[i]['velocities'] = [0.0] * len(names)
    # the controller's spline vs the dense path, and its static margin
    spline_err, spline_margin = (0.0, None), (math.inf, None)
    for t, q in dense:
        k = min(int(t / p.point_dt_s), n_pts - 1)
        h = dict(zip(names, _hermite(pts[k], pts[k + 1], t)))
        e = max(abs(h[n] - q[n]) for n in names)
        if e > spline_err[0]:
            spline_err = (e, t)
        body, feet, swing = cyc.state(t)
        c = designer.mass.com(h)
        support = [feet[lg][:2] for lg in lk.ALL_LEGS if lg != swing]
        m = gm.stability_margin((body[0] + c[0], body[1] + c[1]), support)
        m = -math.inf if m is None else m
        if m < spline_margin[0]:
            spline_margin = (m, t)
    report = {
        'schema': SCHEMA, 'direction': 1, 'stride_m': p.stride_m, 'cycle_s': T,
        'speed_m_s': cyc.speed_m_s, 'lift_m': p.lift_m, 'points': len(pts),
        'phase_durations_s': [round(d, 9) for d in cyc.durations],
        'sway_m': [list(s) for s in cyc.sway],
        'min_static_margin_m': worst_margin[0], 'min_static_margin_time_s': worst_margin[1],
        'spline_min_static_margin_m': spline_margin[0],
        'max_joint_speed_rad_s': worst_speed[0], 'max_joint_speed_at': worst_speed[1],
        'max_spline_error_rad': spline_err[0],
        'max_margin_step_between_samples_m': margin_step,
        'labels': {'static_margin': 'quasi-static approximation (CAD masses, point feet, '
                                    'flat ground, no slip, no dynamics)',
                   'static_margin_method': (
                       f'minimum over samples every {p.check_dt_s} s of the dense IK path and '
                       f'of the controller spline through the waypoints; a sampled value, not '
                       f'a continuous bound (max_margin_step_between_samples_m shows the '
                       f'change between neighbouring samples)'),
                   'support': ('all four feet during shifts; the three feet other than the '
                               'swinging leg for the WHOLE swing, lift-off to touch-down; feet '
                               'are points at the derived foot tips on the plane of the stance '
                               'tips; contacts are assumed (no forces, friction or slip)'),
                   'com': ('whole-robot COM from the URDF inertials (CAD steel-density '
                           'masses) by forward kinematics at each sampled configuration, '
                           'projected vertically; body level, height constant'),
                   'joint_speed': 'development bound, not an actuator rating'},
    }
    failures = []
    if worst_margin[0] < p.min_margin_m:
        failures.append(f'static_margin: {worst_margin[0]:.4f} m < {p.min_margin_m} m at '
                        f't = {worst_margin[1]} s')
    if spline_margin[0] < p.min_margin_m:
        failures.append(f'spline static_margin: {spline_margin[0]:.4f} m < {p.min_margin_m} m')
    if worst_speed[0] > p.max_joint_speed_rad_s:
        failures.append(f'joint_speed: {worst_speed[0]:.4f} rad/s > {p.max_joint_speed_rad_s} '
                        f'({worst_speed[1]})')
    if spline_err[0] > p.spline_tolerance_rad:
        failures.append(f'spline: {spline_err[0]:.5f} rad > {p.spline_tolerance_rad} rad')
    report['failures'] = failures
    if failures:
        raise CrawlError('; '.join(failures))
    info = [(ph.kind, ph.leg) for ph in cyc.phases]
    body = [tuple(cyc.phases[0].body0)] + [tuple(ph.body1) for ph in cyc.phases]
    report['phase_boundaries'] = list(bounds)
    return CrawlTemplate(p, 1, list(names), pts, report, bounds, info, body)


def load_designer(urdf_root=None):
    if urdf_root is None:
        from spiderx_controller.config_check import load_urdf
        urdf_root = load_urdf()
    geoms = lk.load_all_geometries(urdf_root=urdf_root)
    return CrawlDesigner(geoms, gm.MassModel(urdf_root, geoms))


__all__ = ['CrawlParams', 'CrawlCycle', 'CrawlDesigner', 'CrawlTemplate', 'CrawlError',
           'build_template', 'load_designer', 'ORDER_FORWARD', 'SCHEMA']
