"""M4.5 OFFLINE gait metrics and per-gait verdict.

Inputs: a GaitSpec (gait_config), the URDF geometry (leg_kinematics) and the sampled IK of one
cycle (gait_kinematics). Outputs: Check records (gait_results) and plain numeric metrics.

What each metric is - and is NOT (labels in gait_results):
* static_stability [approximation]: quasi-static. The whole-robot COM is computed from the URDF
  <inertial> data (CAD, steel-density masses) at each sample's joint angles and projected on the
  ground; its signed distance to the edges of the support polygon (convex hull of the stance feet,
  point feet at the derived tips, flat ground, level body, no inertial forces) is the margin
  (positive = inside). With fewer than three stance feet the polygon is degenerate and the gait is
  statically unstable by definition at that sample; dynamic stability is never evaluated.
* joint_speed [sampled]: central-difference joint speeds compared with spiderx_legs.yaml
  max_joint_velocity_rad_s, a SIMULATION_PLACEHOLDER (not a servo specification).
* transition_velocity_jump [exact]: foot velocity just before vs at every lift-off/touch-down.
  transition_acceleration_jump [sampled, information only]: the cycloid lift has a vertical
  acceleration step there.
* energy proxies [heuristic, information only], all per metre of body travel: world-frame foot
  path length, summed joint travel, and a lift-work proxy sum(m_leg g dz+) of each leg's COM.
  They are comparison proxies, NOT energy, work or power measurements (no actuator model).

Offline only: no ROS runtime, no Gazebo, never commands the robot.
"""

from dataclasses import dataclass
import math

from spiderx_controller import gait_kinematics as gk
from spiderx_controller import gait_phase as gp
from spiderx_controller import gait_trajectory as gt
from spiderx_controller import leg_kinematics as lk
from spiderx_controller.gait_results import Check

G = 9.80665
PATH_POINTS_PER_SWING = 400       # numeric arc length of one swing curve


# ---------------------------------------------------------------- URDF mass model
class MassModel:
    """Link masses and COMs from the URDF, evaluated at any set of joint angles (base_link frame)."""

    def __init__(self, urdf_root, geoms):
        self.joints = lk.parse_urdf_joints(urdf_root)
        by_child = {j['child']: j for j in self.joints.values()}
        self.links = []                               # (name, mass, local COM, joint chain)
        for link in urdf_root.findall('link'):
            inertial = link.find('inertial')
            if inertial is None:
                continue
            mass = float(inertial.find('mass').get('value'))
            o = inertial.find('origin')
            com_local = lk._vec(o.get('xyz') if o is not None else None)
            chain, name = [], link.get('name')
            while name in by_child:
                chain.append(by_child[name])
                name = by_child[name]['parent']
            self.links.append((link.get('name'), mass, com_local, list(reversed(chain))))
        self.total_mass = sum(m for _, m, _, _ in self.links)
        # base_link pose in the URDF root frame (identity for SpiderX, computed, not assumed)
        self._base = self._pose_of_chain(self._chain_to('base_link', by_child), {})
        # each leg = every link below that leg's hip joint
        self.leg_links = {leg: [name for name, _, _, chain in self.links
                                if geoms[leg].joint_names[0] in [j['name'] for j in chain]]
                          for leg in lk.ALL_LEGS}
        self.leg_mass = {leg: sum(m for name, m, _, _ in self.links if name in names)
                         for leg, names in self.leg_links.items()}

    @staticmethod
    def _chain_to(link, by_child):
        chain = []
        while link in by_child:
            chain.append(by_child[link])
            link = by_child[link]['parent']
        return list(reversed(chain))

    @staticmethod
    def _pose_of_chain(chain, q_by_name):
        pose = lk._Pose()
        for j in chain:
            pose = pose * lk._joint_transform(j, q_by_name.get(j['name'], 0.0))
        return pose

    def _com_root(self, q_by_name, link_names=None):
        total, acc = 0.0, (0.0, 0.0, 0.0)
        for name, mass, com_local, chain in self.links:
            if link_names is not None and name not in link_names:
                continue
            c = self._pose_of_chain(chain, q_by_name).apply(com_local)
            total += mass
            acc = tuple(a + mass * ci for a, ci in zip(acc, c))
        return tuple(a / total for a in acc)

    def _to_base(self, p):
        return lk._mv(lk._t(self._base.R), lk._sub(p, self._base.p))

    def com(self, q_by_name):
        """Whole-robot COM (base_link, m). Joints not given are at 0."""
        return self._to_base(self._com_root(q_by_name))

    def leg_com(self, leg, q_by_name):
        """COM of the links below `leg`'s hip joint (base_link, m)."""
        return self._to_base(self._com_root(q_by_name, set(self.leg_links[leg])))


# ---------------------------------------------------------------- 2-D geometry
def convex_hull(points):
    """Monotone-chain convex hull (counter-clockwise, no repeated points). points: [(x, y)]."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def stability_margin(com_xy, support_xy):
    """Signed distance (m) from com_xy to the support polygon boundary (+ inside, - outside).

    None if fewer than 3 non-collinear support points (no support polygon).
    """
    hull = convex_hull(support_xy)
    if len(hull) < 3:
        return None
    margin = math.inf
    for i in range(len(hull)):
        a, b = hull[i], hull[(i + 1) % len(hull)]
        ex, ey = b[0] - a[0], b[1] - a[1]
        length = math.hypot(ex, ey)
        # left of a counter-clockwise edge = inside
        d = (ex * (com_xy[1] - a[1]) - ey * (com_xy[0] - a[0])) / length
        margin = min(margin, d)
    return margin


# ---------------------------------------------------------------- evaluation
@dataclass
class GaitEvaluation:
    spec: object                 # GaitSpec
    kinematics: object           # gait_kinematics.CycleKinematics
    checks: list                 # [Check], report order
    metrics: dict                # name -> number (or None)
    series: dict                 # per-sample series for reports/plots

    @property
    def passed(self):
        return all(c.passed for c in self.checks if c.applicable)

    @property
    def failed_checks(self):
        return [c.name for c in self.checks if c.applicable and not c.passed]


def _q_by_name(kin, k):
    out = {}
    for s in kin.legs.values():
        if s.q[k] is None:
            return None
        out.update(zip(s.joint_names, s.q[k]))
    return out


def stability_series(spec, kin, mass):
    """Per sample: support count, COM (x, y, z) and static margin (None if < 3 feet or no IK)."""
    support, com, margin = [], [], []
    for k, sample in enumerate(kin.samples):
        legs = sample['support']
        support.append(len(legs))
        q = _q_by_name(kin, k)
        if q is None:
            com.append(None)
            margin.append(None)
            continue
        c = mass.com(q)
        com.append(c)
        pts = [sample['feet'][leg].target[:2] for leg in legs]
        margin.append(stability_margin(c[:2], pts) if len(pts) >= 3 else None)
    return {'support_count': support, 'com_m': com, 'static_margin_m': margin}


def check_static_stability(spec, kin, series, threshold):
    margins = series['static_margin_m']
    known = [m for m in margins if m is not None]
    three_plus = [n >= 3 for n in series['support_count']]
    c = Check('static_stability', True, min(known) if known else None, threshold, 'm',
              'approximation',
              'quasi-static COM-to-support-edge margin (URDF CAD masses, point feet, flat ground, '
              'level body, no dynamics); required only for gaits with requires_static_stability',
              applicable=spec.requires_static_stability)
    for k, (m, ok3) in enumerate(zip(margins, three_plus)):
        where = f'sample {k} (u={kin.samples[k]["u"]:.3f})'
        if not ok3:
            c.add_failure(f'{where}: only {series["support_count"][k]} feet in stance '
                          '(statically unstable by definition)')
        elif m is None:
            c.add_failure(f'{where}: no IK solution, margin not computable')
        elif m < threshold:
            c.add_failure(f'{where}: margin {m * 1000:.2f} mm < {threshold * 1000:.2f} mm '
                          f'(support {list(kin.samples[k]["support"])})')
    c.passed = c.failure_count == 0
    return c


def check_joint_speed(kin, reference):
    worst, where = None, None
    c = Check('joint_speed', True, None, reference, 'rad/s', 'sampled',
              'max central-difference joint speed vs spiderx_legs.yaml max_joint_velocity_rad_s '
              '(a SIMULATION_PLACEHOLDER, not a servo specification)')
    for leg, s in kin.legs.items():
        for k, sp in enumerate(gk.joint_speeds(s.q, kin.dt_s)):
            if sp is None:
                continue
            m = max(sp)
            if worst is None or m > worst:
                worst, where = m, (leg, k, s.joint_names[sp.index(m)])
            if m > reference:
                c.add_failure(f'{leg} sample {k} (u={kin.samples[k]["u"]:.3f}): '
                              f'{s.joint_names[sp.index(m)]} {m:.3f} rad/s')
    c.value = worst
    c.passed = worst is not None and c.failure_count == 0
    if where:
        c.description += f'; worst at {where[0]} {where[2]}, sample {where[1]}'
    return c


def _transition_events(spec, leg):
    phi, beta = spec.phase_offset(leg), spec.duty_factor
    return (('lift-off', phi), ('touch-down', (phi + 1.0 - beta) % 1.0))


def check_transition_velocity(spec, tol):
    worst = 0.0
    c = Check('transition_velocity_jump', True, None, tol, 'm/s', 'exact',
              'foot velocity jump at every lift-off and touch-down (analytic trajectory)')
    for leg in lk.ALL_LEGS:
        for event, u in _transition_events(spec, leg):
            before = gt.foot_sample(spec, leg, (u - 1e-9) % 1.0).velocity
            after = gt.foot_sample(spec, leg, u).velocity
            jump = math.dist(before, after)
            worst = max(worst, jump)
            if jump > tol:
                c.add_failure(f'{leg} {event} (u={u:.3f}): {jump:.2e} m/s')
    c.value, c.passed = worst, c.failure_count == 0
    return c


def transition_acceleration_jump(spec):
    """Largest foot acceleration step (m/s^2) at a transition (numeric, information only)."""
    dt = 1e-6
    du = dt / spec.cycle_period_s
    worst = 0.0
    for leg in lk.ALL_LEGS:
        for _, u in _transition_events(spec, leg):
            def acc(u0):
                v0 = gt.foot_sample(spec, leg, u0 % 1.0).velocity
                v1 = gt.foot_sample(spec, leg, (u0 + du) % 1.0).velocity
                return tuple((b - a) / dt for a, b in zip(v0, v1))
            worst = max(worst, math.dist(acc(u - 3 * du), acc(u + du)))
    return worst


def swing_path_length(spec):
    """World-frame arc length (m) of one swing (the stance foot does not move in the world)."""
    S, h, n = spec.stride_length_m, spec.step_height_m, PATH_POINTS_PER_SWING
    pts = [(S * (s - math.sin(2 * math.pi * s) / (2 * math.pi)),
            h * (1 - math.cos(2 * math.pi * s)) / 2) for s in (i / n for i in range(n + 1))]
    return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))


def energy_proxies(spec, kin, mass):
    """Heuristic proxies per metre of body travel (None if IK failed anywhere)."""
    stride = spec.stride_length_m
    out = {'steps_per_m': 4.0 / stride,
           'foot_path_per_m': 4.0 * swing_path_length(spec) / stride,
           'joint_travel_rad_per_m': None, 'lift_work_proxy_j_per_m': None}
    if not kin.all_solved:
        return out
    travel = sum(sum(st) for s in kin.legs.values() for st in gk.joint_steps(s.q))
    out['joint_travel_rad_per_m'] = travel / stride
    work = 0.0
    for leg, s in kin.legs.items():
        names = s.joint_names
        zs = [mass.leg_com(leg, dict(zip(names, q)))[2] for q in s.q]
        rise = sum(max(0.0, zs[k] - zs[k - 1]) for k in range(len(zs)))   # cyclic
        work += mass.leg_mass[leg] * G * rise
    out['lift_work_proxy_j_per_m'] = work / stride
    return out


def evaluate_gait(spec, cfg, geoms, mass, n=None):
    """Full offline evaluation of one gait: kinematic checks + metric checks + metrics + series."""
    a = cfg.analysis
    n = n or a.samples_per_cycle
    kin = gk.solve_cycle(spec, geoms, n, cfg.margin_rad)
    checks = gk.kinematic_checks(kin, cfg)
    series = stability_series(spec, kin, mass)
    checks.append(check_static_stability(spec, kin, series, a.min_static_margin_m))
    checks.append(check_joint_speed(kin, a.joint_speed_reference_rad_s))
    checks.append(check_transition_velocity(spec, a.velocity_jump_tol_m_s))
    support = gp.support_summary(spec, n)
    known = [m for m in series['static_margin_m'] if m is not None]
    metrics = {
        'duty_factor': spec.duty_factor, 'cycle_period_s': spec.cycle_period_s,
        'body_speed_m_s': spec.body_speed_m_s, 'step_length_m': spec.step_length_m,
        'stride_length_m': spec.stride_length_m, 'step_height_m': spec.step_height_m,
        'min_support_feet': support['min_support'], 'max_support_feet': support['max_support'],
        'fraction_two_feet': support['fraction_by_count'].get(2, 0.0),
        'fraction_three_feet': support['fraction_by_count'].get(3, 0.0),
        'fraction_four_feet': support['fraction_by_count'].get(4, 0.0),
        'min_static_margin_m': min(known) if known else None,
        'fraction_statically_stable': (sum(1 for m in series['static_margin_m']
                                           if m is not None and m > 0) / n),
        'max_joint_speed_rad_s': next(c.value for c in checks if c.name == 'joint_speed'),
        'min_singularity_margin_rad': next(c.value for c in checks
                                           if c.name == 'singularity_margin'),
        'min_joint_limit_margin_rad': next(c.value for c in checks
                                           if c.name == 'joint_limit_margin'),
        'max_joint_step_rad': next(c.value for c in checks if c.name == 'joint_continuity'),
        'max_transition_velocity_jump_m_s': next(c.value for c in checks
                                                 if c.name == 'transition_velocity_jump'),
        'max_transition_acceleration_jump_m_s2': transition_acceleration_jump(spec),
        **energy_proxies(spec, kin, mass),
    }
    return GaitEvaluation(spec, kin, checks, metrics, series)


def load_inputs(config_path=None, config_dir=None, urdf_root=None):
    """(GaitConfig, {leg: LegGeometry}, MassModel) from the installed (or given) files."""
    from spiderx_controller import gait_config as gc
    cfg = gc.load_gait_config(config_path=config_path, config_dir=config_dir)
    if urdf_root is None:
        from spiderx_controller.config_check import load_urdf
        urdf_root = load_urdf()
    try:
        geoms = lk.load_all_geometries(urdf_root=urdf_root, config_dir=config_dir)
    except lk.KinematicsError as e:
        raise gc.GaitConfigError(f'geometry: {e}') from e
    return cfg, geoms, MassModel(urdf_root, geoms)


def evaluate_all(cfg, geoms, mass, names=None):
    """Evaluate every gait (or only `names`), in configuration order."""
    specs = cfg.gaits if names is None else [cfg.gait(n) for n in names]
    return [evaluate_gait(spec, cfg, geoms, mass) for spec in specs]


__all__ = ['MassModel', 'convex_hull', 'stability_margin', 'GaitEvaluation', 'evaluate_gait',
           'evaluate_all', 'load_inputs', 'swing_path_length', 'energy_proxies']
