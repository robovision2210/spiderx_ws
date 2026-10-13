"""M6.1-A: ground clearance of the WELDED (fixed-base) SpiderX over the M6.1 motion (offline).

Pure analysis (numpy, no ROS graph). It answers: "if base_link is welded at height h above the
ground plane (zero roll and pitch), how close does ANY collision geometry come to the ground while
the one approved M6.1 trot cycle runs?" and derives a provisional mounting height from that.

Geometry source of truth: the EXPANDED URDF (xacro). Every <collision> of every link is used:
STL meshes (all unique vertices, mesh scale and collision origin applied; the lowest point of a
triangle mesh is always one of its vertices, so the per-configuration minimum is exact for the
mesh) and analytic primitives (cylinder, box, sphere). Nothing is typed in.

Frames and conventions
----------------------
* Kinematics in base_link (+x right, +y front, +z up), URDF semantics
  T_parent_child(q) = Trans(xyz) . Rz Ry Rx(rpy) . Rot(axis, q).
* The weld holds base_link at T_world_body = Trans(x, y, h) . Rz(yaw) (zero roll and pitch, the
  only arrangement the fixed-base wrapper allows), so the world height of a body-frame point v is
  z_world = h + z_B(v): yaw and x/y do not change heights.
* Ground: the static ground_plane of spiderx_fortress.sdf, plane z = 0, normal +z.
* Units: metres, radians, seconds.

Sampled evidence versus guaranteed bound
----------------------------------------
* "sampled" minima are exact at the evaluated configurations only.
* "bound" values are guaranteed lower bounds over a continuous set, from a Lipschitz argument:
  for a point fixed in link L, |dz/dq_j| <= dist(point, axis_j) <= R_j(L), where
  R_j(L) = sum of the joint-origin offset lengths from joint j down to L plus the largest distance
  of L's geometry from L's origin (triangle inequality; rotations keep lengths). Hence
    - over time between two samples:   min z >= min(z_a, z_b) - (sum_j max|qdot_j| R_j) * dt / 2,
      with max|qdot_j| computed EXACTLY per cubic-Hermite segment (its derivative is quadratic);
    - over a joint box |q_j - q_ref,j| <= d_j:  min z >= z(q_ref) - sum_j d_j R_j.
  The bounds are conservative (never optimistic) and are reported next to the sampled values.
"""

from dataclasses import dataclass, field
import math
import os

import numpy as np

from spiderx_controller import leg_kinematics as lk

SCHEMA = 'spiderx.m61a.clearance/1'
GROUND_Z_M = 0.0                     # spiderx_fortress.sdf ground_plane: plane z = 0, normal +z
TRACKING_TOLERANCE_RAD = 0.05        # M6.1 path tolerance / client abort (m61_limits.yaml)
START_POSE_TOLERANCE_RAD = 0.05      # m6_envelope.START_POSE_TOLERANCE_RAD (readiness start box)
DEFAULT_SAMPLES_PER_SEGMENT = 200    # cycle samples per 0.5 s Hermite segment (2.5 ms spacing)
LIMIT_OVERSHOOT_RAD = 0.02           # joint-limit box widened for limit-constraint softness
                                     # (provisional allowance; DART enforces limits as constraints)
BNB_EPS_M = 1e-3                     # branch and bound: proven gap between best found and bound
BNB_MAX_NODES = 200000               # branch and bound node budget (reported if it is reached)


class ClearanceError(ValueError):
    """Malformed geometry or trajectory input."""


# ---------------------------------------------------------------- small helpers
def _rpy_matrix(rpy):
    return np.array(lk.rot_rpy(*rpy), dtype=float)


def _axis_angle(axis, q):
    return np.array(lk.rot_axis_angle(axis, q), dtype=float)


def _vec(s, default=(0.0, 0.0, 0.0)):
    return tuple(float(v) for v in s.split()) if s else default


@dataclass
class Geometry:
    """Collision geometry of one link, in that link's frame."""
    vertices: np.ndarray                       # (N, 3) unique mesh vertices (may be empty)
    cylinders: list = field(default_factory=list)   # (R, c, radius, length) axis = R[:, 2]
    boxes: list = field(default_factory=list)       # (R, c, half_extents)
    spheres: list = field(default_factory=list)     # (c, radius)
    sources: list = field(default_factory=list)     # human-readable list of what was loaded

    def max_extent(self):
        """Largest distance of any geometry point from the link origin (for the Lipschitz radius)."""
        r = float(np.max(np.linalg.norm(self.vertices, axis=1))) if len(self.vertices) else 0.0
        for _, c, rad, length in self.cylinders:
            r = max(r, float(np.linalg.norm(c)) + math.hypot(rad, length / 2))
        for _, c, half in self.boxes:
            r = max(r, float(np.linalg.norm(c)) + float(np.linalg.norm(half)))
        for c, rad in self.spheres:
            r = max(r, float(np.linalg.norm(c)) + rad)
        return r

    def min_z(self, R, p):
        """Exact lowest z of this geometry when the link frame is at (R, p) in base_link."""
        zs = []
        if len(self.vertices):
            zs.append(float(np.min(self.vertices @ R[2, :] + p[2])))
        for Rc, c, rad, length in self.cylinders:
            a = R @ Rc[:, 2]                                   # axis in base_link
            cz = float(R[2, :] @ c + p[2])
            zs.append(cz - length / 2 * abs(a[2]) - rad * math.sqrt(max(0.0, 1.0 - a[2] ** 2)))
        for Rc, c, half in self.boxes:
            Rw = R @ Rc
            cz = float(R[2, :] @ c + p[2])
            zs.append(cz - float(np.sum(np.abs(Rw[2, :]) * half)))
        for c, rad in self.spheres:
            zs.append(float(R[2, :] @ c + p[2]) - rad)
        return min(zs) if zs else math.inf


def _load_link_geometry(link_el):
    verts, geom = [], Geometry(vertices=np.zeros((0, 3)))
    for col in link_el.findall('collision'):
        o = col.find('origin')
        Rc = _rpy_matrix(_vec(o.get('rpy') if o is not None else None))
        pc = np.array(_vec(o.get('xyz') if o is not None else None))
        g = col.find('geometry')
        if g is None or len(g) != 1:
            raise ClearanceError(f'{link_el.get("name")}: collision without exactly one geometry')
        shape = g[0]
        if shape.tag == 'mesh':
            path = lk.resolve_mesh_path(shape.get('filename'))
            scale = np.array(_vec(shape.get('scale'), (1.0, 1.0, 1.0)))
            raw = np.unique(np.round(np.array(lk.load_stl_vertices(path), dtype=float), 9), axis=0)
            verts.append((raw * scale) @ Rc.T + pc)
            geom.sources.append(f'mesh {os.path.basename(path)} ({len(raw)} unique vertices)')
        elif shape.tag == 'cylinder':
            geom.cylinders.append((Rc, pc, float(shape.get('radius')), float(shape.get('length'))))
            geom.sources.append(f'cylinder r={shape.get("radius")} l={shape.get("length")}')
        elif shape.tag == 'box':
            geom.boxes.append((Rc, pc, np.array(_vec(shape.get('size'))) / 2))
            geom.sources.append(f'box {shape.get("size")}')
        elif shape.tag == 'sphere':
            geom.spheres.append((pc, float(shape.get('radius'))))
            geom.sources.append(f'sphere r={shape.get("radius")}')
        else:
            raise ClearanceError(f'{link_el.get("name")}: unsupported collision {shape.tag}')
    if verts:
        geom.vertices = np.unique(np.concatenate(verts), axis=0)
    return geom


class CollisionModel:
    """Every link's collision geometry plus the URDF kinematic tree (base_link frame)."""

    def __init__(self, urdf_root, body_link='base_link'):
        self.joints = lk.parse_urdf_joints(urdf_root)
        links = {el.get('name'): el for el in urdf_root.findall('link')}
        self.geometry = {n: _load_link_geometry(el) for n, el in links.items()
                         if el.findall('collision')}
        children = {j['child'] for j in self.joints.values()}
        roots = [n for n in links if n not in children]
        if len(roots) != 1:
            raise ClearanceError(f'URDF must have one root link, found {roots}')
        self.root = roots[0]
        self.body_link = body_link
        self.by_child = {j['child']: j for j in self.joints.values()}
        # chains (root -> link) and the revolute joints that move each link
        self.chain = {n: self._chain(n) for n in links}
        self.movers = {n: [j['name'] for j in self.chain[n]
                           if j['type'] in ('revolute', 'continuous')] for n in links}
        self.radius = {n: self._radii(n) for n in self.geometry}
        self.revolute = sorted(j['name'] for j in self.joints.values()
                               if j['type'] in ('revolute', 'continuous'))
        self.limits = {n: (self.joints[n]['lower'], self.joints[n]['upper'])
                       for n in self.revolute}
        T = self._pose_root(body_link, {})
        self._root_in_body = (T[0].T, -T[0].T @ T[1])          # inverse of T_root_body

    def _chain(self, link):
        out, n = [], link
        while n != self.root:
            if n not in self.by_child:
                raise ClearanceError(f'link {link} is not connected to the root {self.root}')
            j = self.by_child[n]
            out.append(j)
            n = j['parent']
        return list(reversed(out))

    def _radii(self, link):
        """{revolute joint: R_j(link)} conservative distance bound from joint j's axis."""
        chain, ext = self.chain[link], self.geometry[link].max_extent()
        out = {}
        for i, j in enumerate(chain):
            if j['type'] not in ('revolute', 'continuous'):
                continue
            out[j['name']] = sum(math.sqrt(sum(c * c for c in k['xyz']))
                                 for k in chain[i + 1:]) + ext
        return out

    def _pose_root(self, link, q):
        R, p = np.eye(3), np.zeros(3)
        for j in self.chain[link]:
            Rj = _rpy_matrix(j['rpy'])
            if j['type'] in ('revolute', 'continuous'):
                Rj = Rj @ _axis_angle(j['axis'], q.get(j['name'], 0.0))
            elif j['type'] != 'fixed':
                raise ClearanceError(f'unsupported joint type {j["type"]} ({j["name"]})')
            p = R @ np.array(j['xyz']) + p
            R = R @ Rj
        return R, p

    def pose(self, link, q):
        """(R, p) of `link` in the body frame for joint values q {name: rad} (missing = 0)."""
        R, p = self._pose_root(link, q)
        Rb, pb = self._root_in_body
        return Rb @ R, Rb @ p + pb

    def link_min_z(self, q, links=None):
        """{link: exact lowest body-frame z of its collision geometry} at configuration q."""
        return {n: self.geometry[n].min_z(*self.pose(n, q))
                for n in (links if links is not None else self.geometry)}

    def lipschitz(self, link, rates):
        """sum_j rates[j] * R_j(link): a bound on |dz/dt| (rates = |qdot|) or on |dz| (rates = box)."""
        return sum(rates.get(j, 0.0) * r for j, r in self.radius[link].items())


# ---------------------------------------------------------------- trajectory helpers
def hermite_speed_bounds(pa, pb):
    """Exact max |qdot| per joint on one cubic-Hermite segment (positions + velocities)."""
    dt = pb['time_from_start_s'] - pa['time_from_start_s']
    if not dt > 0:
        raise ClearanceError('non-increasing waypoint times')
    out = []
    for qa, va, qb, vb in zip(pa['positions'], pa['velocities'], pb['positions'],
                              pb['velocities']):
        # qdot(s) = c0 + c1 s + c2 s^2 (s in [0, 1]), from the Hermite basis derivatives / dt
        c0 = va
        c1 = (-6 * qa + 6 * qb) / dt - 4 * va - 2 * vb
        c2 = (6 * qa - 6 * qb) / dt + 3 * va + 3 * vb
        cands = [0.0, 1.0]
        if abs(c2) > 1e-15:
            s = -c1 / (2 * c2)
            if 0.0 < s < 1.0:
                cands.append(s)
        out.append(max(abs(c0 + c1 * s + c2 * s * s) for s in cands))
    return out


def hermite_q(pa, pb, t):
    """Cubic-Hermite positions at time t (same basis as m61_trot_cycle._segment)."""
    ta, tb = pa['time_from_start_s'], pb['time_from_start_s']
    dt = tb - ta
    s = (t - ta) / dt
    s2, s3 = s * s, s * s * s
    h00, h10, h01, h11 = 2 * s3 - 3 * s2 + 1, s3 - 2 * s2 + s, -2 * s3 + 3 * s2, s3 - s2
    return [h00 * qa + h10 * dt * va + h01 * qb + h11 * dt * vb
            for qa, va, qb, vb in zip(pa['positions'], pa['velocities'], pb['positions'],
                                      pb['velocities'])]


# ---------------------------------------------------------------- the envelopes
def _worst(model, per_link):
    link = min(per_link, key=per_link.get)
    return link, per_link[link]


def commanded_envelope(model, trajectory, samples_per_segment=DEFAULT_SAMPLES_PER_SEGMENT,
                       box_rad=0.0):
    """Clearance over the commanded cubic-Hermite spline (every segment, end points included).

    box_rad > 0 adds a per-joint box |q - q_ref| <= box_rad around EVERY sample (tracking error
    allowed before the controller/client aborts). Returns sampled minima and guaranteed bounds.
    """
    names = list(trajectory['joint_names'])
    pts = trajectory['points']
    box = {n: box_rad for n in names}
    links = sorted(model.geometry)
    sampled = {n: (math.inf, None) for n in links}       # exact at samples (no box)
    bound = {n: math.inf for n in links}                 # continuous + box lower bound
    for a in range(len(pts) - 1):
        pa, pb = pts[a], pts[a + 1]
        speeds = dict(zip(names, hermite_speed_bounds(pa, pb)))
        ta, tb = pa['time_from_start_s'], pb['time_from_start_s']
        dt = (tb - ta) / samples_per_segment
        prev = None
        for i in range(samples_per_segment + 1):
            t = ta + (tb - ta) * i / samples_per_segment
            q = dict(zip(names, hermite_q(pa, pb, t)))
            z = model.link_min_z(q, links)
            for n in links:
                if z[n] < sampled[n][0]:
                    sampled[n] = (z[n], t)
                zb = z[n] - model.lipschitz(n, box)
                if prev is not None:
                    lip = model.lipschitz(n, speeds)
                    bound[n] = min(bound[n], min(zb, prev[n]) - lip * dt / 2)
                else:
                    bound[n] = min(bound[n], zb)
            prev = {n: z[n] - model.lipschitz(n, box) for n in links}
    wl, wz = _worst(model, {n: v[0] for n, v in sampled.items()})
    bl, bz = _worst(model, bound)
    return {'samples_per_segment': samples_per_segment, 'box_rad': box_rad,
            'sampled_min_z_body_m': wz, 'sampled_limiting_link': wl,
            'sampled_limiting_time_s': sampled[wl][1],
            'bound_min_z_body_m': bz, 'bound_limiting_link': bl,
            'per_link_sampled_min_z_body_m': {n: v[0] for n, v in sampled.items()},
            'per_link_bound_min_z_body_m': bound}


def leg_groups(model):
    """{tuple of revolute movers: [links]} - each leg's links depend only on that leg's joints."""
    groups = {}
    for n in sorted(model.geometry):
        groups.setdefault(tuple(sorted(model.movers[n])), []).append(n)
    return groups


def branch_and_bound_min(model, links, lo, hi, eps_m=BNB_EPS_M, max_nodes=BNB_MAX_NODES):
    """Guaranteed minimum of the lowest collision z of `links` over the joint box [lo, hi].

    lo/hi: {joint: rad} for the joints that move these links. Each node's lower bound is
    z(centre) - sum_j half_width_j * R_j (Lipschitz, see the module docstring). Nodes are split
    along the dimension with the largest half_width * R contribution; the search stops when the
    smallest open lower bound is within eps_m of the best value found (or the node budget ends).
    Returns {'sampled': best z found (exact at a configuration), 'bound': proven lower bound,
             'configuration': argmin, 'nodes': evaluated nodes, 'proven_within_m': bound gap}.
    """
    import heapq
    joints = sorted(lo)

    def evaluate(blo, bhi):
        c = {j: (blo[j] + bhi[j]) / 2 for j in joints}
        half = {j: (bhi[j] - blo[j]) / 2 for j in joints}
        z = model.link_min_z(c, links)
        zc = min(z.values())
        lb = min(z[n] - model.lipschitz(n, half) for n in links)
        return zc, lb, c

    best, best_q, nodes, tie = math.inf, None, 0, 0
    zc, lb, c = evaluate(lo, hi)
    best, best_q, nodes = zc, c, 1
    heap = [(lb, tie, lo, hi)]
    while heap:
        lb, _, blo, bhi = heap[0]
        if lb >= best - eps_m or nodes >= max_nodes:
            break
        heapq.heappop(heap)
        # split along the joint with the largest Lipschitz contribution
        contrib = {j: (bhi[j] - blo[j]) / 2 * max(model.radius[n].get(j, 0.0) for n in links)
                   for j in joints}
        js = max(contrib, key=contrib.get)
        mid = (blo[js] + bhi[js]) / 2
        for a, b in ((blo[js], mid), (mid, bhi[js])):
            clo, chi = dict(blo), dict(bhi)
            clo[js], chi[js] = a, b
            zc, clb, c = evaluate(clo, chi)
            nodes += 1
            if zc < best:
                best, best_q = zc, c
            if clb < best - eps_m:
                tie += 1
                heapq.heappush(heap, (clb, tie, clo, chi))
    proven = min([best] + [h[0] for h in heap])
    return {'sampled': best, 'bound': proven, 'configuration': best_q, 'nodes': nodes,
            'proven_within_m': best - proven}


def joint_limit_envelope(model, overshoot_rad=LIMIT_OVERSHOOT_RAD, eps_m=BNB_EPS_M):
    """Passive worst case: every joint ANYWHERE within its URDF limits widened by overshoot_rad
    (e.g. controllers inactive, a fault hold, or limit-constraint softness).

    Every configuration the simulator can physically reach lies in this set, so a weld height
    that clears it cannot let any collision geometry touch the ground, whatever the controller
    does. Branch and bound per leg gives a guaranteed minimum (see branch_and_bound_min).
    """
    per_link, where, nodes, gap = {}, {}, 0, 0.0
    for movers, links in leg_groups(model).items():
        if not movers:
            z = model.link_min_z({}, links)
            for n in links:
                per_link[n] = (z[n], z[n])
                where[n] = {}
            continue
        lo = {j: model.limits[j][0] - overshoot_rad for j in movers}
        hi = {j: model.limits[j][1] + overshoot_rad for j in movers}
        r = branch_and_bound_min(model, links, lo, hi, eps_m)
        nodes += r['nodes']
        gap = max(gap, r['proven_within_m'])
        z = model.link_min_z(r['configuration'], links)
        lim = min(z, key=z.get)
        for n in links:
            per_link[n] = (z[n], r['bound'] if n == lim else z[n])
            where[n] = r['configuration']
    sampled = {n: v[0] for n, v in per_link.items()}
    bounds = {n: v[1] for n, v in per_link.items()}
    wl, wz = _worst(model, sampled)
    bl, bz = _worst(model, bounds)
    return {'method': 'branch and bound per leg (Lipschitz node bounds)',
            'overshoot_rad': overshoot_rad, 'eps_m': eps_m, 'nodes': nodes,
            'proven_within_m': gap,
            'sampled_min_z_body_m': wz, 'sampled_limiting_link': wl,
            'sampled_limiting_configuration': where[wl],
            'bound_min_z_body_m': bz, 'bound_limiting_link': bl}


def tracking_box_envelope(model, trajectory, box_rad=TRACKING_TOLERANCE_RAD, every=10,
                          eps_m=BNB_EPS_M):
    """Clearance with |q - q_ref(t)| <= box_rad around the commanded spline (tracking error the
    controller and client allow before aborting). Branch and bound per leg at every `every`-th
    sample of a 200-per-segment grid, plus the time-continuity term between evaluated samples."""
    names = list(trajectory['joint_names'])
    pts = trajectory['points']
    groups = leg_groups(model)
    worst = (math.inf, math.inf, None, None)               # (bound, sampled, link, time)
    for a in range(len(pts) - 1):
        pa, pb = pts[a], pts[a + 1]
        speeds = dict(zip(names, hermite_speed_bounds(pa, pb)))
        ta, tb = pa['time_from_start_s'], pb['time_from_start_s']
        n_eval = DEFAULT_SAMPLES_PER_SEGMENT // every
        dt = (tb - ta) / n_eval
        for i in range(n_eval + 1):
            t = ta + (tb - ta) * i / n_eval
            q = dict(zip(names, hermite_q(pa, pb, t)))
            for movers, links in groups.items():
                if not movers:
                    continue
                lo = {j: q.get(j, 0.0) - box_rad for j in movers}
                hi = {j: q.get(j, 0.0) + box_rad for j in movers}
                r = branch_and_bound_min(model, links, lo, hi, eps_m)
                lip = max(model.lipschitz(n, speeds) for n in links) * dt / 2
                b = r['bound'] - lip
                if b < worst[0]:
                    z = model.link_min_z(r['configuration'], links)
                    worst = (b, r['sampled'], min(z, key=z.get), t)
    return {'box_rad': box_rad, 'evaluated_per_segment': DEFAULT_SAMPLES_PER_SEGMENT // every,
            'bound_min_z_body_m': worst[0], 'sampled_min_z_body_m': worst[1],
            'limiting_link': worst[2], 'limiting_time_s': worst[3]}


def required_height(min_z_body, margin_m):
    """Smallest weld height h with h + min_z_body >= margin (ground at z = 0)."""
    return GROUND_Z_M + margin_m - min_z_body


def analyse(urdf_root, trajectory, neutral, samples_per_segment=DEFAULT_SAMPLES_PER_SEGMENT,
            tracking_rad=TRACKING_TOLERANCE_RAD, start_rad=START_POSE_TOLERANCE_RAD,
            overshoot_rad=LIMIT_OVERSHOOT_RAD, tracking_every=10):
    """The full clearance report (no mounting decision; see recommend()).

    Envelopes, from the narrowest to the widest:
      neutral                       the commanded start/end posture (exact);
      commanded                     the cubic-Hermite spline the JTC interpolates (sampled +
                                    continuous bound);
      lead_in_start_box             neutral +/- (start tolerance + tracking tolerance): the lead-in
                                    from any readiness-accepted start (monotone, zero end velocities);
      commanded_with_tracking_box   spline +/- tracking tolerance (before the abort);
      joint_limit_box               ANY configuration within the URDF limits + overshoot (passive,
                                    fault hold, controller loss): the physically reachable set.
    """
    model = CollisionModel(urdf_root)
    names = list(trajectory['joint_names'])
    n0 = dict(zip(names, neutral))
    nl, nz = _worst(model, model.link_min_z(n0))
    lead = {}
    for movers, links in leg_groups(model).items():
        if not movers:
            continue
        r = branch_and_bound_min(model, links, {j: n0.get(j, 0.0) - start_rad - tracking_rad
                                                for j in movers},
                                 {j: n0.get(j, 0.0) + start_rad + tracking_rad for j in movers})
        if not lead or r['bound'] < lead['bound_min_z_body_m']:
            z = model.link_min_z(r['configuration'], links)
            lead = {'box_rad': start_rad + tracking_rad, 'bound_min_z_body_m': r['bound'],
                    'sampled_min_z_body_m': r['sampled'], 'limiting_link': min(z, key=z.get),
                    'configuration': r['configuration']}
    rep = {
        'schema': SCHEMA,
        'frame': 'base_link (+x right, +y front, +z up); world z = h + z_body (zero roll/pitch)',
        'ground': 'spiderx_fortress.sdf ground_plane, z = 0, normal +z',
        'units': 'm, rad, s',
        'geometry': {n: g.sources for n, g in sorted(model.geometry.items())},
        'trajectory_id': trajectory.get('trajectory_id'),
        'content_sha256': (trajectory.get('provenance') or {}).get('content_sha256'),
        'neutral': {'min_z_body_m': nz, 'limiting_link': nl},
        'commanded': commanded_envelope(model, trajectory, samples_per_segment, 0.0),
        'lead_in_start_box': lead,
        'commanded_with_tracking_box': tracking_box_envelope(model, trajectory, tracking_rad,
                                                             tracking_every),
        'joint_limit_box': joint_limit_envelope(model, overshoot_rad),
        'tolerances_rad': {'tracking': tracking_rad, 'start_pose': start_rad,
                           'limit_overshoot': overshoot_rad},
    }
    rep['run_envelope_bound_min_z_body_m'] = min(
        rep['lead_in_start_box']['bound_min_z_body_m'],
        rep['commanded_with_tracking_box']['bound_min_z_body_m'])
    return rep


def recommend(report, margin_m, round_to_m=0.005):
    """Provisional weld height: the PASSIVE (joint-limit box) guaranteed bound plus the margin,
    rounded UP to round_to_m. The joint-limit box contains every reachable configuration, so this
    height keeps all collision geometry off the ground whatever the controller does."""
    need = required_height(report['joint_limit_box']['bound_min_z_body_m'], margin_m)
    h = round(math.ceil(need / round_to_m - 1e-9) * round_to_m, 6)
    return {'basis': 'joint_limit_box bound + margin, rounded up',
            'margin_m': margin_m, 'required_min_height_m': need, 'recommended_height_m': h,
            'clearance_at_recommended_m': {
                'neutral': round(h + report['neutral']['min_z_body_m'], 6),
                'commanded_sampled': round(h + report['commanded']['sampled_min_z_body_m'], 6),
                'commanded_bound': round(h + report['commanded']['bound_min_z_body_m'], 6),
                'run_envelope_bound': round(h + report['run_envelope_bound_min_z_body_m'], 6),
                'joint_limit_box_bound': round(
                    h + report['joint_limit_box']['bound_min_z_body_m'], 6)},
            'previous_candidate_0_075_m': {
                'neutral': round(0.075 + report['neutral']['min_z_body_m'], 6),
                'run_envelope_bound': round(0.075 + report['run_envelope_bound_min_z_body_m'], 6),
                'joint_limit_box_bound': round(
                    0.075 + report['joint_limit_box']['bound_min_z_body_m'], 6)}}


def load_default_inputs():
    """(urdf_root, trajectory, neutral) of the approved M6.1 cycle and the unchanged model."""
    from spiderx_controller import m61_trot_cycle as tc
    plan = tc.load_plan()
    if plan.errors:
        raise ClearanceError(f'M6.1 plan errors: {plan.errors}')
    traj = tc.build_trajectory(plan)
    from spiderx_controller.config_check import load_urdf
    return load_urdf(), traj, plan.neutral


__all__ = ['CollisionModel', 'Geometry', 'commanded_envelope',
           'joint_limit_envelope', 'tracking_box_envelope', 'branch_and_bound_min', 'leg_groups',
           'hermite_speed_bounds', 'hermite_q', 'analyse', 'recommend',
           'required_height', 'load_default_inputs', 'ClearanceError', 'SCHEMA',
           'TRACKING_TOLERANCE_RAD', 'START_POSE_TOLERANCE_RAD', 'check_config', 'main']


# ---------------------------------------------------------------- command line
EXIT_OK, EXIT_MISMATCH, EXIT_USAGE = 0, 1, 2
DEFAULT_OUT = 'log/m61a_clearance'


def _summary_lines(rep, rec):
    out = [f'trajectory {rep["trajectory_id"]} (content {str(rep["content_sha256"])[:16]})',
           'lowest collision point, metres below base_link (z_body):']
    rows = (('neutral (exact)', rep['neutral']['min_z_body_m'], rep['neutral']['limiting_link']),
            ('commanded spline, sampled', rep['commanded']['sampled_min_z_body_m'],
             rep['commanded']['sampled_limiting_link']),
            ('commanded spline, guaranteed bound', rep['commanded']['bound_min_z_body_m'],
             rep['commanded']['bound_limiting_link']),
            ('lead-in start box, bound', rep['lead_in_start_box']['bound_min_z_body_m'],
             rep['lead_in_start_box']['limiting_link']),
            ('spline +/- tracking tolerance, bound',
             rep['commanded_with_tracking_box']['bound_min_z_body_m'],
             rep['commanded_with_tracking_box']['limiting_link']),
            ('every reachable configuration, bound', rep['joint_limit_box']['bound_min_z_body_m'],
             rep['joint_limit_box']['bound_limiting_link']))
    for name, z, link in rows:
        out.append(f'  {name:<40} {-z:.5f}  ({link})')
    out.append(f'recommended (provisional) weld height: {rec["recommended_height_m"]:.3f} m '
               f'(bound + margin {rec["margin_m"]:.3f} m, rounded up)')
    for k, v in rec['clearance_at_recommended_m'].items():
        out.append(f'  clearance at that height, {k:<24} {v:.4f} m')
    return out


def check_config(cfg_clearance, mount_z, recomputed_bound, eps_m=BNB_EPS_M):
    """Problems (list of strings) between the stored config and a recomputed passive bound."""
    probs = []
    stored = cfg_clearance['passive_bound_min_z_body_m']
    if abs(stored - recomputed_bound) > eps_m + 1e-5:
        probs.append(f'stored passive bound {stored} differs from recomputed {recomputed_bound:.5f}')
    need = required_height(recomputed_bound, cfg_clearance['margin_m'])
    if cfg_clearance['min_mount_height_m'] + 1e-9 < need:
        probs.append(f'min_mount_height_m {cfg_clearance["min_mount_height_m"]} < required '
                     f'{need:.5f}')
    if mount_z + 1e-9 < cfg_clearance['min_mount_height_m']:
        probs.append(f'mount z {mount_z} < min_mount_height_m')
    return probs


def main(argv=None):
    import argparse
    import json
    import sys
    from datetime import datetime, timezone
    p = argparse.ArgumentParser(
        prog='m61a_clearance',
        description='M6.1-A offline ground clearance of the welded SpiderX (no simulator).')
    p.add_argument('--margin', type=float, default=0.015, help='clearance margin (m)')
    p.add_argument('--check-config', action='store_true',
                   help='only recompute the passive bound and compare it with '
                        'config/m61a_fixed_base.yaml (about 30 s)')
    p.add_argument('--out', default=DEFAULT_OUT, help=f'report root (default {DEFAULT_OUT})')
    p.add_argument('--no-write', action='store_true', help='print only; write no file')
    args = p.parse_args(sys.argv[1:] if argv is None else argv)
    if not (math.isfinite(args.margin) and args.margin >= 0):
        print('usage: --margin must be a finite number >= 0')
        return EXIT_USAGE
    print('SpiderX M6.1-A offline clearance analysis (welded base; no Gazebo, no goal)')
    from spiderx_controller import m61a_fixed_base as fb
    raw = fb.load_config().raw
    urdf, traj, neutral = load_default_inputs()
    if args.check_config:
        jl = joint_limit_envelope(CollisionModel(urdf))
        probs = check_config(raw['clearance'], raw['mount']['z_m'], jl['bound_min_z_body_m'])
        print(f'passive bound (recomputed): {-jl["bound_min_z_body_m"]:.5f} m below base_link')
        for pr in probs:
            print(f'MISMATCH: {pr}')
        print('Config consistent with the geometry.' if not probs else 'Config NOT consistent.')
        return EXIT_MISMATCH if probs else EXIT_OK
    rep = analyse(urdf, traj, neutral)
    rec = recommend(rep, args.margin)
    rep['recommendation'] = rec
    rep['config_mount_z_m'] = raw['mount']['z_m']
    rep['config_check'] = check_config(raw['clearance'], raw['mount']['z_m'],
                                       rep['joint_limit_box']['bound_min_z_body_m'])
    for line in _summary_lines(rep, rec):
        print(line)
    print('config check: ' + ('consistent' if not rep['config_check'] else
                              '; '.join(rep['config_check'])))
    if not args.no_write:
        d = os.path.join(args.out, datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
        os.makedirs(d, exist_ok=False)
        path = os.path.join(d, 'clearance.json')
        with open(path, 'x') as f:
            json.dump(rep, f, indent=1, sort_keys=True, default=str)
        print(f'report: {path}')
    return EXIT_MISMATCH if rep['config_check'] else EXIT_OK
