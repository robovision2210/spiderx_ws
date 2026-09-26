"""M3 single-leg forward and inverse kinematics for SpiderX (SIMULATION-ONLY validation scope).

Geometry source of truth: the EXPANDED URDF (xacro output). Every joint origin, rpy, axis and
limit, and the foot collision mesh, are read from it at load time; no link length or offset is
typed into this module. The chain joint names come from config/spiderx_legs.yaml, which
validate_controller_config already checks against the URDF.

Frames and units
----------------
* Reference frame for all inputs/outputs: ``base_link`` (SpiderX body frame, NOT REP-103:
  +x = robot right, +y = robot front, +z = up). Metres and radians, right-handed frames.
* Joint vector order: [hip, thigh, foot] of the leg (spiderx_legs.yaml joint_order), e.g.
  [lf_hip, lf_thigh_joint, lf_foot_joint]. Positive rotation = right-hand rule about the joint's
  URDF axis (the axis sign comes from the URDF, never assumed).
* URDF transform semantics: T_parent_child(q) = Trans(xyz) . Rz(yaw) Ry(pitch) Rx(roll) . Rot(axis, q).
* Foot tip: the URDF has NO foot-tip frame (the foot link origin lies on the knee axis). The tip is
  a DERIVED, non-physical reference point fixed in the foot link: the lowest point of the foot's
  own collision mesh at q = 0 (unique vertices within TIP_VERTEX_TOL_M of the minimum base_link z;
  tip = (mean x, mean y, min z)). No frame is added to the URDF or TF.

Forward kinematics returns the full pose of the URDF foot link plus the tip point. Inverse
kinematics is POSITION-ONLY (three joints cannot also set foot orientation) and ANALYTIC; its
geometric preconditions (hip axis perpendicular to thigh axis, thigh axis parallel to knee axis) are
checked from the extracted URDF and it refuses to run if they do not hold.

Nothing here imports ROS or commands the robot.
"""

import math
import os
import struct

FRAME = 'base_link'
SUPPORTED_LEGS = ('front_left',)
LEG_ALIASES = {'front_left': 'front_left', 'lf': 'front_left'}
IK_POSITION_TOL_M = 1e-6        # FK(IK(p)) must reproduce p within this (math residual)
TIP_VERTEX_TOL_M = 1e-4         # foot-tip definition: mesh vertices within 0.1 mm of min z
ACOS_EPS = 1e-12                # acos arguments beyond +-1 by more than this are unreachable
STRUCT_TOL = 1e-9               # axis orthogonality / parallelism tolerance
DEFAULT_MARGIN_RAD = 0.05       # existing M1 safety margin (spiderx_legs.yaml)


class KinematicsError(ValueError):
    """Invalid input, unsupported leg/frame or malformed geometry."""


# ---------------------------------------------------------------- small linear algebra (3D)
def _add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _scale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _norm(a):
    return math.sqrt(_dot(a, a))


def _unit(a):
    n = _norm(a)
    if n < 1e-12:
        raise KinematicsError('zero-length axis in URDF geometry')
    return _scale(a, 1.0 / n)


def _mv(m, v):
    return tuple(m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2] for i in range(3))


def _mm(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def _t(m):
    return tuple(tuple(m[j][i] for j in range(3)) for i in range(3))


IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def rot_axis_angle(axis, angle):
    """Rotation matrix for a right-handed rotation of `angle` about unit `axis` (Rodrigues)."""
    x, y, z = _unit(axis)
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return ((c + x * x * C, x * y * C - z * s, x * z * C + y * s),
            (y * x * C + z * s, c + y * y * C, y * z * C - x * s),
            (z * x * C - y * s, z * y * C + x * s, c + z * z * C))


def rot_rpy(roll, pitch, yaw):
    """URDF rpy: R = Rz(yaw) . Ry(pitch) . Rx(roll)."""
    return _mm(rot_axis_angle((0, 0, 1), yaw),
               _mm(rot_axis_angle((0, 1, 0), pitch), rot_axis_angle((1, 0, 0), roll)))


def quaternion_from_matrix(m):
    """(x, y, z, w) unit quaternion from a rotation matrix."""
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x = 0.25 * s, (m[2][1] - m[1][2]) / s
        y, z = (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        w, x = (m[2][1] - m[1][2]) / s, 0.25 * s
        y, z = (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        w, x = (m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s
        y, z = 0.25 * s, (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        w, x = (m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s
        y, z = (m[1][2] + m[2][1]) / s, 0.25 * s
    q = (x, y, z, w)
    n = math.sqrt(sum(c * c for c in q))
    q = tuple(c / n for c in q)
    return q if q[3] >= 0 else tuple(-c for c in q)


def matrix_from_quaternion(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise KinematicsError('zero-length quaternion')
    x, y, z, w = x / n, y / n, z / n, w / n
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def rotation_angle_between(a, b):
    """Angle (rad) of the relative rotation a^T b."""
    r = _mm(_t(a), b)
    c = (r[0][0] + r[1][1] + r[2][2] - 1.0) / 2.0
    s = 0.5 * math.sqrt((r[2][1] - r[1][2]) ** 2 + (r[0][2] - r[2][0]) ** 2 +
                        (r[1][0] - r[0][1]) ** 2)
    return math.atan2(s, c)          # accurate for small angles (acos(c) is not)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------- input validation
def _finite(value, what):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise KinematicsError(f'{what} must be a finite number, got {value!r}')
    return float(value)


def validate_joint_vector(q, joint_names):
    """Exactly len(joint_names) finite numbers, in the documented [hip, thigh, foot] order."""
    if isinstance(q, (str, bytes, dict)) or not hasattr(q, '__len__'):
        raise KinematicsError(f'joint vector must be a sequence of {len(joint_names)} numbers '
                              f'ordered {list(joint_names)}, got {q!r}')
    if len(q) != len(joint_names):
        raise KinematicsError(f'joint vector must have exactly {len(joint_names)} values '
                              f'ordered {list(joint_names)}, got {len(q)}')
    return tuple(_finite(v, f'joint {n}') for v, n in zip(q, joint_names))


def validate_point(p, what='target'):
    if isinstance(p, (str, bytes, dict)) or not hasattr(p, '__len__') or len(p) != 3:
        raise KinematicsError(f'{what} must be [x, y, z] in metres, got {p!r}')
    return tuple(_finite(v, f'{what} {c}') for v, c in zip(p, 'xyz'))


def resolve_leg(leg):
    if leg not in LEG_ALIASES:
        raise KinematicsError(f'unknown or unsupported leg "{leg}": M3 validates only '
                              f'{list(SUPPORTED_LEGS)} (aliases {sorted(LEG_ALIASES)})')
    return LEG_ALIASES[leg]


def check_frame(frame):
    if frame != FRAME:
        raise KinematicsError(f'frame must be "{FRAME}" (SpiderX body frame), got {frame!r}')


# ---------------------------------------------------------------- URDF and mesh extraction
def _vec(s, default=(0.0, 0.0, 0.0)):
    return tuple(float(v) for v in s.split()) if s else default


def parse_urdf_joints(urdf_root):
    """{joint name: {type, parent, child, xyz, rpy, axis, lower, upper}} from a URDF root."""
    joints = {}
    for j in urdf_root.findall('joint'):
        o, a, lim = j.find('origin'), j.find('axis'), j.find('limit')
        joints[j.get('name')] = {
            'name': j.get('name'),
            'type': j.get('type'),
            'parent': j.find('parent').get('link'),
            'child': j.find('child').get('link'),
            'xyz': _vec(o.get('xyz')) if o is not None else (0.0, 0.0, 0.0),
            'rpy': _vec(o.get('rpy')) if o is not None else (0.0, 0.0, 0.0),
            'axis': _vec(a.get('xyz')) if a is not None else (1.0, 0.0, 0.0),
            'lower': float(lim.get('lower')) if lim is not None and lim.get('lower') else None,
            'upper': float(lim.get('upper')) if lim is not None and lim.get('upper') else None,
        }
    return joints


def chain_between(joints, base_link, tip_link):
    """Joints (dicts) from base_link down to tip_link, in parent->child order."""
    by_child = {j['child']: j for j in joints.values()}
    chain, link = [], tip_link
    while link != base_link:
        if link not in by_child:
            raise KinematicsError(f'no URDF chain from {base_link} to {tip_link}')
        j = by_child[link]
        chain.append(j)
        link = j['parent']
    return list(reversed(chain))


def resolve_mesh_path(filename):
    if filename.startswith('file://'):
        return filename[len('file://'):]
    if filename.startswith('package://'):
        pkg, rel = filename[len('package://'):].split('/', 1)
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory(pkg), rel)
    return filename


def load_stl_vertices(path):
    """All triangle vertices (x, y, z) of a binary or ASCII STL (file units)."""
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) >= 84:
        n = struct.unpack('<I', data[80:84])[0]
        if 84 + 50 * n == len(data):
            out = []
            for i in range(n):
                vals = struct.unpack('<12f', data[84 + 50 * i: 84 + 50 * i + 48])
                out.extend([vals[3:6], vals[6:9], vals[9:12]])
            return out
    text = data.decode('ascii', errors='strict')
    out = []
    for line in text.splitlines():
        parts = line.split()
        if parts and parts[0] == 'vertex':
            out.append(tuple(float(v) for v in parts[1:4]))
    if not out:
        raise KinematicsError(f'no vertices in STL {path}')
    return out


class _Pose:
    """Rigid transform (R, p)."""

    __slots__ = ('R', 'p')

    def __init__(self, R=IDENTITY, p=(0.0, 0.0, 0.0)):
        self.R, self.p = R, p

    def __mul__(self, other):
        return _Pose(_mm(self.R, other.R), _add(_mv(self.R, other.p), self.p))

    def apply(self, v):
        return _add(_mv(self.R, v), self.p)


def _joint_transform(j, q=0.0):
    fixed = _Pose(rot_rpy(*j['rpy']), j['xyz'])
    if j['type'] in ('revolute', 'continuous'):
        return fixed * _Pose(rot_axis_angle(j['axis'], q))
    if j['type'] == 'fixed':
        return fixed
    raise KinematicsError(f'unsupported joint type {j["type"]} for {j["name"]}')


class LegGeometry:
    """URDF-derived geometry of ONE SpiderX leg (base_link -> foot link + derived foot tip)."""

    def __init__(self, leg, chain, joint_names, foot_link, tip_local, mesh_path, tip_vertices):
        self.leg = leg
        self.frame = FRAME
        self.chain = chain                      # all joints base_link -> foot link (dicts)
        self.joint_names = tuple(joint_names)   # [hip, thigh, foot]
        self.foot_link = foot_link
        self.tip_local = tip_local              # derived tip in the foot-link frame
        self.mesh_path = mesh_path
        self.tip_vertex_count = tip_vertices
        revolute = [j for j in chain if j['type'] == 'revolute']
        if [j['name'] for j in revolute] != list(joint_names):
            raise KinematicsError(f'URDF revolute joints {[j["name"] for j in revolute]} between '
                                  f'{FRAME} and {foot_link} differ from {list(joint_names)}')
        self.limits = tuple((j['lower'], j['upper']) for j in revolute)
        # zero-configuration (q = 0) axis points and axes in base_link, for the PoE / IK
        pose = _Pose()
        self.axis_points, self.axes = [], []
        for j in chain:
            frame = pose * _Pose(rot_rpy(*j['rpy']), j['xyz'])
            if j['type'] == 'revolute':
                self.axis_points.append(frame.p)
                self.axes.append(_unit(_mv(frame.R, j['axis'])))
            pose = frame
        self.foot_pose0 = pose
        self.tip0 = pose.apply(tip_local)
        self._check_structure()

    @classmethod
    def from_urdf(cls, urdf_root, legs_cfg, leg='front_left', tip_vertex_tol=TIP_VERTEX_TOL_M):
        leg = resolve_leg(leg)
        if leg not in legs_cfg.get('legs', {}):
            raise KinematicsError(f'leg {leg} not in spiderx_legs.yaml')
        lc = legs_cfg['legs'][leg]
        names = [lc['joints'][role]['name'] for role in legs_cfg['joint_order']]
        foot_link = lc['foot_link']
        joints = parse_urdf_joints(urdf_root)
        chain = chain_between(joints, FRAME, foot_link)
        # foot collision mesh -> derived tip
        link = [lk for lk in urdf_root.findall('link') if lk.get('name') == foot_link]
        if not link:
            raise KinematicsError(f'foot link {foot_link} not in URDF')
        verts, mesh_path = [], None
        for col in link[0].findall('collision'):
            mesh = col.find('geometry/mesh')
            if mesh is None:
                continue
            mesh_path = resolve_mesh_path(mesh.get('filename'))
            scale = _vec(mesh.get('scale'), (1.0, 1.0, 1.0))
            o = col.find('origin')
            cpose = _Pose(rot_rpy(*_vec(o.get('rpy') if o is not None else None)),
                          _vec(o.get('xyz') if o is not None else None))
            for v in load_stl_vertices(mesh_path):
                verts.append(cpose.apply((v[0] * scale[0], v[1] * scale[1], v[2] * scale[2])))
        if not verts:
            raise KinematicsError(f'{foot_link} has no collision mesh to derive the foot tip from')
        pose0 = _Pose()
        for j in chain:
            pose0 = pose0 * _joint_transform(j, 0.0)
        in_base = [pose0.apply(v) for v in verts]
        zmin = min(v[2] for v in in_base)
        # unique vertices (STL repeats shared triangle corners; multiplicity must not bias the tip)
        low = sorted({tuple(round(c, 9) for c in v) for v in in_base
                      if v[2] <= zmin + tip_vertex_tol})
        tip_base = (sum(v[0] for v in low) / len(low), sum(v[1] for v in low) / len(low), zmin)
        tip_local = _mv(_t(pose0.R), _sub(tip_base, pose0.p))
        return cls(leg, chain, names, foot_link, tip_local, mesh_path, len(low))

    def _check_structure(self):
        a1, a2, a3 = self.axes
        self.analytic_ok = True
        self.structure = {
            'hip_dot_thigh_axis': _dot(a1, a2),
            'thigh_cross_knee_axis_norm': _norm(_cross(a2, a3)),
            'knee_sign_vs_thigh': _dot(a2, a3),
        }
        problems = []
        if abs(self.structure['hip_dot_thigh_axis']) > STRUCT_TOL:
            problems.append('hip axis is not perpendicular to the thigh axis')
        if self.structure['thigh_cross_knee_axis_norm'] > STRUCT_TOL:
            problems.append('thigh axis is not parallel to the knee axis')
        H, T, K = self.axis_points
        if _norm(_sub(K, T)) < 1e-6 or _norm(_sub(self.tip0, K)) < 1e-6:
            problems.append('degenerate link length')
        self.structure_problems = problems

    def summary(self):
        H, T, K = self.axis_points
        return {
            'leg': self.leg, 'frame': self.frame, 'joint_names': list(self.joint_names),
            'chain': [{'name': j['name'], 'type': j['type'], 'parent': j['parent'],
                       'child': j['child'], 'xyz': list(j['xyz']), 'rpy': list(j['rpy']),
                       'axis': list(j['axis']) if j['type'] == 'revolute' else None}
                      for j in self.chain],
            'limits_rad': [list(lim) for lim in self.limits],
            'axes_base_q0': [list(a) for a in self.axes],
            'hip_axis_point_m': list(H), 'thigh_axis_point_m': list(T),
            'knee_axis_point_m': list(K),
            'foot_link': self.foot_link, 'foot_link_origin_q0_m': list(self.foot_pose0.p),
            'tip_local_m': list(self.tip_local), 'tip_q0_m': list(self.tip0),
            'tip_definition': (f'derived: lowest point of the {self.foot_link} collision mesh at '
                               f'q = 0 (mean x, y of the {self.tip_vertex_count} unique vertices within '
                               f'{TIP_VERTEX_TOL_M} m of min z; z = min z); not a URDF/TF frame'),
            'mesh': os.path.basename(self.mesh_path) if self.mesh_path else None,
            'structure': self.structure, 'analytic_ik_preconditions_ok': not self.structure_problems,
        }


# ---------------------------------------------------------------- forward kinematics
def forward(geom, q):
    """FK by the URDF transform chain.

    q: [hip, thigh, foot] (rad). Returns dict (frame base_link):
      foot_link_position (m), foot_link_rotation (3x3), foot_link_quaternion (x, y, z, w),
      tip_position (m, derived foot tip).
    """
    q = validate_joint_vector(q, geom.joint_names)
    pose, i = _Pose(), 0
    for j in geom.chain:
        if j['type'] == 'revolute':
            pose = pose * _joint_transform(j, q[i])
            i += 1
        else:
            pose = pose * _joint_transform(j)
    return {
        'frame': FRAME,
        'joint_names': list(geom.joint_names),
        'q': list(q),
        'foot_link': geom.foot_link,
        'foot_link_position': pose.p,
        'foot_link_rotation': pose.R,
        'foot_link_quaternion': quaternion_from_matrix(pose.R),
        'tip_position': pose.apply(geom.tip_local),
    }


def forward_tip_poe(geom, q):
    """Tip position by the product of exponentials (zero-configuration axes and points)."""
    q = validate_joint_vector(q, geom.joint_names)
    H, T, K = geom.axis_points
    a1, a2, a3 = geom.axes
    inner = _add(_sub(K, T), _mv(rot_axis_angle(a3, q[2]), _sub(geom.tip0, K)))
    mid = _add(_sub(T, H), _mv(rot_axis_angle(a2, q[1]), inner))
    return _add(H, _mv(rot_axis_angle(a1, q[0]), mid))


# ---------------------------------------------------------------- inverse kinematics
def check_limits(geom, q, margin=DEFAULT_MARGIN_RAD):
    """List of violations of URDF limits pulled in by `margin` (empty = joint-safe)."""
    out = []
    for name, v, (lo, hi) in zip(geom.joint_names, q, geom.limits):
        if not lo + margin <= v <= hi - margin:
            out.append(f'{name}={v:.4f} outside [{lo + margin:.4f}, {hi - margin:.4f}] '
                       f'(URDF [{lo:.4f}, {hi:.4f}] -/+ {margin})')
    return out


def _acos_checked(c):
    if c > 1.0 + ACOS_EPS or c < -1.0 - ACOS_EPS:
        return None
    return math.acos(max(-1.0, min(1.0, c)))


def inverse(geom, target, frame=FRAME, reference=None, margin=DEFAULT_MARGIN_RAD):
    """Analytic position IK for the foot tip.

    target: [x, y, z] (m) in `frame` (must be base_link). reference: joint vector used to pick the
    solution (default CAD neutral = zeros). Returns a dict:
      ok (bool), reason ('ok' | 'invalid_input' | 'analytic_preconditions' | 'singular' |
      'unreachable_hip' | 'unreachable_knee' | 'joint_limits' | 'numerical'), message,
      solution (list or None), candidates (every analytic candidate with fk_residual_m,
      limit violations, singularity margins, distance to reference), selection_rule.
    Never clamps: a candidate outside the limits (minus margin) is rejected, not adjusted.
    """
    result = {'ok': False, 'reason': 'invalid_input', 'solution': None, 'candidates': [],
              'frame': FRAME, 'joint_names': list(geom.joint_names), 'margin_rad': margin,
              'selection_rule': ('nearest (Euclidean joint-space) valid candidate to the '
                                 'reference; reference = CAD neutral (0, 0, 0) unless given; '
                                 'ties -> lowest candidate index')}
    try:
        check_frame(frame)
        p = validate_point(target)
        ref = (validate_joint_vector(reference, geom.joint_names) if reference is not None
               else (0.0, 0.0, 0.0))
        margin = _finite(margin, 'margin')
        if margin < 0:
            raise KinematicsError('margin must be >= 0')
    except KinematicsError as e:
        result['message'] = str(e)
        return result
    result['target'] = list(p)
    if geom.structure_problems:
        result.update(reason='analytic_preconditions',
                      message='; '.join(geom.structure_problems))
        return result

    H, T, K = geom.axis_points
    u, w, a3 = geom.axes
    v = _cross(u, w)
    sigma = 1.0 if _dot(w, a3) > 0 else -1.0
    r = _sub(p, H)
    d = _dot(_sub(geom.tip0, H), w)
    rw, rv = _dot(r, w), _dot(r, v)
    rho = math.hypot(rw, rv)
    if rho < 1e-12:
        result.update(reason='singular', message='target lies on the hip axis (singular)')
        return result
    alpha = _acos_checked(d / rho)
    if alpha is None:
        result.update(reason='unreachable_hip',
                      message=f'unreachable: target is {rho:.4f} m from the hip axis in the hip '
                              f'plane but the leg needs |d| = {abs(d):.4f} m')
        return result
    phi = math.atan2(rv, rw)
    hip_margin = min(alpha, math.pi - alpha)
    hips = [phi + alpha] if alpha < 1e-12 else [phi + alpha, phi - alpha]

    L1, L2 = _sub(K, T), _sub(geom.tip0, K)

    def perp(x):
        return _sub(x, _scale(w, _dot(x, w)))
    L1p, L2p = perp(L1), perp(L2)
    l1, l2 = _norm(L1p), _norm(L2p)
    b1 = _unit(L1p)
    b2 = _cross(w, b1)

    def to2(x):
        return (_dot(x, b1), _dot(x, b2))
    A, B = to2(L1p), to2(L2p)
    betaA, betaB = math.atan2(A[1], A[0]), math.atan2(B[1], B[0])

    knee_unreachable = []
    for q1 in hips:
        e = _sub(_mv(rot_axis_angle(u, -q1), r), _sub(T, H))
        E = to2(perp(e))
        cosk = (E[0] ** 2 + E[1] ** 2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
        psi = _acos_checked(cosk)
        if psi is None:
            knee_unreachable.append(math.hypot(*E))
            continue
        knee_margin = min(psi, math.pi - psi)
        for th3 in ([psi - (betaB - betaA)] if psi < 1e-12
                    else [psi - (betaB - betaA), -psi - (betaB - betaA)]):
            c3, s3 = math.cos(th3), math.sin(th3)
            S = (A[0] + c3 * B[0] - s3 * B[1], A[1] + s3 * B[0] + c3 * B[1])
            q2 = math.atan2(E[1], E[0]) - math.atan2(S[1], S[0])
            cand = tuple(_wrap(x) for x in (q1, q2, sigma * th3))
            residual = _norm(_sub(forward(geom, cand)['tip_position'], p))
            viol = check_limits(geom, cand, margin)
            result['candidates'].append({
                'q': list(cand), 'fk_residual_m': residual, 'limit_violations': viol,
                'hip_singularity_margin_rad': hip_margin,
                'knee_singularity_margin_rad': knee_margin,
                'distance_to_reference_rad': math.sqrt(sum((a - b) ** 2
                                                           for a, b in zip(cand, ref))),
                'valid': residual <= IK_POSITION_TOL_M and not viol,
            })
    if not result['candidates']:
        result.update(reason='unreachable_knee',
                      message=f'unreachable: knee-plane distance '
                              f'{min(knee_unreachable):.4f} m outside '
                              f'[{abs(l1 - l2):.4f}, {l1 + l2:.4f}] m')
        return result
    valid = [i for i, c in enumerate(result['candidates']) if c['valid']]
    if not valid:
        if all(c['fk_residual_m'] <= IK_POSITION_TOL_M for c in result['candidates']):
            result.update(reason='joint_limits',
                          message='geometrically reachable, but every solution violates the '
                                  f'URDF limits minus {margin} rad')
        else:
            result.update(reason='numerical', message='no candidate reproduced the target '
                                                      f'within {IK_POSITION_TOL_M} m')
        return result
    best = min(valid, key=lambda i: (result['candidates'][i]['distance_to_reference_rad'], i))
    result.update(ok=True, reason='ok', message='solution found', selected_index=best,
                  solution=result['candidates'][best]['q'])
    return result


def load_geometry(leg='front_left', urdf_root=None, legs_cfg=None, config_dir=None):
    """Convenience loader: expanded installed URDF + installed spiderx_legs.yaml."""
    if urdf_root is None:
        from spiderx_controller.config_check import load_urdf
        urdf_root = load_urdf()
    if legs_cfg is None:
        import yaml
        if config_dir is None:
            from ament_index_python.packages import get_package_share_directory
            config_dir = os.path.join(get_package_share_directory('spiderx_controller'), 'config')
        with open(os.path.join(config_dir, 'spiderx_legs.yaml')) as f:
            legs_cfg = yaml.safe_load(f)
    return LegGeometry.from_urdf(urdf_root, legs_cfg, leg)


__all__ = ['FRAME', 'KinematicsError', 'LegGeometry', 'forward', 'forward_tip_poe', 'inverse',
           'check_limits', 'load_geometry', 'validate_joint_vector', 'validate_point',
           'quaternion_from_matrix', 'matrix_from_quaternion', 'rotation_angle_between',
           'IK_POSITION_TOL_M', 'TIP_VERTEX_TOL_M']
