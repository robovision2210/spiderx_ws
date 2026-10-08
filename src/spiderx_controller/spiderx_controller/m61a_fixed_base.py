"""M6.1-A fixed-base model: frames, weld identity, body-pose observability and attachment checks.

Pure Python (no ROS graph, no numpy). Everything that touches a live graph lives in
m61a_observer.py and m61a_live.py; this module only computes from the values they hand in.

Arrangement (Approach B, docs/M61A_FIXED_BASE_IMPLEMENTATION.md)
-------------------------------------------------------------------
The fixed-base wrapper spiderx_fixed_base.urdf.xacro includes the UNCHANGED spiderx.urdf.xacro
and adds a link named ``world`` plus ONE fixed joint ``spiderx_fixed_base_weld`` world ->
dummy_link whose origin is the mounting transform (x, y, z, yaw; roll = pitch = 0). The model is
spawned with the IDENTITY pose. sdformat 12 converts that joint into a fixed joint with
<parent>world</parent> whose pose is the origin, relative to __model__; dummy_link sits at the
joint and base_link is a frame of dummy_link (merged by fixed-joint reduction). So

    T_model_dummy = T_weld_origin                       (converter rule, checked by tests)
    T_model_body  = T_model_dummy * T_dummy_body        (T_dummy_body = dummy_joint origin)
    T_world_body  = T_world_model * T_model_body        (full rigid composition, never a z add)

and with a correct identity spawn and an intact weld, T_world_model = I and
T_world_body = T_weld_origin * T_dummy_body.

Composition order: T_a_c = T_a_b * T_b_c ("pose of c in a"). Rotations are unit quaternions
(x, y, z, w); Euler angles are ZYX (roll about x, pitch about y, yaw about z), as in
posture_metrics.quat_to_rpy. Units: m, rad, s.

Observation source (verified offline; runtime REPORTED from M2/M3)
------------------------------------------------------------------
/spiderx/sim/world_poses (tf2_msgs/TFMessage) bridged from Gazebo /world/spiderx_fortress/pose/info
(SceneBroadcaster). The entry whose child_frame_id is the model name is the MODEL pose in the
Gazebo world; an entry whose child_frame_id is a link name is that LINK's pose RELATIVE TO THE
MODEL (M3 compared lf_foot_1 that way to 5.4e-11 m). Per-pose stamps are 0 at the bridge, so
samples are timed by receipt (monotonic wall) and by the receiving node's /clock.

The physical body is measured as T_world_model * T_model_dummy(Gazebo entry) * T_dummy_body:
the Gazebo-side dummy_link entry (the canonical link carrying the merged base_link) is used, not
the robot_description, so a body that moves relative to the model root is still seen whatever
convention Gazebo uses to update the model pose. The robot_description is only cross-checked
against that entry (identity association) and never trusted alone.
"""

from dataclasses import dataclass, field
import math
import os
import xml.etree.ElementTree as ET

CONFIG_FILE = 'm61a_fixed_base.yaml'
CONFIG_SCHEMA = 'spiderx.m61a.fixed_base/1'
WORLD_LINK = 'world'
QUAT_NORM_TOL = 1e-3          # |norm - 1| above this: the quaternion is refused, not normalized
ROLL_PITCH_TOL_RAD = 1e-12    # the wrapper only allows zero roll and pitch in the weld

# ---------------------------------------------------------------- observation codes
# Each is a distinct, named problem. Readiness lists every failing code; the in-flight monitor
# trips on the first one that is a cancel reason.
POSE_MISSING_MODEL = 'pose_missing_model'
POSE_MISSING_BODY_LINK = 'pose_missing_body_link'
POSE_AMBIGUOUS_MODEL = 'pose_ambiguous_model'
POSE_AMBIGUOUS_BODY_LINK = 'pose_ambiguous_body_link'
POSE_NONFINITE = 'pose_nonfinite'
POSE_BAD_QUATERNION = 'pose_bad_quaternion'
POSE_STALE = 'body_pose_stale'
POSE_NEVER_RECEIVED = 'body_pose_missing'
FRAME_LINK_INCONSISTENT = 'frame_body_link_inconsistent'   # Gazebo link entry != description
FRAME_SPAWN_NOT_IDENTITY = 'frame_spawn_not_identity'       # model root not at identity
ATTACHMENT_DISPLACED = 'attachment_displaced'
DESCRIPTION_MISSING = 'robot_description_missing'
DESCRIPTION_NOT_FIXED_BASE = 'robot_description_not_fixed_base'
DESCRIPTION_INVALID = 'robot_description_invalid'
MOUNT_MISMATCH = 'mount_not_approved'
MOUNT_TOO_LOW = 'mount_below_clearance_minimum'
JOINT_STATES_STALE = 'joint_states_stale'
JOINT_STATES_MISSING = 'joint_states_missing'
JOINT_STATES_INCOMPLETE = 'joint_states_incomplete'
CLOCK_MISSING = 'sim_clock_missing'
CLOCK_STALLED = 'sim_time_stalled'
CLOCK_RESET = 'sim_time_reset'
CONTROLLERS_NOT_ACTIVE = 'controllers_not_active'
CONTROLLERS_UNKNOWN = 'controllers_unknown'
JOINT_STATE_PUBLISHERS = 'joint_state_publisher_count'
COMMAND_PUBLISHERS = 'competing_command_publishers'


class FixedBaseError(ValueError):
    """Malformed configuration or robot description."""

    def __init__(self, code, message):
        super().__init__(f'{code}: {message}')
        self.code = code


# ---------------------------------------------------------------- rigid transforms
def _finite(*vals):
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
               for v in vals)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quat_rotate(q, v):
    x, y, z, w = q
    # v' = q v q*, expanded (unit q)
    tx = 2 * (y * v[2] - z * v[1])
    ty = 2 * (z * v[0] - x * v[2])
    tz = 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + (y * tz - z * ty),
            v[1] + w * ty + (z * tx - x * tz),
            v[2] + w * tz + (x * ty - y * tx))


def quat_from_rpy(roll, pitch, yaw):
    """R = Rz(yaw) Ry(pitch) Rx(roll) (URDF/SDF rpy) as (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def quat_to_rpy(q):
    """ZYX Euler angles of a unit quaternion (same convention as posture_metrics.quat_to_rpy)."""
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


@dataclass(frozen=True)
class Pose:
    """Rigid transform T_a_b: translation p (m) and unit quaternion q (x, y, z, w)."""
    p: tuple = (0.0, 0.0, 0.0)
    q: tuple = (0.0, 0.0, 0.0, 1.0)

    @classmethod
    def from_xyz_rpy(cls, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        return cls((float(x), float(y), float(z)), quat_from_rpy(roll, pitch, yaw))

    def __mul__(self, other):
        """T_a_c = T_a_b * T_b_c."""
        r = quat_rotate(self.q, other.p)
        return Pose((self.p[0] + r[0], self.p[1] + r[1], self.p[2] + r[2]),
                    _normalize(quat_mul(self.q, other.q)))

    def inverse(self):
        qi = (-self.q[0], -self.q[1], -self.q[2], self.q[3])
        r = quat_rotate(qi, self.p)
        return Pose((-r[0], -r[1], -r[2]), qi)

    def apply(self, v):
        r = quat_rotate(self.q, v)
        return (r[0] + self.p[0], r[1] + self.p[1], r[2] + self.p[2])

    def rpy(self):
        return quat_to_rpy(self.q)

    def xyz_rpy(self):
        return tuple(self.p) + tuple(self.rpy())

    def tilt(self):
        """Angle between this frame's z axis and the parent z axis (rad)."""
        x, y = self.q[0], self.q[1]
        zz = 1.0 - 2.0 * (x * x + y * y)
        return math.acos(max(-1.0, min(1.0, zz)))


def _normalize(q):
    n = math.sqrt(sum(c * c for c in q))
    return tuple(c / n for c in q)


def rotation_angle(qa, qb):
    """Smallest rotation angle (rad) between two unit quaternions."""
    d = abs(sum(a * b for a, b in zip(qa, qb)))
    return 2.0 * math.acos(max(-1.0, min(1.0, d)))


def deviation(a, b):
    """(translation distance m, rotation angle rad, dz m, dxy m) of pose a relative to b."""
    dx, dy, dz = (a.p[0] - b.p[0], a.p[1] - b.p[1], a.p[2] - b.p[2])
    return (math.sqrt(dx * dx + dy * dy + dz * dz), rotation_angle(a.q, b.q), dz,
            math.hypot(dx, dy))


# ---------------------------------------------------------------- configuration
@dataclass(frozen=True)
class FixedBaseConfig:
    model_name: str
    world_name: str
    body_link: str
    body_frame: str
    weld_joint: str
    pose_topic: str
    mount: Pose                                  # approved T_world_body (provisional)
    min_mount_height_m: float                    # from the clearance analysis
    attachment_translation_tol_m: float
    attachment_rotation_tol_rad: float
    attachment_debounce_samples: int
    spawn_translation_tol_m: float
    spawn_rotation_tol_rad: float
    link_translation_tol_m: float
    link_rotation_tol_rad: float
    pose_stale_s: float
    joint_states_stale_s: float
    sim_stall_s: float
    clock_reset_tol_s: float
    controllers: tuple
    joint_state_topic: str
    command_topic: str
    raw: dict = field(default_factory=dict, compare=False)


_TOP_KEYS = ('schema', 'milestone', 'simulation_only', 'status', 'model', 'mount', 'clearance',
             'attachment', 'spawn_identity', 'link_consistency', 'freshness', 'graph',
             'observation')


def _num(d, k, where, positive=True):
    v = d.get(k)
    if not _finite(v):
        raise FixedBaseError('config_invalid', f'{where}.{k} must be a finite number, got {v!r}')
    if positive and not v > 0:
        raise FixedBaseError('config_invalid', f'{where}.{k} must be > 0, got {v!r}')
    return float(v)


def parse_config(data):
    """Validate the parsed YAML dict; unknown or missing keys are refused."""
    if not isinstance(data, dict):
        raise FixedBaseError('config_invalid', 'top level must be a mapping')
    extra = sorted(set(data) - set(_TOP_KEYS))
    missing = sorted(set(_TOP_KEYS) - set(data))
    if extra or missing:
        raise FixedBaseError('config_invalid', f'unknown keys {extra}, missing keys {missing}')
    if data['schema'] != CONFIG_SCHEMA:
        raise FixedBaseError('config_invalid', f'schema must be {CONFIG_SCHEMA}')
    if data['simulation_only'] is not True:
        raise FixedBaseError('config_invalid', 'simulation_only must be true')
    m, mt = data['model'], data['mount']
    for k in ('x_m', 'y_m', 'z_m', 'yaw_rad'):
        _num(mt, k, 'mount', positive=False)
    if set(mt) != {'x_m', 'y_m', 'z_m', 'yaw_rad'}:
        raise FixedBaseError('config_invalid', 'mount takes exactly x_m, y_m, z_m, yaw_rad '
                                               '(roll and pitch are fixed at 0)')
    cl, at, sp, lc = (data['clearance'], data['attachment'], data['spawn_identity'],
                      data['link_consistency'])
    fr, gr = data['freshness'], data['graph']
    deb = at.get('debounce_samples')
    if not (isinstance(deb, int) and not isinstance(deb, bool) and deb >= 1):
        raise FixedBaseError('config_invalid', 'attachment.debounce_samples must be an int >= 1')
    cfg = FixedBaseConfig(
        model_name=str(m['model_name']), world_name=str(m['world_name']),
        body_link=str(m['body_link']), body_frame=str(m['body_frame']),
        weld_joint=str(m['weld_joint']), pose_topic=str(m['pose_topic']),
        mount=Pose.from_xyz_rpy(mt['x_m'], mt['y_m'], mt['z_m'], 0.0, 0.0, mt['yaw_rad']),
        min_mount_height_m=_num(cl, 'min_mount_height_m', 'clearance'),
        attachment_translation_tol_m=_num(at, 'translation_tolerance_m', 'attachment'),
        attachment_rotation_tol_rad=_num(at, 'rotation_tolerance_rad', 'attachment'),
        attachment_debounce_samples=deb,
        spawn_translation_tol_m=_num(sp, 'translation_tolerance_m', 'spawn_identity'),
        spawn_rotation_tol_rad=_num(sp, 'rotation_tolerance_rad', 'spawn_identity'),
        link_translation_tol_m=_num(lc, 'translation_tolerance_m', 'link_consistency'),
        link_rotation_tol_rad=_num(lc, 'rotation_tolerance_rad', 'link_consistency'),
        pose_stale_s=_num(fr, 'pose_stale_s', 'freshness'),
        joint_states_stale_s=_num(fr, 'joint_states_stale_s', 'freshness'),
        sim_stall_s=_num(fr, 'sim_stall_s', 'freshness'),
        clock_reset_tol_s=_num(fr, 'clock_reset_tolerance_s', 'freshness'),
        controllers=tuple(gr['controllers']),
        joint_state_topic=str(gr['joint_state_topic']),
        command_topic=str(gr['command_topic']),
        raw=data)
    if cfg.mount.p[2] + 1e-12 < cfg.min_mount_height_m:
        raise FixedBaseError('config_invalid', f'mount.z_m {cfg.mount.p[2]} is below '
                                               f'clearance.min_mount_height_m '
                                               f'{cfg.min_mount_height_m}')
    return cfg


def config_path(config_dir=None):
    if config_dir is None:
        from ament_index_python.packages import get_package_share_directory
        config_dir = os.path.join(get_package_share_directory('spiderx_controller'), 'config')
    return os.path.join(config_dir, CONFIG_FILE)


def load_config(config_dir=None):
    import yaml
    with open(config_path(config_dir)) as f:
        return parse_config(yaml.safe_load(f))


# ---------------------------------------------------------------- the robot description
@dataclass(frozen=True)
class WeldInfo:
    """What the robot description says about the fixed base."""
    fixed_base: bool
    weld_joint: str = None
    weld_origin: Pose = None                     # T_world_dummy (URDF origin)
    dummy_to_body: Pose = None                   # T_dummy_body (dummy_joint origin)
    model_to_dummy: Pose = None                  # T_model_dummy (converter rule)
    model_to_body: Pose = None                   # T_model_body

    def expected_world_body(self):
        """T_world_body with an identity spawn (T_world_model = I)."""
        return self.model_to_body


def _origin_pose(joint):
    o = joint.find('origin')
    xyz = tuple(float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split())
    rpy = tuple(float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split())
    if len(xyz) != 3 or len(rpy) != 3 or not _finite(*xyz, *rpy):
        raise FixedBaseError(DESCRIPTION_INVALID, f'joint {joint.get("name")}: bad origin')
    return xyz, rpy


def parse_robot_description(urdf, body_link='dummy_link', body_frame='base_link',
                            weld_joint='spiderx_fixed_base_weld'):
    """WeldInfo from a URDF string or ElementTree root. Raises FixedBaseError if malformed.

    A description without a `world` link is a FREE-BASE description (fixed_base False). A
    fixed-base description must have exactly one joint whose parent is `world`: fixed, named
    weld_joint, child body_link, origin roll = pitch = 0.
    """
    try:
        root = ET.fromstring(urdf) if isinstance(urdf, (str, bytes)) else urdf
    except ET.ParseError as e:
        raise FixedBaseError(DESCRIPTION_INVALID, f'not well-formed XML: {e}') from None
    if root is None or getattr(root, 'tag', None) != 'robot':
        raise FixedBaseError(DESCRIPTION_INVALID,
                             f'root element is {getattr(root, "tag", None)}, not robot')
    links = [el.get('name') for el in root.findall('link')]
    joints = root.findall('joint')
    by_name = {j.get('name'): j for j in joints}
    if len(by_name) != len(joints) or len(set(links)) != len(links):
        raise FixedBaseError(DESCRIPTION_INVALID, 'duplicate link or joint names')
    if body_link not in links:
        raise FixedBaseError(DESCRIPTION_INVALID, f'no link {body_link}')
    dj = [j for j in joints if j.find('parent').get('link') == body_link
          and j.find('child').get('link') == body_frame]
    if body_frame != body_link and len(dj) != 1:
        raise FixedBaseError(DESCRIPTION_INVALID, f'expected one joint {body_link} -> {body_frame}')
    if body_frame == body_link:
        dummy_to_body = Pose()
    else:
        if dj[0].get('type') != 'fixed':
            raise FixedBaseError(DESCRIPTION_INVALID, f'{dj[0].get("name")} must be fixed')
        xyz, rpy = _origin_pose(dj[0])
        dummy_to_body = Pose.from_xyz_rpy(*xyz, *rpy)
    if WORLD_LINK not in links:
        return WeldInfo(False, dummy_to_body=dummy_to_body)
    world = [el for el in root.findall('link') if el.get('name') == WORLD_LINK][0]
    if len(world):
        raise FixedBaseError(DESCRIPTION_INVALID, 'the world link must be empty (no inertia, '
                                                  'visual or collision)')
    to_world = [j for j in joints if j.find('child').get('link') == WORLD_LINK]
    if to_world:
        raise FixedBaseError(DESCRIPTION_INVALID, 'world must be the root link')
    from_world = [j for j in joints if j.find('parent').get('link') == WORLD_LINK]
    if len(from_world) != 1:
        raise FixedBaseError(DESCRIPTION_INVALID,
                             f'expected exactly one joint from world, found {len(from_world)}')
    w = from_world[0]
    if w.get('name') != weld_joint:
        raise FixedBaseError(DESCRIPTION_INVALID, f'the world joint is {w.get("name")}, '
                                                  f'expected {weld_joint}')
    if w.get('type') != 'fixed' or w.find('child').get('link') != body_link:
        raise FixedBaseError(DESCRIPTION_INVALID, f'{weld_joint} must be fixed world -> '
                                                  f'{body_link}')
    xyz, rpy = _origin_pose(w)
    if abs(rpy[0]) > ROLL_PITCH_TOL_RAD or abs(rpy[1]) > ROLL_PITCH_TOL_RAD:
        raise FixedBaseError(DESCRIPTION_INVALID, f'{weld_joint} roll/pitch must be 0, got '
                                                  f'{rpy[0]}, {rpy[1]}')
    weld = Pose.from_xyz_rpy(*xyz, *rpy)
    return WeldInfo(True, weld_joint, weld, dummy_to_body, weld, weld * dummy_to_body)


def strip_weld(urdf_root, weld_joint='spiderx_fixed_base_weld'):
    """A copy of the description without the world link and the weld joint."""
    import copy
    r = copy.deepcopy(urdf_root)
    for el in list(r):
        if (el.tag == 'link' and el.get('name') == WORLD_LINK) or (
                el.tag == 'joint' and el.get('name') == weld_joint):
            r.remove(el)
    return r


def canonical_xml(el):
    """Whitespace-insensitive canonical form (tag, sorted attributes, stripped text, children)."""
    return (el.tag, tuple(sorted(el.attrib.items())), ' '.join((el.text or '').split()),
            tuple(canonical_xml(c) for c in el))


def same_model_except_weld(wrapper_root, free_root, weld_joint='spiderx_fixed_base_weld'):
    """True if the wrapper equals the free-base description after removing world + weld."""
    return canonical_xml(strip_weld(wrapper_root, weld_joint)) == canonical_xml(free_root)


def check_mount(weld, cfg):
    """Failure codes for a fixed-base description against the approved mount (may be empty)."""
    codes = []
    if not weld.fixed_base:
        return [DESCRIPTION_NOT_FIXED_BASE]
    if weld.weld_origin.p[2] + 1e-12 < cfg.min_mount_height_m:
        codes.append(MOUNT_TOO_LOW)
    t, r, _, _ = deviation(weld.weld_origin, cfg.mount)
    if t > cfg.link_translation_tol_m or r > cfg.link_rotation_tol_rad:
        codes.append(MOUNT_MISMATCH)
    return codes


# ---------------------------------------------------------------- converted SDF (offline check)
def _sdf_pose(el):
    pe = el.find('pose')
    if pe is None or not (pe.text or '').split():
        return None, Pose()
    v = [float(x) for x in pe.text.split()]
    if len(v) != 6:
        raise FixedBaseError('sdf_invalid', f'{el.get("name")}: pose must have 6 values')
    return pe.get('relative_to'), Pose.from_xyz_rpy(*v)


def resolve_sdf_model(sdf_text):
    """{frame name: T_model_frame} for every link, joint and frame of the ONE model in an SDF
    1.9 document (e.g. `ign sdf -p` output), by explicit composition. SDF defaults: a link pose is
    relative to __model__, a joint pose to its child link, a frame pose to its attached_to frame
    (or __model__). Also returns the model element facts this module relies on."""
    root = ET.fromstring(sdf_text)
    models = root.findall('model')
    if len(models) != 1:
        raise FixedBaseError('sdf_invalid', f'expected one model, found {len(models)}')
    m = models[0]
    rel = {'__model__': (None, Pose())}
    for kind in ('link', 'joint', 'frame'):
        for e in m.findall(kind):
            ref, pose = _sdf_pose(e)
            if ref is None:
                ref = ('__model__' if kind == 'link' else e.findtext('child') if kind == 'joint'
                       else (e.get('attached_to') or '__model__'))
            rel[e.get('name')] = (ref, pose)
    cache = {}

    def resolve(n, depth=0):
        if n in cache:
            return cache[n]
        if depth > 500 or n not in rel:
            raise FixedBaseError('sdf_invalid', f'cannot resolve frame {n}')
        ref, pose = rel[n]
        out = pose if ref is None else resolve(ref, depth + 1) * pose
        cache[n] = out
        return out
    frames = {n: resolve(n) for n in rel}
    world_joints = [{'name': j.get('name'), 'type': j.get('type'), 'child': j.findtext('child')}
                    for j in m.findall('joint') if j.findtext('parent') == WORLD_LINK]
    links = [lk.get('name') for lk in m.findall('link')]
    facts = {'model_name': m.get('name'), 'model_has_pose': m.find('pose') is not None,
             'canonical_link': m.get('canonical_link') or (links[0] if links else None),
             'world_joints': world_joints, 'links': links}
    return frames, facts


# ---------------------------------------------------------------- pose observations
@dataclass(frozen=True)
class PoseSelection:
    """The entries of one TFMessage that matter, or why they cannot be used."""
    codes: tuple                      # empty = usable
    model: Pose = None                # T_world_model
    body_link: Pose = None            # T_model_body_link (Gazebo-side entry)
    frame_ids: tuple = ()             # header.frame_id of (model entry, body entry), recorded


def _entry_pose(t):
    tr, q = t.transform.translation, t.transform.rotation
    vals = (tr.x, tr.y, tr.z, q.x, q.y, q.z, q.w)
    if not _finite(*vals):
        return None, POSE_NONFINITE
    n = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if abs(n - 1.0) > QUAT_NORM_TOL:
        return None, POSE_BAD_QUATERNION
    return Pose((float(tr.x), float(tr.y), float(tr.z)),
                (q.x / n, q.y / n, q.z / n, q.w / n)), None


def select_entries(transforms, model_name='spiderx', body_link='dummy_link'):
    """Pick the model entry and the body-link entry out of a TFMessage.transforms list.

    Missing, duplicated (ambiguous), non-finite or non-unit entries are reported by code and
    never guessed around.
    """
    codes = []
    mods = [t for t in transforms if getattr(t, 'child_frame_id', None) == model_name]
    bods = [t for t in transforms if getattr(t, 'child_frame_id', None) == body_link]
    if not mods:
        codes.append(POSE_MISSING_MODEL)
    elif len(mods) > 1:
        codes.append(POSE_AMBIGUOUS_MODEL)
    if not bods:
        codes.append(POSE_MISSING_BODY_LINK)
    elif len(bods) > 1:
        codes.append(POSE_AMBIGUOUS_BODY_LINK)
    if codes:
        return PoseSelection(tuple(codes))
    model, c1 = _entry_pose(mods[0])
    body, c2 = _entry_pose(bods[0])
    codes = tuple(c for c in (c1, c2) if c)
    fids = tuple(getattr(getattr(t, 'header', None), 'frame_id', None) for t in (mods[0], bods[0]))
    if codes:
        return PoseSelection(tuple(sorted(set(codes))), frame_ids=fids)
    return PoseSelection((), model, body, fids)


@dataclass(frozen=True)
class BodyObservation:
    """One evaluated ground-truth sample."""
    wall: float                       # monotonic receipt time (s)
    stamp: float                      # receiving node's /clock at receipt (s) or None
    codes: tuple                      # problems with THIS sample (empty = clean)
    body: Pose = None                 # T_world_body (physical body = base_link)
    model: Pose = None                # T_world_model
    link_entry: Pose = None           # T_model_dummy as Gazebo reports it
    link_deviation: tuple = None      # (m, rad) Gazebo dummy_link entry vs description
    spawn_deviation: tuple = None     # (m, rad) model root vs identity
    attachment: tuple = None          # (m, rad, dz, dxy) body vs expected weld pose

    def pose6(self):
        """(x, y, z, roll, pitch, yaw) of the body, the M6.1 gate format, or None."""
        return None if self.body is None else self.body.xyz_rpy()


def evaluate_sample(selection, weld, cfg, wall, stamp):
    """Compose the body pose and classify one TFMessage sample against the fixed-base model.

    The body is T_world_model * T_model_dummy(Gazebo entry) * T_dummy_body. Frame consistency
    (Gazebo link entry vs description) and spawn identity are separate codes from the
    attachment displacement, which compares the composed body with the expected weld pose.
    """
    if selection.codes:
        return BodyObservation(wall, stamp, tuple(selection.codes))
    if weld is None or not weld.fixed_base:
        return BodyObservation(wall, stamp, (DESCRIPTION_NOT_FIXED_BASE,))
    body = selection.model * selection.body_link * weld.dummy_to_body
    lt, lr, _, _ = deviation(selection.body_link, weld.model_to_dummy)
    st, sr, _, _ = deviation(selection.model, Pose())
    att = deviation(body, weld.expected_world_body())
    codes = []
    if lt > cfg.link_translation_tol_m or lr > cfg.link_rotation_tol_rad:
        codes.append(FRAME_LINK_INCONSISTENT)
    if st > cfg.spawn_translation_tol_m or sr > cfg.spawn_rotation_tol_rad:
        codes.append(FRAME_SPAWN_NOT_IDENTITY)
    if att[0] > cfg.attachment_translation_tol_m or att[1] > cfg.attachment_rotation_tol_rad:
        codes.append(ATTACHMENT_DISPLACED)
    return BodyObservation(wall, stamp, tuple(codes), body, selection.model, selection.body_link,
                           (lt, lr), (st, sr), att)


class AttachmentMonitor:
    """Debounced attachment-integrity check over consecutive samples (pure; times injected)."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.consecutive = 0
        self.max_translation_m = 0.0
        self.max_rotation_rad = 0.0
        self.samples = 0

    def update(self, obs):
        """Returns ATTACHMENT_DISPLACED once debounced, else None. Unusable samples do not count
        as evidence either way (the pose-freshness check covers their absence)."""
        if obs.attachment is None:
            return None
        self.samples += 1
        self.max_translation_m = max(self.max_translation_m, obs.attachment[0])
        self.max_rotation_rad = max(self.max_rotation_rad, obs.attachment[1])
        if ATTACHMENT_DISPLACED in obs.codes:
            self.consecutive += 1
        else:
            self.consecutive = 0
        if self.consecutive >= self.cfg.attachment_debounce_samples:
            return ATTACHMENT_DISPLACED
        return None


# ---------------------------------------------------------------- clocks and streams
class ClockMonitor:
    """Simulation-clock progress, judged in WALL time (monotonic), never mixed.

    * stalled: /clock has not advanced for sim_stall_s of wall time (pause, freeze, lost bridge);
    * reset:   /clock went backwards by more than clock_reset_tol_s (world reset or restart).
    A reset latches until clear() so that evidence from before it is never reused.
    """

    def __init__(self, sim_stall_s, reset_tol_s):
        self.sim_stall_s = sim_stall_s
        self.reset_tol_s = reset_tol_s
        self.last_sim = None
        self.last_advance_wall = None
        self.first_wall = None
        self.reset_seen = False
        self.messages = 0

    def on_clock(self, wall, sim):
        if not _finite(sim):
            return
        self.messages += 1
        if self.first_wall is None:
            self.first_wall = wall
        if self.last_sim is None:
            self.last_sim, self.last_advance_wall = sim, wall
            return
        if sim < self.last_sim - self.reset_tol_s:
            self.reset_seen = True
            self.last_sim, self.last_advance_wall = sim, wall
            return
        if sim > self.last_sim:
            self.last_sim, self.last_advance_wall = sim, wall

    def check(self, now_wall):
        """First violated rule (code) or None."""
        if self.reset_seen:
            return CLOCK_RESET
        if self.last_sim is None:
            return CLOCK_MISSING
        if now_wall - self.last_advance_wall > self.sim_stall_s:
            return CLOCK_STALLED
        return None


def check_joint_states(latest, now_wall, joint_names, stale_s):
    """Code for the latest (wall, {name: position}) joint-state sample, or None."""
    if latest is None:
        return JOINT_STATES_MISSING
    wall, pos = latest
    if now_wall - wall > stale_s:
        return JOINT_STATES_STALE
    if any(n not in pos or not _finite(pos[n]) for n in joint_names):
        return JOINT_STATES_INCOMPLETE
    return None


@dataclass(frozen=True)
class PoseSample:
    """A received TFMessage whose model and body-link entries were usable (no selection code)."""
    wall: float                       # monotonic receipt time (s)
    stamp: float                      # receiving node's /clock at receipt (s) or None
    selection: PoseSelection


def check_pose_freshness(latest, now_wall, stale_s):
    """Code for the latest usable PoseSample (by receipt wall time), or None."""
    if latest is None:
        return POSE_NEVER_RECEIVED
    if now_wall - latest.wall > stale_s:
        return POSE_STALE
    return None


# ---------------------------------------------------------------- readiness
@dataclass
class Evidence:
    """Everything the read-only observer or the M6.1 transport has seen (all optional)."""
    description: str = None                   # /robot_description (transient local)
    description_received_wall: float = None
    latest_usable_pose: PoseSample = None      # last sample whose entries were usable
    latest_pose_codes: tuple = ()              # selection codes of the most recent sample (any)
    latest_joint_states: tuple = None          # (wall, {name: position})
    clock: ClockMonitor = None
    controllers: dict = None                   # {name: state} or None = not queried
    joint_state_publishers: int = None
    command_publishers: int = None


INTEGRITY_CODES = (ATTACHMENT_DISPLACED, FRAME_SPAWN_NOT_IDENTITY, FRAME_LINK_INCONSISTENT)


def assess_plant(description, latest_usable_pose, cfg):
    """(ok, codes, checks) for the fixed-base PLANT only: the description (fixed base, valid, the
    approved mount above the clearance minimum) and the latest usable pose sample (Gazebo link
    entry consistent with the description, model root at the identity spawn, body attached).
    Freshness, joint states, the clock and the graph are separate checks (see assess())."""
    codes, checks = [], {}

    def put(name, code, detail=None):
        checks[name] = {'ok': code is None, 'code': code, 'detail': detail}
        if code is not None:
            codes.append(code)

    weld = None
    if description is None:
        put('robot_description', DESCRIPTION_MISSING)
    else:
        try:
            weld = parse_robot_description(description, cfg.body_link, cfg.body_frame,
                                           cfg.weld_joint)
        except FixedBaseError as e:
            put('robot_description', e.code, str(e))
        else:
            mc = check_mount(weld, cfg)
            put('robot_description', mc[0] if mc else None,
                {'fixed_base': weld.fixed_base,
                 'weld_xyz_rpy': None if weld.weld_origin is None else weld.weld_origin.xyz_rpy(),
                 'all_codes': mc})
            codes.extend(mc[1:])
    if latest_usable_pose is None:
        put('fixed_base_pose', POSE_NEVER_RECEIVED)
    elif weld is not None and weld.fixed_base:
        obs = evaluate_sample(latest_usable_pose.selection, weld, cfg, latest_usable_pose.wall,
                              latest_usable_pose.stamp)
        put('frame_body_link', FRAME_LINK_INCONSISTENT if FRAME_LINK_INCONSISTENT in obs.codes
            else None, {'translation_m': obs.link_deviation[0],
                        'rotation_rad': obs.link_deviation[1]})
        put('spawn_identity', FRAME_SPAWN_NOT_IDENTITY if FRAME_SPAWN_NOT_IDENTITY in obs.codes
            else None, {'translation_m': obs.spawn_deviation[0],
                        'rotation_rad': obs.spawn_deviation[1]})
        put('attachment', ATTACHMENT_DISPLACED if ATTACHMENT_DISPLACED in obs.codes else None,
            dict(zip(('translation_m', 'rotation_rad', 'dz_m', 'dxy_m'), obs.attachment)))
        checks['body_pose_xyz_rpy'] = obs.body.xyz_rpy()
        checks['body_tilt_rad'] = obs.body.tilt()
    return not codes, codes, checks


def assess(evidence, cfg, now_wall, joint_names):
    """(ready, failure codes, report) for the read-only observer. Decides nothing about a goal.

    Every check is separate. READY needs all of: the plant checks (assess_plant); a fresh usable
    body pose (wall receipt age); fresh complete joint states; a progressing /clock with no
    reset; both controllers active; exactly one /joint_states publisher; no command publisher.
    """
    ok, codes, checks = assess_plant(evidence.description, evidence.latest_usable_pose, cfg)
    rep = {'checks': checks}

    def put(name, code, detail=None):
        checks[name] = {'ok': code is None, 'code': code, 'detail': detail}
        if code is not None:
            codes.append(code)

    smp = evidence.latest_usable_pose
    put('body_pose_fresh', check_pose_freshness(smp, now_wall, cfg.pose_stale_s),
        {'age_s': None if smp is None else now_wall - smp.wall,
         'latest_sample_codes': list(evidence.latest_pose_codes)})
    put('joint_states', check_joint_states(evidence.latest_joint_states, now_wall, joint_names,
                                           cfg.joint_states_stale_s))
    put('sim_clock', CLOCK_MISSING if evidence.clock is None else evidence.clock.check(now_wall))
    if evidence.controllers is None:
        put('controllers', CONTROLLERS_UNKNOWN)
    else:
        bad = {c: evidence.controllers.get(c, 'missing') for c in cfg.controllers
               if evidence.controllers.get(c) != 'active'}
        put('controllers', CONTROLLERS_NOT_ACTIVE if bad else None, bad or None)
    put('joint_state_publishers', None if evidence.joint_state_publishers == 1
        else JOINT_STATE_PUBLISHERS, evidence.joint_state_publishers)
    put('command_publishers', None if evidence.command_publishers == 0 else COMMAND_PUBLISHERS,
        evidence.command_publishers)
    return not codes, codes, rep


# ---------------------------------------------------------------- shared live/mock tracker
class FixedBasePoseTracker:
    """TFMessage transforms + /robot_description -> M6.1 events, the same code for live and mock.

    on_transforms() returns the events the M6.1 session consumes:
      ('body_pose', wall, stamp, (x, y, z, roll, pitch, yaw))   the COMPOSED body (base_link)
      ('fixed_base', wall, stamp, codes, attachment)            the plant checks of that sample
    A sample whose entries are missing, ambiguous or malformed yields no event (so it never
    refreshes pose freshness); without a fixed-base description no body pose can be composed,
    so only a 'fixed_base' event with the description code is returned.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.description = None
        self.description_wall = None
        self.weld = None
        self.description_code = DESCRIPTION_MISSING
        self.latest_usable = None
        self.latest_codes = ()
        self.messages = 0
        self.invalid = {}

    def on_description(self, text, wall):
        self.description, self.description_wall = text, wall
        try:
            self.weld = parse_robot_description(text, self.cfg.body_link, self.cfg.body_frame,
                                                self.cfg.weld_joint)
            self.description_code = None if self.weld.fixed_base else DESCRIPTION_NOT_FIXED_BASE
        except FixedBaseError as e:
            self.weld, self.description_code = None, e.code

    def on_transforms(self, transforms, wall, stamp):
        self.messages += 1
        sel = select_entries(transforms, self.cfg.model_name, self.cfg.body_link)
        self.latest_codes = sel.codes
        if sel.codes:
            for c in sel.codes:
                self.invalid[c] = self.invalid.get(c, 0) + 1
            return []
        self.latest_usable = PoseSample(wall, stamp, sel)
        if self.description_code is not None:
            return [('fixed_base', wall, stamp, (self.description_code,), None)]
        obs = evaluate_sample(sel, self.weld, self.cfg, wall, stamp)
        return [('body_pose', wall, stamp, obs.pose6()),
                ('fixed_base', wall, stamp, obs.codes, obs.attachment)]

    def latest_body_pose(self):
        """(wall, stamp, pose6) of the latest usable sample composed with the weld, or None."""
        smp = self.latest_usable
        if smp is None or self.description_code is not None:
            return None
        obs = evaluate_sample(smp.selection, self.weld, self.cfg, smp.wall, smp.stamp)
        return (smp.wall, smp.stamp, obs.pose6())

    def snapshot(self):
        return {'description': self.description, 'latest_usable_pose': self.latest_usable,
                'latest_pose_codes': self.latest_codes, 'messages': self.messages,
                'invalid_samples': dict(self.invalid)}


# ---------------------------------------------------------------- message-like helpers (no ROS)
def _ns(**kw):
    from types import SimpleNamespace
    return SimpleNamespace(**kw)


def make_transform(child_frame_id, pose, frame_id=''):
    """A TransformStamped-shaped object (the fields this module reads), for mocks and tests."""
    return _ns(child_frame_id=child_frame_id, header=_ns(frame_id=frame_id),
               transform=_ns(translation=_ns(x=pose.p[0], y=pose.p[1], z=pose.p[2]),
                             rotation=_ns(x=pose.q[0], y=pose.q[1], z=pose.q[2], w=pose.q[3])))


def minimal_description(mount=None, fixed_base=True, weld_joint='spiderx_fixed_base_weld',
                        dummy_to_body=None):
    """A minimal URDF string with the frames this module reads (for mocks and tests only)."""
    db = dummy_to_body or Pose()
    dr = db.rpy()
    weld = ''
    if fixed_base:
        m = mount or Pose()
        r = m.rpy()
        weld = (f'<link name="world"/><joint name="{weld_joint}" type="fixed">'
                f'<parent link="world"/><child link="dummy_link"/>'
                f'<origin xyz="{m.p[0]!r} {m.p[1]!r} {m.p[2]!r}" '
                f'rpy="{r[0]!r} {r[1]!r} {r[2]!r}"/></joint>')
    return ('<robot name="spiderx"><link name="dummy_link"/><link name="base_link"/>'
            '<joint name="dummy_joint" type="fixed"><parent link="dummy_link"/>'
            f'<child link="base_link"/><origin xyz="{db.p[0]!r} {db.p[1]!r} {db.p[2]!r}" '
            f'rpy="{dr[0]!r} {dr[1]!r} {dr[2]!r}"/></joint>{weld}</robot>')


__all__ = [n for n in dir() if not n.startswith('_')]
