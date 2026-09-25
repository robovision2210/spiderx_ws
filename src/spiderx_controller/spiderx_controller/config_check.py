"""Validate the SpiderX controller configuration against the robot description (URDF).

Checks that the leg/joint groups, limits, chain order, named poses and the ros2_control
controller scaffold in this package agree with spiderx_description. It commands nothing.
"""

import math
import os
import subprocess
import xml.etree.ElementTree as ET

import yaml

TOL = 1e-6


def _share(pkg):
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(pkg)


def load_urdf(urdf_path=None):
    """Return the parsed URDF root, expanding the SpiderX xacro when no file is given."""
    if urdf_path:
        with open(urdf_path) as f:
            return ET.fromstring(f.read())
    xacro_file = os.path.join(_share('spiderx_description'), 'urdf', 'spiderx.urdf.xacro')
    out = subprocess.run(['xacro', xacro_file, 'sim_backend:=none'],
                         check=True, capture_output=True, text=True).stdout
    return ET.fromstring(out)


def load_configs(config_dir=None):
    config_dir = config_dir or os.path.join(_share('spiderx_controller'), 'config')

    def rd(name):
        with open(os.path.join(config_dir, name)) as f:
            return yaml.safe_load(f)
    return rd('spiderx_legs.yaml'), rd('spiderx_poses.yaml'), rd('spiderx_ros2_controllers.yaml')


def _vec(s):
    return [float(v) for v in s.split()] if s else [0.0, 0.0, 0.0]


def _rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _zero_pose_positions(joints_by_child, root):
    """Origin of every link in the root frame at q = 0 (fixed and revolute origins only)."""
    pos = {root: ([0.0, 0.0, 0.0], [[1, 0, 0], [0, 1, 0], [0, 0, 1]])}

    def resolve(link):
        if link in pos:
            return pos[link]
        j = joints_by_child[link]
        p_pos, p_rot = resolve(j['parent'])
        xyz, rpy = j['xyz'], j['rpy']
        world = [p_pos[i] + sum(p_rot[i][k] * xyz[k] for k in range(3)) for i in range(3)]
        rot_local = _rpy_matrix(*rpy)
        rot = [[sum(p_rot[i][k] * rot_local[k][c] for k in range(3)) for c in range(3)]
               for i in range(3)]
        pos[link] = (world, rot)
        return pos[link]

    for child in joints_by_child:
        resolve(child)
    return {k: v[0] for k, v in pos.items()}


def check(urdf_root, legs_cfg, poses_cfg, ctrl_cfg):
    """Run all checks. Returns (errors, info) lists of strings."""
    errors, info = [], []
    joints = {}
    for j in urdf_root.findall('joint'):
        o = j.find('origin')
        a = j.find('axis')
        lim = j.find('limit')
        joints[j.get('name')] = {
            'type': j.get('type'),
            'parent': j.find('parent').get('link'),
            'child': j.find('child').get('link'),
            'xyz': _vec(o.get('xyz')) if o is not None else [0.0] * 3,
            'rpy': _vec(o.get('rpy')) if o is not None else [0.0] * 3,
            'axis': _vec(a.get('xyz')) if a is not None else None,
            'lower': float(lim.get('lower')) if lim is not None else None,
            'upper': float(lim.get('upper')) if lim is not None else None,
        }
    by_child = {j['child']: j for j in joints.values()}
    children = set(by_child)
    roots = [l.get('name') for l in urdf_root.findall('link') if l.get('name') not in children]
    if len(roots) != 1:
        errors.append(f'URDF must have exactly one root link, found {roots}')
        return errors, info
    positions = _zero_pose_positions(by_child, roots[0])

    urdf_revolute = {n for n, j in joints.items() if j['type'] == 'revolute'}
    margin = float(legs_cfg['soft_limit_margin_rad'])
    order = legs_cfg['joint_order']
    cfg_joints = {}
    for leg_name, leg in legs_cfg['legs'].items():
        chain = [leg['joints'][k]['name'] for k in order]
        for role in order:
            jc = leg['joints'][role]
            name = jc['name']
            if name in cfg_joints:
                errors.append(f'{name} listed twice')
            cfg_joints[name] = jc
            if name not in joints:
                errors.append(f'{leg_name}.{role}: joint {name} not in URDF')
                continue
            ju = joints[name]
            if ju['type'] != 'revolute':
                errors.append(f'{name}: URDF type is {ju["type"]}, expected revolute')
            for key in ('parent', 'child'):
                if jc[key] != ju[key]:
                    errors.append(f'{name}: {key} {jc[key]} != URDF {ju[key]}')
            if any(abs(float(a) - b) > TOL for a, b in zip(jc['axis'], ju['axis'])):
                errors.append(f'{name}: axis {jc["axis"]} != URDF {ju["axis"]}')
            for key in ('lower', 'upper'):
                if abs(float(jc[key]) - ju[key]) > TOL:
                    errors.append(f'{name}: {key} {jc[key]} != URDF {ju[key]}')
            if abs(float(jc['soft_lower']) - (ju['lower'] + margin)) > TOL or \
               abs(float(jc['soft_upper']) - (ju['upper'] - margin)) > TOL:
                errors.append(f'{name}: soft limits must be URDF limits -/+ {margin}')
            if not float(jc['soft_lower']) < float(jc['soft_upper']):
                errors.append(f'{name}: empty soft range')
        # chain order: hip -> thigh -> foot along the parent chain
        for upper_j, lower_j in zip(chain, chain[1:]):
            if upper_j in joints and lower_j in joints:
                ancestors, link = [], joints[lower_j]['parent']
                while True:
                    ancestors.append(link)
                    if link not in by_child:
                        break
                    link = by_child[link]['parent']
                if joints[upper_j]['child'] not in ancestors:
                    errors.append(f'{leg_name}: {upper_j} is not upstream of {lower_j}')
        if chain[0] in joints and joints[chain[0]]['parent'] != leg['hip_servo_body_link']:
            errors.append(f'{leg_name}: hip parent is not {leg["hip_servo_body_link"]}')
        if chain[-1] in joints and joints[chain[-1]]['child'] != leg['foot_link']:
            errors.append(f'{leg_name}: foot joint child is not {leg["foot_link"]}')

    missing = urdf_revolute - set(cfg_joints)
    extra = set(cfg_joints) - urdf_revolute
    if missing:
        errors.append(f'URDF revolute joints missing from legs config: {sorted(missing)}')
    if extra:
        errors.append(f'legs config joints that are not URDF revolute joints: {sorted(extra)}')
    info.append(f'{len(cfg_joints)} leg joints in {len(legs_cfg["legs"])} legs match the URDF '
                f'({len(urdf_revolute)} revolute joints)')

    # Leg placement: base_link +y = front, +x = right (docs/SPIDERX_URDF_AUDIT.md section 3).
    feet = {n: positions[l['foot_link']] for n, l in legs_cfg['legs'].items()
            if l['foot_link'] in positions}
    if len(feet) == 4:
        checks = [
            ('front_left', 'rear_left', 1, 'front feet must be at larger y (forward) than rear'),
            ('front_right', 'rear_right', 1, 'front feet must be at larger y (forward) than rear'),
            ('front_right', 'front_left', 0, 'right feet must be at larger x than left'),
            ('rear_right', 'rear_left', 0, 'right feet must be at larger x than left'),
        ]
        for a, b, axis, msg in checks:
            if not feet[a][axis] > feet[b][axis]:
                errors.append(f'{msg} ({a} vs {b})')
        info.append('leg placement consistent with +y forward / +x right: ' + ', '.join(
            f'{n}=({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})' for n, p in feet.items()))

    for pose_name, pose in poses_cfg['poses'].items():
        pj = pose['joints']
        if set(pj) != set(cfg_joints):
            errors.append(f'pose {pose_name}: joints differ from legs config '
                          f'(missing {sorted(set(cfg_joints) - set(pj))}, '
                          f'extra {sorted(set(pj) - set(cfg_joints))})')
        for name, value in pj.items():
            if name in cfg_joints:
                jc = cfg_joints[name]
                if not float(jc['soft_lower']) <= float(value) <= float(jc['soft_upper']):
                    errors.append(f'pose {pose_name}: {name}={value} outside soft limits')
        info.append(f'pose {pose_name} ({pose.get("status", "?")}) within soft limits')

    ctrl_joints = ctrl_cfg['leg_trajectory_controller']['ros__parameters']['joints']
    if sorted(ctrl_joints) != sorted(cfg_joints) or len(ctrl_joints) != len(set(ctrl_joints)):
        errors.append('spiderx_ros2_controllers.yaml joints differ from the legs config')
    else:
        info.append('ros2_control scaffold lists exactly the 12 leg joints')

    vmax = legs_cfg['motion_constraints']['max_joint_velocity_rad_s']
    if not (vmax is not None and float(vmax) > 0):
        errors.append('motion_constraints.max_joint_velocity_rad_s must be > 0')
    return errors, info
