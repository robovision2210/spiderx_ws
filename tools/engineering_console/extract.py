#!/usr/bin/env python3
"""Read-only data extraction for the SpiderX Engineering Console.

Reads the repository (URDF xacro, STL meshes, one committed evidence CSV, screenshots) and returns
plain Python data for build.py. It imports nothing from the robot packages and runs none of the
repository scripts. Standard library only.
"""
import base64
import csv
import json
import os
import struct
import subprocess
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
DESC = os.path.join(REPO, 'src', 'spiderx_description')
URDF = os.path.join(DESC, 'urdf', 'spiderx.urdf.xacro')
RUN = 'docs/evidence/m61_cloud_one_cycle/20261012T103615Z'
CSV_PATH = os.path.join(REPO, RUN, 'run', 'm61_run', '20261012T104415Z', 'commanded_vs_observed.csv')
QUANT = 1e-5          # mesh quantisation step, metres (10 micrometres)
TIP_TOL = 1e-4        # project rule: vertices within 0.1 mm of the lowest z (leg_kinematics.py)

LEGS = {
    'lf': ('front_left', ['lf_hip', 'lf_thigh_joint', 'lf_foot_joint'], 'lf_foot_1'),
    'rf': ('front_right', ['rf_hip', 'rf_thigh_joint', 'rf_foot_joint'], 'rf_foot_1'),
    'lr': ('rear_left', ['lr_hip', 'lr_thigh_joint', 'lr_foot_joint'], 'lr_foot_1'),
    'rr': ('rear_right', ['rr_hip', 'rr_thigh_joint', 'rr_foot_joint'], 'rr_foot_1'),
}


def vec(s):
    return [float(x) for x in s.split()] if s else [0.0, 0.0, 0.0]


def git(*args):
    return subprocess.check_output(['git', '-C', REPO] + list(args), text=True).strip()


def load_stl(path):
    with open(path, 'rb') as f:
        data = f.read()
    n = struct.unpack_from('<I', data, 80)[0]
    assert len(data) == 84 + 50 * n, f'{path}: not a binary STL'
    tris = []
    for i in range(n):
        v = struct.unpack_from('<9f', data, 84 + 50 * i + 12)
        tris.append((v[0:3], v[3:6], v[6:9]))
    return tris


def line_of(text_lines, needle):
    for i, ln in enumerate(text_lines, 1):
        if needle in ln:
            return i
    return None


def parse_urdf():
    with open(URDF) as f:
        lines = f.read().splitlines()
    root = ET.parse(URDF).getroot()
    links, joints = [], []
    for el in root.findall('link'):
        name = el.get('name')
        vis = el.find('visual')
        mesh, vxyz, cyl = None, [0, 0, 0], None
        if vis is not None:
            o = vis.find('origin')
            vxyz = vec(o.get('xyz')) if o is not None else [0, 0, 0]
            m = vis.find('geometry/mesh')
            c = vis.find('geometry/cylinder')
            if m is not None:
                mesh = os.path.basename(m.get('filename'))
            if c is not None:
                cyl = {'radius': float(c.get('radius')), 'length': float(c.get('length'))}
        inertial = el.find('inertial')
        mass = float(inertial.find('mass').get('value')) if inertial is not None else None
        links.append({'name': name, 'mesh': mesh, 'visual_xyz': vxyz, 'cylinder': cyl, 'mass': mass,
                      'line': line_of(lines, f'<link name="{name}">')})
    for el in root.findall('joint'):
        o = el.find('origin')
        ax = el.find('axis')
        lim = el.find('limit')
        j = {'name': el.get('name'), 'type': el.get('type'),
             'parent': el.find('parent').get('link'), 'child': el.find('child').get('link'),
             'xyz': vec(o.get('xyz')) if o is not None else [0.0, 0.0, 0.0],
             'rpy': vec(o.get('rpy')) if o is not None else [0.0, 0.0, 0.0],
             'line': line_of(lines, f'<joint name="{el.get("name")}"')}
        if el.get('type') == 'revolute':
            j['axis'] = [x + 0.0 for x in vec(ax.get('xyz'))]
            j['lower'], j['upper'] = float(lim.get('lower')), float(lim.get('upper'))
            j['effort'], j['velocity'] = float(lim.get('effort')), float(lim.get('velocity'))
        joints.append(j)
    return links, joints


def zero_pose_origins(links, joints):
    """Link-frame origins in base_link at q = 0. Every leg-chain rpy is 0, so only translations add
    (lidar_joint has a yaw, but lidar_link is a leaf, so its origin is still a pure sum)."""
    by_child = {j['child']: j for j in joints}
    out = {}

    def origin(name):
        if name in out:
            return out[name]
        if name not in by_child:
            out[name] = [0.0, 0.0, 0.0]
        else:
            j = by_child[name]
            p = origin(j['parent'])
            out[name] = [p[k] + j['xyz'][k] for k in range(3)]
        return out[name]
    for l in links:
        origin(l['name'])
    return out


def build_meshes(links):
    """Quantised, indexed meshes in each LINK frame (STL mm -> m, plus the URDF visual origin)."""
    blob = bytearray()
    index = {}
    total_tris = 0
    raw = {}
    for l in links:
        if not l['mesh']:
            continue
        tris = load_stl(os.path.join(DESC, 'meshes', l['mesh']))
        ox, oy, oz = l['visual_xyz']
        vmap, verts, idx = {}, [], []
        link_tris = []
        for tri in tris:
            pts = []
            for v in tri:
                p = (v[0] * 0.001 + ox, v[1] * 0.001 + oy, v[2] * 0.001 + oz)
                pts.append(p)
                q = (round(p[0] / QUANT), round(p[1] / QUANT), round(p[2] / QUANT))
                assert all(-32768 <= c <= 32767 for c in q), (l['name'], q)
                if q not in vmap:
                    vmap[q] = len(verts)
                    verts.append(q)
                idx.append(vmap[q])
            link_tris.append(pts)
        assert len(verts) < 65536
        raw[l['name']] = link_tris
        while len(blob) % 2:
            blob.append(0)
        off = len(blob)
        for q in verts:
            blob += struct.pack('<3h', *q)
        ioff = len(blob)
        blob += struct.pack(f'<{len(idx)}H', *idx)
        index[l['name']] = {'v': [off, len(verts)], 'i': [ioff, len(idx)], 'tris': len(tris)}
        total_tris += len(tris)
    return bytes(blob), index, total_tris, raw


def foot_tips(raw, origins):
    """The project's derived foot-tip rule (leg_kinematics.LegGeometry.from_urdf), recomputed here:
    lowest point of the foot collision mesh at q = 0; mean x, y of the unique vertices within
    0.1 mm of the minimum base_link z; z = that minimum."""
    tips = {}
    for key, (_, _, foot) in LEGS.items():
        o = origins[foot]
        pts = {tuple(round(p[k] + o[k], 9) for k in range(3)) for tri in raw[foot] for p in tri}
        zmin = min(p[2] for p in pts)
        low = [p for p in pts if p[2] <= zmin + TIP_TOL]
        tip = [sum(p[0] for p in low) / len(low), sum(p[1] for p in low) / len(low), zmin]
        tips[key] = {'tip_base': tip, 'tip_local': [tip[k] - o[k] for k in range(3)], 'n': len(low)}
    return tips


def bbox(raw, origins):
    lo, hi = [1e9] * 3, [-1e9] * 3
    for name, tris in raw.items():
        o = origins[name]
        for tri in tris:
            for p in tri:
                for k in range(3):
                    lo[k] = min(lo[k], p[k] + o[k])
                    hi[k] = max(hi[k], p[k] + o[k])
    return lo, hi


def playback():
    """Recorded commanded vs observed joint positions of the one PASSED fixed-base trot cycle."""
    with open(CSV_PATH) as f:
        rows = list(csv.reader(f))
    head, rows = rows[0], rows[1:]
    names = [h[:-4] for h in head if h.endswith('_cmd')]
    col = {h: i for i, h in enumerate(head)}
    t, cmd, obs = [], [], []
    max_err, max_joint, max_t = 0.0, None, None
    per_joint = {n: 0.0 for n in names}
    for r in rows:
        tt = float(r[col['ref_time_s']])
        for n in names:
            e = abs(float(r[col[n + '_err']]))
            per_joint[n] = max(per_joint[n], e)
            if e > max_err:
                max_err, max_joint, max_t = e, n, tt
    for r in rows[::2]:
        t.append(round(float(r[col['ref_time_s']]), 3))
        cmd.append([round(float(r[col[n + '_cmd']]) * 1e5) for n in names])
        obs.append([round(float(r[col[n + '_obs']]) * 1e5) for n in names])
    return {'joints': names, 't': t, 'cmd': cmd, 'obs': obs, 'scale': 1e-5, 'rows_total': len(rows),
            'max_abs_err_rad': max_err, 'max_err_joint': max_joint, 'max_err_ref_time_s': max_t,
            'per_joint_max_err': per_joint, 'source': os.path.relpath(CSV_PATH, REPO)}


IMAGES = [
    ('local_gazebo', 'media/local_validation/gazebo_fortress_ubuntu.png'),
    ('local_lidar', 'media/local_validation/gazebo_lidar_rays_collisions.png'),
    ('cloud_gui', 'media/cloud_validation/gazebo_fortress_gui.png'),
    ('cloud_rviz', 'media/cloud_validation/rviz_scan_tf.png'),
    ('cloud_m1', 'media/cloud_validation/m1_joints_held_cad_neutral.png'),
    ('cloud_m2', 'media/cloud_validation/m2_cad_neutral_posture_hold.png'),
    ('m61_side', RUN + '/run/captures/screenshot_gui_side.png'),
    ('m61_foot', RUN + '/run/captures/screenshot_gui_foot_level.png'),
]


def images():
    out = {}
    for key, rel in IMAGES:
        path = os.path.join(REPO, rel)
        with open(path, 'rb') as f:
            data = f.read()
        w, h = struct.unpack('>II', data[16:24])
        added = git('log', '--diff-filter=A', '--format=%h %ad', '--date=short', '--', rel).splitlines()[-1]
        out[key] = {'path': rel, 'w': w, 'h': h, 'commit': added.split()[0], 'date': added.split()[1],
                    'uri': 'data:image/png;base64,' + base64.b64encode(data).decode()}
    return out


def snapshot():
    return {'branch': git('rev-parse', '--abbrev-ref', 'HEAD'), 'commit': git('rev-parse', 'HEAD'),
            'short': git('rev-parse', '--short', 'HEAD'),
            'date': git('log', '-1', '--format=%ad', '--date=short'),
            'subject': git('log', '-1', '--format=%s'),
            'commits': int(git('rev-list', '--count', 'HEAD'))}


def extract():
    links, joints = parse_urdf()
    origins = zero_pose_origins(links, joints)
    blob, mesh_index, total_tris, raw = build_meshes(links)
    tips = foot_tips(raw, origins)
    lo, hi = bbox(raw, origins)
    robot = {
        'links': links, 'joints': joints, 'mesh_index': mesh_index, 'quant': QUANT,
        'legs': {k: {'name': v[0], 'joints': v[1], 'foot_link': v[2], **tips[k]} for k, v in LEGS.items()},
        'counts': {'links': len(links), 'joints': len(joints),
                   'revolute': sum(j['type'] == 'revolute' for j in joints),
                   'fixed': sum(j['type'] == 'fixed' for j in joints),
                   'meshes': len(mesh_index), 'triangles': total_tris},
        'bbox': {'min': lo, 'max': hi, 'size': [hi[k] - lo[k] for k in range(3)]},
        'mass_total': sum(l['mass'] for l in links if l['mass']),
        'origins': origins,
    }
    return {'snapshot': snapshot(), 'robot': robot, 'mesh_b64': base64.b64encode(blob).decode(),
            'playback': playback(), 'images': images()}


if __name__ == '__main__':
    d = extract()
    r = d['robot']
    print(json.dumps({'snapshot': d['snapshot'], 'counts': r['counts'], 'bbox': r['bbox'],
                      'mass_total': r['mass_total'],
                      'tips': {k: v['tip_base'] for k, v in r['legs'].items()},
                      'tip_n': {k: v['n'] for k, v in r['legs'].items()},
                      'mesh_bytes': len(d['mesh_b64']) * 3 // 4,
                      'playback': {k: d['playback'][k] for k in
                                   ('rows_total', 'max_abs_err_rad', 'max_err_joint', 'max_err_ref_time_s')},
                      'image_bytes': sum(len(v['uri']) for v in d['images'].values())}, indent=1))
