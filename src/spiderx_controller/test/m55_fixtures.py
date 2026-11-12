"""Shared M5.5 test fixtures: a synthetic phase library (fast) and the real one (slow, cached).

The synthetic templates have the real joint names and phase sequence but fabricated, smooth joint
paths; they exercise the state machine, the goal library and the transports without the ~20 s
per-level kinematic design. Gait feasibility itself is tested against the real templates.
"""

import functools
import math
import os

from spiderx_controller import m55_crawl as cr
from spiderx_controller import m55_locomotion as loc

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'config')
JOINTS = [f'{p}_{j}' for p in ('lf', 'rf', 'lr', 'rr')
          for j in ('hip', 'thigh_joint', 'foot_joint')]
PHASES = [('shift', None), ('swing', 'rear_left'), ('shift', None), ('swing', 'front_left'),
          ('shift', None), ('swing', 'rear_right'), ('shift', None), ('swing', 'front_right'),
          ('shift', None)]


def config():
    return loc.load_config(CONFIG_DIR)


def synthetic_template(stride, phase_points=4, dt=0.1, amp=0.4):
    """A forward CrawlTemplate-shaped object: rest at every boundary, neutral (0) at both ends,
    boundary k of stride s at amp*s/0.02*sin(pi*k/P)*(1+k/2P) per joint (levels differ
    mid-cycle)."""
    P = len(PHASES)
    scale = stride / 0.02

    def boundary(k):
        return [amp * scale * math.sin(math.pi * k / P) * (1 + 0.5 * k / P) * (1 + 0.1 * j)
                for j in range(len(JOINTS))]          # asymmetric: no phase equals another
    pts, bounds, t = [], [0], 0.0
    for k in range(P):
        a, b = boundary(k), boundary(k + 1)
        for i in range(phase_points + (1 if k == 0 else 0)):
            s = (i if k == 0 else i + 1) / phase_points
            h = 3 * s * s - 2 * s ** 3
            dh = (6 * s - 6 * s * s) / (phase_points * dt)
            pts.append({'time_from_start_s': round(t + (s * phase_points * dt), 9),
                        'positions': [x + (y - x) * h for x, y in zip(a, b)],
                        'velocities': [(y - x) * dh for x, y in zip(a, b)]})
        t = round(t + phase_points * dt, 9)
        bounds.append(len(pts) - 1)
    for i in bounds:
        pts[i]['velocities'] = [0.0] * len(JOINTS)
    pts[-1]['positions'] = [0.0] * len(JOINTS)
    params = cr.CrawlParams(stride, 0.012, 0.5, 0.6, 0.02, 0.015, 0.25, 0.9, dt, 0.02, 0.002)
    body = [(0.0, stride * k / P) for k in range(P + 1)]
    report = {'schema': cr.SCHEMA, 'failures': [], 'speed_m_s': stride / t}
    return cr.CrawlTemplate(params, 1, list(JOINTS), pts, report, bounds, list(PHASES), body)


def synthetic_library(strides=(0.02, 0.04, 0.06), lead_s=0.2):
    return loc.PhaseLibrary([synthetic_template(s) for s in strides], lead_s)


@functools.lru_cache(maxsize=None)
def real_library(strides=None):
    """The real templates for the configured levels (or `strides`); ~20 s per level."""
    cfg = config()
    designer = cr.load_designer()
    levels = cfg.gait.stride_levels_m if strides is None else strides
    temps = [cr.build_template(designer, cfg.crawl_params(s)) for s in levels]
    return loc.PhaseLibrary(temps, cfg.dispatch.phase_lead_s), temps
