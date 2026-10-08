"""M5.5 free-base crawl on the REAL geometry and mass model: template validation, the phase-goal
library, walking against the dispatching fake, the feasibility CLI, and the preservation of the
approved M6.0-D and M6.1 trajectories.

Slow (the kinematic design is ~20-25 s per stride level; the templates are built once per
module). Offline only: no ROS graph, no simulator.
"""

import json
import os

import m55_fixtures as fx
import pytest

from spiderx_controller import leg_kinematics as lk
from spiderx_controller import m55_contract as c55
from spiderx_controller import m55_crawl as cr
from spiderx_controller import m55_fake as fk
from spiderx_controller import m55_feasibility as fe
from spiderx_controller import m55_locomotion as loc
from spiderx_controller import m61_goal as goal61
from spiderx_controller import m61_trot_cycle as tc
from spiderx_controller import m6_goal_fingerprint as gf
from spiderx_controller import m6_trajectory as m6t
from spiderx_controller.config_check import load_urdf

M6D_TRAJECTORY_ID = '44f0a7ad52e5c330'
M6D_FINGERPRINT = '0d6ef4171f2d338a01e76b934be94d1a4386e7c0fc68fe74262fb3c65005ff83'
M61_CONTENT_SHA256 = '94a492c43fcc046125d6bc71ee7f1d9a8a5d8466f1a1bdd9958a16044dd58e64'
M61_TRAJECTORY_ID = '241760e7dfd5ef12'
M61_FINGERPRINT = '9dba1a173e212bfc172ffcd7da59124f87e98a321906e914e9d60e55595e5c3a'


@pytest.fixture(scope='module')
def cfg():
    return fx.config()


@pytest.fixture(scope='module')
def real(cfg):
    return fx.real_library()


@pytest.fixture(scope='module')
def lib(real):
    return real[0]


@pytest.fixture(scope='module')
def temps(real):
    return real[1]


@pytest.fixture(scope='module')
def designer():
    return cr.load_designer()


# ==================================================================== the templates
def test_every_configured_level_validates(cfg, temps):
    assert [t.params.stride_m for t in temps] == list(cfg.gait.stride_levels_m)
    for t in temps:
        r = t.report
        assert r['failures'] == [] and t.direction == 1
        assert r['min_static_margin_m'] >= cfg.gait.min_margin_m
        assert r['spline_min_static_margin_m'] >= cfg.gait.min_margin_m
        assert r['max_joint_speed_rad_s'] <= cfg.gait.max_joint_speed_rad_s
        assert r['max_spline_error_rad'] <= cfg.gait.spline_tolerance_rad
        assert 'quasi-static approximation' in r['labels']['static_margin']
        assert 'not an actuator rating' in r['labels']['joint_speed']


def test_phase_structure(cfg, temps):
    for t in temps:
        p = t.params
        assert [k for k, _ in t.phase_info] == ['shift', 'swing'] * 4 + ['shift']
        assert [lg for _, lg in t.phase_info if lg] == list(cr.ORDER_FORWARD)
        assert t.boundaries[0] == 0 and t.boundaries[-1] == len(t.points) - 1
        for d, (kind, _) in zip(t.report['phase_durations_s'], t.phase_info):
            assert d >= (p.min_shift_s if kind == 'shift' else p.min_swing_s) - 1e-9
            assert abs(d / p.point_dt_s - round(d / p.point_dt_s)) < 1e-9
        for i in t.boundaries:
            assert t.points[i]['velocities'] == [0.0] * 12                  # rest points
        assert t.boundary_body[0] == (0.0, 0.0)
        assert t.boundary_body[-1] == pytest.approx((0.0, p.stride_m))      # +y = forward


def test_cycle_starts_and_ends_in_the_cad_neutral_stance(temps):
    for t in temps:
        assert max(abs(q) for q in t.points[0]['positions']) < 1e-6
        assert max(abs(q) for q in t.points[-1]['positions']) < 1e-6


def test_joint_order_is_the_controller_order(temps):
    import yaml
    with open(os.path.join(fx.CONFIG_DIR, 'spiderx_ros2_controllers.yaml')) as f:
        ctrl = yaml.safe_load(f)
    joints = ctrl['leg_trajectory_controller']['ros__parameters']['joints']
    assert temps[0].joint_names == joints == fx.JOINTS


def test_boundaries_are_unambiguous_within_a_level_and_against_neutral(cfg, lib):
    tol = cfg.dispatch.continuity_tolerance_rad
    neutral = lib.boundary_positions(0, 0)
    close_across_levels = []
    for lv in range(len(lib.templates)):
        qs = [lib.boundary_positions(lv, b) for b in range(lib.n_phases)]
        for i in range(lib.n_phases):
            if i:
                assert max(abs(qs[i][n] - neutral[n]) for n in lib.joint_names) > 2 * tol
            for j in range(i + 1, lib.n_phases):
                assert max(abs(qs[i][n] - qs[j][n]) for n in lib.joint_names) > 2 * tol, \
                    (lv, i, j)
        for lv2 in range(lv + 1, len(lib.templates)):
            for b in range(1, lib.n_phases):
                q2 = lib.boundary_positions(lv2, b)
                d = max(abs(qs[b][n] - q2[n]) for n in lib.joint_names)
                if d <= 2 * tol:
                    close_across_levels.append((lv, lv2, b, d))
    # the same phase of two strides can be close (real templates: down to ~0.023 rad). Without
    # this process's rest record such a posture is refused; with it, the recorded one is used.
    assert close_across_levels, 'expected close cross-level boundaries in the real templates'
    for lv, lv2, b, d in close_across_levels:
        q = lib.boundary_positions(lv2, b)
        cands = lib.candidates(q, tol)
        assert (lv2, b) in [c[:2] for c in cands]
        level, bb, _ = loc.resolve_arm_posture(cands, None, False)
        assert level is None and bb == loc.POSTURE_UNVERIFIED
        assert loc.resolve_arm_posture(cands, (lv2, b), False)[:2] == (lv2, b)
        if d <= tol:
            assert loc.resolve_arm_posture(cands, (lv, b), False)[:2] == (lv, b)


def test_dispatched_goals_respect_the_joint_speed_bound_including_the_lead_in(cfg, lib):
    bound = cfg.gait.max_joint_speed_rad_s
    assert cfg.dispatch.continuity_tolerance_rad / cfg.dispatch.phase_lead_s <= bound
    for g in lib.goals.values():
        for a, b in zip(g.points, g.points[1:]):
            dt = b[0] - a[0]
            assert max(abs(y - x) for x, y in zip(a[1], b[1])) / dt <= bound
            assert max(abs(v) for v in b[2]) <= bound + 1e-9


def test_library_speeds(lib, temps):
    assert len(lib.goals) == len(temps) * 9 * 2 == len(lib.fingerprints)
    assert all(b > a for a, b in zip(lib.speeds_m_s, lib.speeds_m_s[1:]))
    for v, t in zip(lib.speeds_m_s, temps):
        assert v < t.report['speed_m_s']                   # the lead-in costs speed
    assert max(g.duration_s for g in lib.goals.values()) < 6.0


# ==================================================================== walking (fake plant)
def _world(cfg, lib):
    plant = fk.FakeLocomotionTransport(lib, lib.boundary_positions(0, 0))
    s = loc.LocomotionSession(cfg, lib, plant)
    return fk.FakeWorld(s, plant, dt=0.05), s, plant


@pytest.mark.parametrize('sign', [1, -1])
def test_one_full_cycle_each_way_with_the_real_library(cfg, lib, monkeypatch, sign):
    monkeypatch.setattr(c55, 'M55_LOCOMOTION_DISPATCH_ENABLED', True)        # tests only
    w, s, plant = _world(cfg, lib)
    w.run(1.0)
    assert w.request('arm')[0]
    w.cmd = (sign * lib.speeds_m_s[0], 0.0, 0.0)            # REP-103 forward / reverse
    assert w.run_until(lambda: plant.sent == 10, 60.0), (s.state, s.faults)
    w.cmd = (0.0, 0.0, 0.0)
    assert w.run_until(lambda: s.state == loc.READY, 10.0)
    assert s.faults == [] and s.cycles == sign and s.b == (1 if sign > 0 else 8)
    for a, b in zip(plant.goals, plant.goals[1:]):
        assert b.start == a.end
    # planned body motion: one stride along base +y (forward) or -y; sway returns to zero
    xy = [0.0, 0.0]
    for g in plant.goals[:9]:
        xy = [xy[0] + g.body_delta[0], xy[1] + g.body_delta[1]]
    assert xy == pytest.approx([0.0, sign * lib.strides_m[0]], abs=1e-6)


# ==================================================================== free-base feasibility
def test_support_analysis_explains_the_crawl_choice(designer):
    sup = fe.support_analysis(designer)
    three = sup['three_foot_margin_no_body_shift_m']
    assert set(three) == set(lk.ALL_LEGS)
    assert max(abs(m) for m in three.values()) < 0.01        # COM within 1 cm of an edge
    assert min(three.values()) < 0.005                       # far below the 15 mm requirement
    for diag in sup['trot_two_foot_support'].values():
        assert diag['static_margin_m'] is None and diag['support_area_m2'] == 0.0
    assert 'NOT shown' in sup['labels']['trot']


def test_feasibility_cli_writes_a_report(designer, tmp_path, capsys):
    rc = fe.main(['--strides', '0.02', '--out', str(tmp_path)], config_dir=fx.CONFIG_DIR,
                 designer=designer, utc='20261008T000000Z')
    out = capsys.readouterr().out
    assert rc == 0 and 'two-foot support' in out
    data = json.loads((tmp_path / '20261008T000000Z' / 'report.json').read_text())
    assert data['schema'] == fe.REPORT_SCHEMA and data['ok'] and data['dispatch_gate'] is False
    assert data['levels'][0]['stride_m'] == 0.02 and data['library']['goals'] == 18
    assert len(data['config_sha256']) == 64 and data['non_claims']
    assert 'not a continuous bound' in data['template_labels']['static_margin_method']
    assert data['levels'][0]['max_margin_step_between_samples_m'] > 0


def test_feasibility_cli_reports_an_infeasible_level(designer, capsys):
    rc = fe.main(['--strides', '0.02', '0.5', '--no-write'], config_dir=fx.CONFIG_DIR,
                 designer=designer)
    assert rc == 1 and 'FAILED' in capsys.readouterr().out


def test_feasibility_cli_usage_errors(capsys):
    assert fe.main(['--strides', '-0.02', '--no-write'], config_dir=fx.CONFIG_DIR) == 2
    assert fe.main(['--bogus'], config_dir=fx.CONFIG_DIR) == 2


# ==================================================================== approved trajectories
def test_m6d_one_goal_identity_is_unchanged():
    sources = m6t.load_sources(fx.CONFIG_DIR, load_urdf())
    traj = m6t.build_trajectory(sources)
    assert traj['trajectory_id'] == M6D_TRAJECTORY_ID
    assert gf.fingerprint(gf.approved_spec(traj, sources)) == M6D_FINGERPRINT


def test_m61_fixed_base_trot_is_preserved_as_its_own_test_case():
    plan = tc.load_plan(fx.CONFIG_DIR, load_urdf())
    assert plan.errors == ()
    traj = tc.build_trajectory(plan)
    assert traj['trajectory_id'] == M61_TRAJECTORY_ID
    assert traj['provenance']['content_sha256'] == M61_CONTENT_SHA256
    assert gf.fingerprint(goal61.approved_spec(traj, plan)) == M61_FINGERPRINT


def test_no_m55_goal_is_an_approved_m6_goal(lib):
    """The crawl library and the M6.0-D / M6.1 goals are disjoint: M5.5 never replays them."""
    m61 = tc.build_trajectory(tc.load_plan(fx.CONFIG_DIR, load_urdf()))
    trot = [tuple(round(q, 9) for q in p['positions']) for p in m61['points']]
    for g in lib.goals.values():
        assert not any(p[1] == q for p in g.points for q in trot[1:-1])


# ============================================================ review: claims the code relies on
def test_the_margin_report_states_its_method(cfg, temps):
    for t in temps:
        r = t.report
        assert 'not a continuous bound' in r['labels']['static_margin_method']
        assert 'WHOLE swing' in r['labels']['support'] and 'URDF inertials' in r['labels']['com']
        step = r['max_margin_step_between_samples_m']
        assert 0.0 < step < 0.002                     # samples are close in margin terms
        assert r['min_static_margin_m'] - step / 2 > cfg.gait.min_margin_m


def test_only_the_neutral_adjacent_shifts_come_near_the_neutral_stance(cfg, temps):
    """Supports resolve_arm_posture accepting neutral after an interruption: a posture within
    2 x tolerance of neutral can only be partway into S1 or S5 (four feet down), never a swing or
    another shift (checked at every 0.1 s waypoint)."""
    tol = cfg.dispatch.continuity_tolerance_rad
    for t in temps:
        n0 = t.points[0]['positions']
        last = len(t.phase_info) - 1
        assert t.phase_info[0][0] == t.phase_info[last][0] == 'shift'
        for k in range(1, last):
            for p in t.points[t.boundaries[k]:t.boundaries[k + 1] + 1]:
                assert max(abs(a - b) for a, b in zip(p['positions'], n0)) > 2 * tol, (k, p)
