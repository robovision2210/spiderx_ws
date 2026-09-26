"""Comparison, outcome and report logic for the M3 SIMULATION-ONLY single-leg kinematics check.

Pure Python (no ROS graph): validate_leg_kinematics collects observations and this module decides.

Observation sources (all expressed in base_link):
  * TF: robot_state_publisher transform base_link -> <foot link>, looked up at the /joint_states
    stamp used for FK. Same URDF, independent implementation (KDL).
  * Gazebo: ground-truth link pose of <foot link> from /spiderx/sim/world_poses (SceneBroadcaster
    pose/info). Link poses there are relative to the model frame, which is dummy_link = base_link.
    The bridge leaves stamps at 0, so samples are taken only after the leg has settled.
The observed foot TIP is the observed link pose composed with the derived tip offset; only the link
pose itself is independently observed.
"""

import math

from spiderx_controller import leg_kinematics as lk

FK_VERIFIED = 'Single-leg FK verified against the current URDF/TF/Gazebo model.'
IK_VERIFIED = 'Single-leg IK verified for documented reachable, joint-safe simulation targets.'
NOT_VERIFIED = 'M3 kinematics validation not verified.'
DISCLAIMER = [
    'Single-leg simulation kinematics validation (Gazebo Fortress).',
    'Not walking, not a gait, not balance control, not locomotion.',
    'Not real-world leg control and not hardware validation.',
]
LIMITATIONS = [
    'Placeholder actuator effort/velocity (100 N m, 100 rad/s): joints are stiff and track almost '
    'perfectly in simulation.',
    'Gazebo and TF use the same URDF: agreement validates the kinematics implementation, not the '
    'physical dimensions of the real robot.',
    'The foot tip is a derived point (lowest point of the foot collision mesh at q = 0), not a '
    'measured contact point; the real contact point on the rounded pad moves as the foot rotates.',
    'Contact, friction (mu = 0.2), damping (none), masses (steel density) and the physics step '
    '(DART, 1 ms) are simulation assumptions.',
    'One leg (front_left) only; other legs are held at their measured positions.',
    'No real actuator, servo backlash, frame flex, battery, electronics or sensor validation.',
]


def position_error(a, b):
    """a - b per axis and Euclidean (m)."""
    d = [x - y for x, y in zip(a, b)]
    return {'dx': d[0], 'dy': d[1], 'dz': d[2], 'euclidean': math.sqrt(sum(v * v for v in d))}


def compare_pose(fk, obs_position, obs_quaternion, tip_local, pos_tol, ang_tol, source):
    """Compare FK (dict from leg_kinematics.forward) with an observed foot-link pose.

    Returns a record with link-origin error, orientation error, tip error, passed, failures.
    """
    if obs_position is None or obs_quaternion is None:
        return {'source': source, 'passed': False,
                'failures': [f'{source}: no observation of the foot link']}
    R_obs = lk.matrix_from_quaternion(*obs_quaternion)
    link_err = position_error(fk['foot_link_position'], obs_position)
    ang_err = lk.rotation_angle_between(fk['foot_link_rotation'], R_obs)
    obs_tip = tuple(obs_position[i] + sum(R_obs[i][k] * tip_local[k] for k in range(3))
                    for i in range(3))
    tip_err = position_error(fk['tip_position'], obs_tip)
    failures = []
    if link_err['euclidean'] > pos_tol:
        failures.append(f'{source}: foot-link origin error {link_err["euclidean"]:.3e} m > '
                        f'{pos_tol:.1e} m')
    if ang_err > ang_tol:
        failures.append(f'{source}: foot-link orientation error {ang_err:.3e} rad > '
                        f'{ang_tol:.1e} rad')
    if tip_err['euclidean'] > pos_tol:
        failures.append(f'{source}: foot-tip error {tip_err["euclidean"]:.3e} m > {pos_tol:.1e} m')
    return {'source': source, 'observed_link_position_m': list(obs_position),
            'observed_link_quaternion_xyzw': list(obs_quaternion),
            'observed_tip_m': list(obs_tip), 'link_origin_error_m': link_err,
            'orientation_error_rad': ang_err, 'tip_error_m': tip_err,
            'position_tolerance_m': pos_tol, 'orientation_tolerance_rad': ang_tol,
            'passed': not failures, 'failures': failures}


def fk_check(geom, q, tf_obs, gz_obs, tol, label):
    """One FK check at joint vector q against TF and Gazebo observations (pos, quat) or None."""
    fk = lk.forward(geom, q)
    tf_rec = compare_pose(fk, *(tf_obs or (None, None)), geom.tip_local,
                          tol['fk_vs_tf_position_m'], tol['fk_vs_tf_orientation_rad'], 'TF')
    gz_rec = compare_pose(fk, *(gz_obs or (None, None)), geom.tip_local,
                          tol['fk_vs_gazebo_position_m'], tol['fk_vs_gazebo_orientation_rad'],
                          'Gazebo')
    return {'label': label, 'q_measured_rad': list(q),
            'fk_foot_link_position_m': list(fk['foot_link_position']),
            'fk_foot_link_quaternion_xyzw': list(fk['foot_link_quaternion']),
            'fk_tip_m': list(fk['tip_position']), 'vs_tf': tf_rec, 'vs_gazebo': gz_rec,
            'passed': tf_rec['passed'] and gz_rec['passed'],
            'failures': tf_rec['failures'] + gz_rec['failures']}


def ik_target_check(geom, name, target, ik, q_measured, gz_obs, fk_rec, action, returned, tol):
    """Decide one safe-target test. gz_obs = observed (pos, quat) of the foot link after settling."""
    failures = []
    rec = {'name': name, 'target_m': list(target), 'ik_reason': ik['reason'],
           'q_commanded_rad': ik['solution'], 'q_measured_rad': list(q_measured or []),
           'action': action, 'returned_to_start': returned, 'fk_check': fk_rec}
    if not ik['ok']:
        failures.append(f'{name}: IK failed ({ik["reason"]})')
    else:
        rec['fk_of_commanded_residual_m'] = position_error(
            lk.forward(geom, ik['solution'])['tip_position'], target)['euclidean']
        if rec['fk_of_commanded_residual_m'] > tol['ik_position_tol_m']:
            failures.append(f'{name}: FK(IK) residual {rec["fk_of_commanded_residual_m"]:.2e} m')
    if action.get('error_code') != 0:
        failures.append(f'{name}: trajectory result {action}')
    if gz_obs is None:
        failures.append(f'{name}: no Gazebo observation after the move')
    else:
        R = lk.matrix_from_quaternion(*gz_obs[1])
        tip = tuple(gz_obs[0][i] + sum(R[i][k] * geom.tip_local[k] for k in range(3))
                    for i in range(3))
        rec['observed_tip_m'] = list(tip)
        rec['target_error_m'] = position_error(tip, target)
        if rec['target_error_m']['euclidean'] > tol['target_reached_position_m']:
            failures.append(f'{name}: observed tip {rec["target_error_m"]["euclidean"] * 1000:.3f}'
                            f' mm from target > {tol["target_reached_position_m"] * 1000:.1f} mm')
    if fk_rec is None or not fk_rec['passed']:
        failures += (fk_rec or {}).get('failures', [f'{name}: FK check missing'])
    if not returned:
        failures.append(f'{name}: leg did not return to the start configuration')
    rec.update(passed=not failures, failures=failures)
    return rec


def negative_check(name, expect_reasons, ik, goals_before, goals_after, max_joint_motion,
                   motion_tol=1e-3):
    failures = []
    if ik['ok']:
        failures.append(f'{name}: IK accepted a negative-test target')
    elif ik['reason'] not in expect_reasons:
        failures.append(f'{name}: rejected as {ik["reason"]}, expected one of {expect_reasons}')
    if goals_after != goals_before:
        failures.append(f'{name}: {goals_after - goals_before} trajectory goal(s) were sent')
    if max_joint_motion is None or max_joint_motion > motion_tol:
        failures.append(f'{name}: joints moved {max_joint_motion} rad during the rejection test')
    return {'name': name, 'ik_reason': ik['reason'], 'ik_message': ik.get('message'),
            'goals_sent': goals_after - goals_before, 'max_joint_motion_rad': max_joint_motion,
            'passed': not failures, 'failures': failures}


def outcome(run):
    """(fk_verified, ik_verified, outcome_lines, failures) from the collected run records."""
    failures = list(run.get('precondition_failures', []))
    fk_checks = run.get('fk_checks', [])
    fk_ok = bool(fk_checks) and all(c['passed'] for c in fk_checks) and not failures
    for c in fk_checks:
        failures += c['failures']
    ik_recs, neg = run.get('ik_targets', []), run.get('negative_tests', [])
    ik_ok = (fk_ok and bool(ik_recs) and all(r['passed'] for r in ik_recs)
             and len(ik_recs) == run.get('expected_safe_targets', len(ik_recs))
             and bool(neg) and all(n['passed'] for n in neg)
             and run.get('final_return_ok', False))
    for r in ik_recs + neg:
        failures += r['failures']
    if fk_ok and ik_recs and len(ik_recs) != run.get('expected_safe_targets', len(ik_recs)):
        failures.append('not every safe target was tested')
    if fk_ok and not run.get('final_return_ok', False):
        failures.append('leg did not return to the start configuration at the end')
    lines = []
    if fk_ok:
        lines.append(FK_VERIFIED)
    if ik_ok:
        lines.append(IK_VERIFIED)
    if not (fk_ok and ik_ok):
        lines.append(NOT_VERIFIED)
    return fk_ok, ik_ok, lines, failures


def build_report(geom, cfg, run):
    fk_ok, ik_ok, lines, failures = outcome(run)
    return {
        'simulation_only': True,
        'disclaimer': DISCLAIMER,
        'outcome': lines,
        'fk_verified': fk_ok,
        'ik_verified': ik_ok,
        'passed': fk_ok and ik_ok,
        'failures': failures,
        'leg': geom.leg,
        'frame': lk.FRAME,
        'geometry': geom.summary(),
        'config_version': cfg['config_version'],
        'config_date': cfg['date'],
        'tolerances': cfg['tolerances'],
        'safety': cfg['safety'],
        'validation_margin_rad': cfg['margin'],
        'reference_tip_m': list(geom.tip0),
        'controller_checks': run.get('controller_checks', []),
        'joint_state_publishers': run.get('joint_state_publishers'),
        'start_positions_rad': run.get('start_positions'),
        'fk_checks': run.get('fk_checks', []),
        'negative_tests': run.get('negative_tests', []),
        'ik_targets': run.get('ik_targets', []),
        'body_tilt_rad_max': run.get('body_tilt_rad_max'),
        'goals_sent': run.get('goals_sent'),
        'final_return_ok': run.get('final_return_ok'),
        'limitations': LIMITATIONS,
    }
