"""M6.1 goal contract and fingerprint: binds the ONE approved trot-cycle goal to a hash.

Re-uses the M6.0-D fingerprint machinery unchanged (m6_goal_fingerprint: binding, spec_from_goal,
fingerprint, verify_goal), so the M6.0-D adapter's fingerprint gate (RclpyLiveTransport.send_goal)
applies to M6.1 exactly as it does to M6.0-D: a goal whose recomputed fingerprint differs from
the approved one - a return-to-neutral, a second cycle, any edit - is refused before any ROS call.

The approved goal: header stamp 0, the 12 canonical joints, the 9 preflighted points with their
positions and velocities (no accelerations, so the controller interpolates cubic splines),
path tolerance 0.05 rad, goal tolerance 0.05 rad and 0.05 rad/s, goal_time_tolerance 1.0 s (all
from m61_limits.yaml), no multi-DOF and no component tolerances.

ROS message types are imported only inside build_goal_message().
"""

from spiderx_controller import m6_action_client as ac
from spiderx_controller import m6_goal_fingerprint as gf
from spiderx_controller import m61_trot_cycle as tc


def _ns(seconds):
    sec, nsec = ac.seconds_to_duration_fields(seconds)
    return int(sec) * 1_000_000_000 + int(nsec)


def _tol(names, position, velocity):
    return [{'name': n, 'position': position, 'velocity': velocity, 'acceleration': 0.0}
            for n in names]


def _require_preflight(trajectory, plan):
    report = tc.preflight(trajectory, plan)
    if not report.ok:
        raise gf.FingerprintError('preflight_refused', ', '.join(report.codes))
    return report


def approved_spec(trajectory, plan):
    """The spec of the one approved M6.1 goal (preflight must pass). Raises FingerprintError."""
    _require_preflight(trajectory, plan)
    lim = plan.limits
    names = list(trajectory['joint_names'])
    spec = {
        'schema': gf.SPEC_SCHEMA,
        'binding': gf.binding(trajectory),
        'header_stamp_ns': 0,
        'joint_names': names,
        'points': [{'time_ns': _ns(p['time_from_start_s']),
                    'positions': [float(v) for v in p['positions']],
                    'velocities': [float(v) for v in p['velocities']],
                    'accelerations': [], 'effort': []} for p in trajectory['points']],
        'path_tolerance': _tol(names, lim.path_position_tolerance_rad, 0.0),
        'goal_tolerance': _tol(names, lim.goal_position_tolerance_rad,
                               lim.goal_velocity_tolerance_rad_s),
        'goal_time_tolerance_ns': _ns(lim.goal_time_tolerance_s),
        'component_path_tolerance': 0,
        'component_goal_tolerance': 0,
        'multi_dof_joint_names': [],
        'multi_dof_points': 0,
    }
    gf.validate_spec(spec)
    return spec


def build_goal_message(trajectory, plan):
    """FollowJointTrajectory.Goal for the preflighted M6.1 trajectory. Raises FingerprintError."""
    report = _require_preflight(trajectory, plan)
    from builtin_interfaces.msg import Duration, Time
    from control_msgs.action import FollowJointTrajectory
    from control_msgs.msg import JointTolerance
    from trajectory_msgs.msg import JointTrajectoryPoint

    lim = plan.limits
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.header.stamp = Time(sec=0, nanosec=0)            # start on receipt
    goal.trajectory.joint_names = list(trajectory['joint_names'])
    for p in trajectory['points']:
        sec, nsec = ac.seconds_to_duration_fields(p['time_from_start_s'])
        goal.trajectory.points.append(JointTrajectoryPoint(
            positions=[float(v) for v in p['positions']],
            velocities=[float(v) for v in p['velocities']],
            time_from_start=Duration(sec=sec, nanosec=nsec)))
    names = goal.trajectory.joint_names
    goal.path_tolerance = [JointTolerance(**t) for t in
                           _tol(names, lim.path_position_tolerance_rad, 0.0)]
    goal.goal_tolerance = [JointTolerance(**t) for t in
                           _tol(names, lim.goal_position_tolerance_rad,
                                lim.goal_velocity_tolerance_rad_s)]
    sec, nsec = ac.seconds_to_duration_fields(lim.goal_time_tolerance_s)
    goal.goal_time_tolerance = Duration(sec=sec, nanosec=nsec)
    return goal, report


def build_live_goal(trajectory, plan):
    """(goal, report, spec, fingerprint) for the one approved M6.1 goal. Raises on any mismatch."""
    expected = approved_spec(trajectory, plan)
    goal, report = build_goal_message(trajectory, plan)
    fp = gf.fingerprint(expected)
    gf.verify_goal(goal, gf.binding(trajectory), fp)
    return goal, report, expected, fp


__all__ = ['approved_spec', 'build_goal_message', 'build_live_goal']
