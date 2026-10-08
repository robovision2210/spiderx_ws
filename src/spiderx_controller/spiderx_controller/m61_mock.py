"""Deterministic in-memory transport for M6.1 tests and the CLI --mock mode. No ROS, no graph.

M61FakeTransport extends the unchanged M6.0-D m6_live_mock.FakeTransport (scripted action server
and /joint_states stream on a fake wall clock) with:
  - the M6.1 reference (cubic Hermite with the waypoint velocities) for the simulated joints;
  - the result time derived from the M6.1 trajectory duration (7.0 s), not the M6.0-D 9 s;
  - a scripted ground-truth body-pose stream ('body_pose' events, latest_body_pose());
  - gait-gate scenarios: body too low, tilt, joint near its limit, body-pose loss, drift.
Every goal and cancel it receives is recorded, so tests can prove what was (not) dispatched.

M6.1-A fixed-base mode (fixed_base=<FixedBaseConfig>, what the CLI --mock uses): the mock plays
the WELDED plant. It publishes a /robot_description-like weld (the approved mount unless a
scenario changes it) and, per pose tick, Gazebo-like TFMessage entries - the model root
'spiderx' (world frame) and the body link 'dummy_link' (relative to the model) - which go
through the SAME m61a_fixed_base.FixedBasePoseTracker as the live transport. The scripted body
pose (height drop, tilt, drift) is then the PHYSICAL body: the model root moves with it, so a
displaced body is an attachment failure (G8) as it would be on a real weld.
"""

import math

from spiderx_controller import m6_live_mock as m6mock
from spiderx_controller import m61_trot_cycle as tc
from spiderx_controller import m61a_fixed_base as fb

NOMINAL_BODY_Z_M = 0.0545            # measured in M2 at CAD neutral


class M61FakeTransport(m6mock.FakeTransport):
    """Scripted M6.1 server + joint stream + body-pose stream; see the module docstring."""

    def __init__(self, trajectory, expected_fp, *, body_z=None, pose_rate=20.0,
                 no_pose=False, pose_stop_at=None, body_low_at=None, low_z=0.040, tilt_at=None,
                 tilt_rad=0.30, drift_at=None, drift_dx=0.03, joint_jump_at=None,
                 jump_joint='rf_foot_joint', jump_to=0.60, fixed_base=None,
                 description='fixed_base', spawn_offset=None, link_offset=None,
                 mount_override=None, command_publishers=0, foreign_clients=0,
                 competing_after_calls=None, **kw):
        latency = kw.get('response_latency', 0.1)
        rtf = kw.get('rtf', 1.0)
        duration = trajectory['points'][-1]['time_from_start_s']
        if kw.get('result_at') is None and kw.get('result', 'success') is not None:
            # the fake clock does not advance before the send, so this is relative to the send
            kw['result_at'] = latency + duration / rtf + 0.05
        super().__init__(trajectory, expected_fp, **kw)
        self.fixed_base = fixed_base
        self.tracker = None
        self.spawn_offset = spawn_offset          # model root offset (a non-identity spawn)
        self.link_offset = link_offset            # Gazebo body link != description (z offset)
        if fixed_base is not None:
            mount = mount_override or fixed_base.mount
            if body_z is None:
                body_z = mount.p[2]
            self.tracker = fb.FixedBasePoseTracker(fixed_base)
            if description is not None:
                self.tracker.on_description(
                    fb.minimal_description(mount, fixed_base=(description == 'fixed_base')),
                    self.t_start)
            self.weld = fb.parse_robot_description(fb.minimal_description(mount))
        elif body_z is None:
            body_z = NOMINAL_BODY_Z_M
        self.body_z, self.pose_period = body_z, 1.0 / pose_rate
        self.no_pose, self.pose_stop_at = no_pose, pose_stop_at
        self.body_low_at, self.low_z = body_low_at, low_z
        self.tilt_at, self.tilt_rad = tilt_at, tilt_rad
        self.drift_at, self.drift_dx = drift_at, drift_dx
        self.joint_jump_at, self.jump_joint, self.jump_to = joint_jump_at, jump_joint, jump_to
        self.next_pose = self.t_start
        # other commanders visible in the graph; competing_after_calls = a client that appears
        # after that many snapshots (late arrival between readiness observations)
        self.command_publishers, self.foreign_clients = command_publishers, foreign_clients
        self.competing_after_calls = competing_after_calls
        self.owner_snapshots = 0

    def command_owner_snapshot(self):
        self.owner_snapshots += 1
        late = (self.competing_after_calls is not None
                and self.owner_snapshots > self.competing_after_calls)
        return {'command_publishers': self.command_publishers,
                'foreign_action_clients': self.foreign_clients + (1 if late else 0)}

    # ---------------------------------------------------------------- joints (M6.1 reference)
    def _positions(self, wall):
        stamp = self._stamp(wall)
        if self.held_at is not None:
            stamp = self._stamp(self.held_at)
        ref = None
        if self.ctrl_start_sim is not None:
            ref = tc.reference_positions(self.traj, stamp - self.ctrl_start_sim)
        if ref is None:
            ref = list(self.traj['points'][0]['positions'])
        off = self.js_offset
        if self.error_after is not None and self.ctrl_start_sim is not None and \
                stamp - self.ctrl_start_sim >= self.error_after:
            off += self.error_value
        q = [v + off for v in ref]
        if self.joint_jump_at is not None and wall - self.t_start >= self.joint_jump_at:
            q[list(self.traj['joint_names']).index(self.jump_joint)] = self.jump_to
        return q

    # ---------------------------------------------------------------- body pose
    def _pose(self, wall):
        rel = wall - self.t_start
        x, z, roll = 0.0, self.body_z, 0.0
        if self.body_low_at is not None and rel >= self.body_low_at:
            z = self.low_z
        if self.tilt_at is not None and rel >= self.tilt_at:
            roll = self.tilt_rad
        if self.drift_at is not None and rel >= self.drift_at:
            x = self.drift_dx
        return (x, 0.0, z, roll, 0.0, 0.0)

    def _pose_flowing(self, rel):
        return not self.no_pose and (self.pose_stop_at is None or rel < self.pose_stop_at)

    def _transforms(self, wall):
        """Gazebo-like entries for the scripted PHYSICAL body pose (fixed-base mode)."""
        x, y, z, roll, pitch, yaw = self._pose(wall)
        body = fb.Pose.from_xyz_rpy(x, y, z, roll, pitch, yaw)
        link = self.weld.model_to_dummy
        if self.link_offset is not None:
            link = fb.Pose.from_xyz_rpy(z=self.link_offset) * link
        model = body * self.weld.dummy_to_body.inverse() * link.inverse()
        if self.spawn_offset is not None:
            model = fb.Pose.from_xyz_rpy(z=self.spawn_offset) * model
        return [fb.make_transform('ground_plane', fb.Pose()),
                fb.make_transform(self.fixed_base.model_name, model),
                fb.make_transform(self.fixed_base.body_link, link, self.fixed_base.model_name)]

    def _latest_pose_wall(self):
        rel = self.t - self.t_start
        if self.pose_stop_at is not None:
            rel = min(rel, self.pose_stop_at - 1e-9)
        k = math.floor(rel / self.pose_period + 1e-9)
        return self.t_start + k * self.pose_period

    def latest_body_pose(self):
        """(wall, stamp, pose) of the most recent body-pose sample, or None if none ever came."""
        if self.no_pose:
            return None
        if self.tracker is not None:
            self._feed_tracker_until(self._latest_pose_wall())
            return self.tracker.latest_body_pose()
        wall = self._latest_pose_wall()
        return (wall, self._stamp(wall), self._pose(wall))

    def fixed_base_snapshot(self):
        """The tracker snapshot (description, latest usable pose) or {} without a fixed base."""
        if self.tracker is None:
            return {}
        if not self.no_pose:
            self._feed_tracker_until(self._latest_pose_wall())
        return self.tracker.snapshot()

    def _feed_tracker_until(self, wall):
        """Readiness may look before poll() delivered the samples; feed the latest one."""
        smp = self.tracker.latest_usable
        if smp is None or smp.wall < wall:
            self.tracker.on_transforms(self._transforms(wall), wall, self._stamp(wall))

    def poll(self, timeout_s):
        events = list(super().poll(timeout_s))          # advances the fake clock to self.t
        while self.next_pose <= self.t:
            if self._pose_flowing(self.next_pose - self.t_start):
                w = self.next_pose
                if self.tracker is not None:
                    events.extend(self.tracker.on_transforms(self._transforms(w), w,
                                                             self._stamp(w)))
                else:
                    events.append(('body_pose', w, self._stamp(w), self._pose(w)))
            self.next_pose += self.pose_period
        return events


SCENARIOS = {
    'success': {},
    'interrupt': {'interrupts_at': (5.0,)},
    'tracking_error': {'error_after': 4.0, 'error_value': 0.06},
    'stale_joint_states': {'js_stop_at': 5.0},
    'controller_lost': {'server_lost_at': 5.0},
    'rejected': {'accept': False},
    'body_too_low': {'body_low_at': 5.0},                  # G1
    'body_tilt': {'tilt_at': 5.0},                         # G2
    'joint_near_limit': {'joint_jump_at': 5.0},            # G3
    'sim_stall': {'stamp_freeze_at': 1.0},                 # G5 (re-used M6.0-D monitor)
    'joint_state_gap': {'gap_at': 5.0},                    # G6 (re-used M6.0-D monitor)
    'body_pose_stale': {'pose_stop_at': 5.0},              # pose freshness while running
    'no_body_pose': {'no_pose': True},                     # readiness refuses; nothing sent
    'attachment_drift': {'drift_at': 5.0},                 # G8: a weld that lets the body slide
    'not_fixed_base': {'description': 'free_base'},        # readiness refuses; nothing sent
    'spawn_offset': {'spawn_offset': 0.075},               # readiness refuses (double counting)
    'description_mismatch': {'link_offset': -0.05},        # Gazebo body link != description
    'mount_not_approved': {'mount_override': fb.Pose.from_xyz_rpy(z=0.15)},   # refused
    'competing_publisher': {'command_publishers': 1},     # readiness refuses; nothing sent
    'competing_client': {'foreign_clients': 1},           # readiness refuses; nothing sent
    'competing_client_late': {'competing_after_calls': 1},  # appears before the final check
}
# Fixed-base scenarios need the fixed-base mode; G7 report-only drift is a free-base behaviour
# (on a weld, any drift G7 could flag is first an attachment failure, G8).
FREE_BASE_SCENARIOS = {'body_drift_report_only': {'drift_at': 5.0}}

__all__ = ['M61FakeTransport', 'SCENARIOS', 'FREE_BASE_SCENARIOS', 'NOMINAL_BODY_Z_M']
