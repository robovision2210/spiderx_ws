"""Deterministic in-memory transport for M6.1 tests and the CLI --mock mode. No ROS, no graph.

M61FakeTransport extends the unchanged M6.0-D m6_live_mock.FakeTransport (scripted action server
and /joint_states stream on a fake wall clock) with:
  - the M6.1 reference (cubic Hermite with the waypoint velocities) for the simulated joints;
  - the result time derived from the M6.1 trajectory duration (7.0 s), not the M6.0-D 9 s;
  - a scripted ground-truth body-pose stream ('body_pose' events, latest_body_pose());
  - gait-gate scenarios: body too low, tilt, joint near its limit, body-pose loss, drift;
  - streams_now() for the M6.1 pre-send check (the scripted streams at the current fake time)
    and server_wait_s, a server wait during which fake time passes and nothing is delivered (as
    rclpy wait_for_server, which does not spin): streams that stop, or a world that pauses,
    after readiness and before the send.
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
                 competing_after_calls=None, server_wait_s=0.0, **kw):
        latency = kw.get('response_latency', 0.1)
        rtf = kw.get('rtf', 1.0)
        duration = trajectory['points'][-1]['time_from_start_s']
        self._result_after_send = None
        if kw.get('result_at') is None and kw.get('result', 'success') is not None:
            # relative to the send; send_goal re-anchors it if fake time passed before the send
            self._result_after_send = latency + duration / rtf + 0.05
            kw['result_at'] = self._result_after_send
        super().__init__(trajectory, expected_fp, **kw)
        self.server_wait_s = server_wait_s
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

    # ---------------------------------------------------------------- server wait and send
    def server_ready(self, timeout_s):
        """As rclpy wait_for_server: time passes (server_wait_s) and nothing is delivered."""
        ok = super().server_ready(timeout_s)
        self.t += min(self.server_wait_s, timeout_s)
        return ok

    def send_goal(self, goal, binding):
        if self._result_after_send is not None:
            self.result_at = self.t - self.t_start + self._result_after_send
        return super().send_goal(goal, binding)

    # ---------------------------------------------------------------- streams now (pre-send)
    def _js_skipped(self, rel):
        return (self.js_stop_at is not None and rel >= self.js_stop_at) or (
            self.gap_at is not None and self.gap_at <= rel < self.gap_at + 0.4)

    def _latest_joint_states(self, horizon_s=60.0):
        """(wall, {name: position}) of the latest scripted joint state at or before now."""
        k = math.floor((self.t - self.t_start) / self.js_period + 1e-9)
        for _ in range(int(horizon_s / self.js_period)):
            w = self.t_start + k * self.js_period
            if not self._js_skipped(w - self.t_start):
                return w, dict(zip(self.traj['joint_names'], self._positions(w)))
            k -= 1
        return None

    def streams_now(self, settle_s=0.0):
        """The scripted streams at the current fake time, for the pre-send check: the latest
        joint state and the tracker snapshot. The fake clock is not advanced; what a live spin
        would have received before now is consumed and dropped (as the live transport does), so
        tracking still starts at the dispatch. A scripted timeline has no backlog, so there is
        no since_wall: the readiness window ending now applies. Read-only."""
        out = {'now': self.t, 'joint_states': self._latest_joint_states(),
               'tracker': self.fixed_base_snapshot()}
        while self.next_js < self.t:
            self.next_js += self.js_period
        while self.next_pose < self.t:
            self.next_pose += self.pose_period
        return out

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
        """Readiness may look before poll() delivered the samples; feed the latest one. The first
        time, also the pose ticks of the preceding progress window: the world was already running
        (or paused) when this transport opened, which a live readiness observation also sees."""
        smp = self.tracker.latest_usable
        if smp is None:
            n = math.ceil(self.fixed_base.progress_window_s / self.pose_period - 1e-9)
            for k in range(n, 0, -1):
                w = wall - k * self.pose_period
                self.tracker.on_transforms(self._transforms(w), w, self._stamp(w))
            smp = self.tracker.latest_usable
        elif smp.wall < wall - self.pose_period:
            # fake time passed without poll() (a server wait): the window's ticks since then
            first = max(smp.wall, wall - self.fixed_base.progress_window_s)
            k = math.floor((first - self.t_start) / self.pose_period + 1e-9) + 1
            while self.t_start + k * self.pose_period < wall - 1e-9:
                w = self.t_start + k * self.pose_period
                self.tracker.on_transforms(self._transforms(w), w, self._stamp(w))
                k += 1
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
    'sim_paused': {'stamp_freeze_at': -10.0},              # paused before readiness: refused
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
    # Pre-send check (plan E5): a 2 s server wait after readiness, then the send.
    'slow_server': {'server_wait_s': 2.0},                       # streams current: sent
    'joint_states_stop_before_send': {'server_wait_s': 2.0, 'js_stop_at': 1.0},   # refused
    'body_pose_stops_before_send': {'server_wait_s': 2.0, 'pose_stop_at': 0.5},   # refused
    'sim_pauses_before_send': {'server_wait_s': 2.0, 'stamp_freeze_at': 0.5},     # refused
}
# Fixed-base scenarios need the fixed-base mode; G7 report-only drift is a free-base behaviour
# (on a weld, any drift G7 could flag is first an attachment failure, G8).
FREE_BASE_SCENARIOS = {'body_drift_report_only': {'drift_at': 5.0}}

__all__ = ['M61FakeTransport', 'SCENARIOS', 'FREE_BASE_SCENARIOS', 'NOMINAL_BODY_Z_M']
