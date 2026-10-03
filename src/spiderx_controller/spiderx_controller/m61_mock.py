"""Deterministic in-memory transport for M6.1 tests and the CLI --mock mode. No ROS, no graph.

M61FakeTransport extends the unchanged M6.0-D m6_live_mock.FakeTransport (scripted action server
and /joint_states stream on a fake wall clock) with:
  - the M6.1 reference (cubic Hermite with the waypoint velocities) for the simulated joints;
  - the result time derived from the M6.1 trajectory duration (7.0 s), not the M6.0-D 9 s;
  - a scripted ground-truth body-pose stream ('body_pose' events, latest_body_pose());
  - gait-gate scenarios: body too low, tilt, joint near its limit, body-pose loss, drift.
Every goal and cancel it receives is recorded, so tests can prove what was (not) dispatched.
"""

import math

from spiderx_controller import m6_live_mock as m6mock
from spiderx_controller import m61_trot_cycle as tc

NOMINAL_BODY_Z_M = 0.0545            # measured in M2 at CAD neutral


class M61FakeTransport(m6mock.FakeTransport):
    """Scripted M6.1 server + joint stream + body-pose stream; see the module docstring."""

    def __init__(self, trajectory, expected_fp, *, body_z=NOMINAL_BODY_Z_M, pose_rate=20.0,
                 no_pose=False, pose_stop_at=None, body_low_at=None, low_z=0.040, tilt_at=None,
                 tilt_rad=0.30, drift_at=None, drift_dx=0.03, joint_jump_at=None,
                 jump_joint='rf_foot_joint', jump_to=0.60, **kw):
        latency = kw.get('response_latency', 0.1)
        rtf = kw.get('rtf', 1.0)
        duration = trajectory['points'][-1]['time_from_start_s']
        if kw.get('result_at') is None and kw.get('result', 'success') is not None:
            # the fake clock does not advance before the send, so this is relative to the send
            kw['result_at'] = latency + duration / rtf + 0.05
        super().__init__(trajectory, expected_fp, **kw)
        self.body_z, self.pose_period = body_z, 1.0 / pose_rate
        self.no_pose, self.pose_stop_at = no_pose, pose_stop_at
        self.body_low_at, self.low_z = body_low_at, low_z
        self.tilt_at, self.tilt_rad = tilt_at, tilt_rad
        self.drift_at, self.drift_dx = drift_at, drift_dx
        self.joint_jump_at, self.jump_joint, self.jump_to = joint_jump_at, jump_joint, jump_to
        self.next_pose = self.t_start

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

    def latest_body_pose(self):
        """(wall, stamp, pose) of the most recent body-pose sample, or None if none ever came."""
        if self.no_pose:
            return None
        rel = self.t - self.t_start
        if self.pose_stop_at is not None:
            rel = min(rel, self.pose_stop_at - 1e-9)
        k = math.floor(rel / self.pose_period + 1e-9)
        wall = self.t_start + k * self.pose_period
        return (wall, self._stamp(wall), self._pose(wall))

    def poll(self, timeout_s):
        events = list(super().poll(timeout_s))          # advances the fake clock to self.t
        while self.next_pose <= self.t:
            if self._pose_flowing(self.next_pose - self.t_start):
                events.append(('body_pose', self.next_pose, self._stamp(self.next_pose),
                               self._pose(self.next_pose)))
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
    'body_drift_report_only': {'drift_at': 5.0},           # G7: reported, never a cancel
}

__all__ = ['M61FakeTransport', 'SCENARIOS', 'NOMINAL_BODY_Z_M']
