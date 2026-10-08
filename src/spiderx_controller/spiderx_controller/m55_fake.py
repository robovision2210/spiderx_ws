"""M5.5 in-memory test doubles: a dispatching fake transport with a simulated joint plant, and a
deterministic world loop. Nothing here touches ROS.

FakeLocomotionTransport has mode 'dispatch', so a LocomotionSession accepts it ONLY while
m55_contract.M55_LOCOMOTION_DISPATCH_ENABLED is True; tests set that inside the test
(monkeypatch), never in the module. It verifies every goal against the library like the real
transport, follows it like the joint_trajectory_controller (a cubic lead-in from the current
state to the first point, then the cubic-Hermite spline through the waypoints), and reports
feedback, results and cancels. Scenarios inject the failures the session must handle.
"""

from spiderx_controller import m55_locomotion as loc

SCENARIOS = ('ok', 'reject', 'abort', 'tracking_drift', 'no_result', 'no_response',
             'cancel_ignored', 'send_raises')


def _cubic(a, b, s):
    s = min(max(s, 0.0), 1.0)
    h = 3 * s * s - 2 * s ** 3
    return a + (b - a) * h


def _hermite(p0, p1, t):
    """JTC-style cubic Hermite between two (t, q, v) points (per joint)."""
    t0, q0, v0 = p0
    t1, q1, v1 = p1
    dt = t1 - t0
    s = min(max((t - t0) / dt, 0.0), 1.0)
    s2, s3 = s * s, s * s * s
    h00, h10, h01, h11 = 2 * s3 - 3 * s2 + 1, s3 - 2 * s2 + s, -2 * s3 + 3 * s2, s3 - s2
    return [h00 * a + h10 * dt * va + h01 * b + h11 * dt * vb
            for a, va, b, vb in zip(q0, v0, q1, v1)]


class FakeLocomotionTransport:
    """Dispatching fake with a joint plant. `scenario` and `fail_phase` (the 1-based goal count
    at which the scenario's failure happens) select the injected behaviour."""

    mode = 'dispatch'

    def __init__(self, library, start_positions, scenario='ok', fail_phase=1, latency_s=0.05,
                 drift_rad_s=0.05, abort_after_s=0.5):
        if scenario not in SCENARIOS:
            raise ValueError(f'unknown scenario {scenario!r}')
        self.lib = library
        self.names = list(library.joint_names)
        self.q = [float(start_positions[n]) for n in self.names]
        self.scenario = scenario
        self.fail_phase = fail_phase
        self.latency_s = latency_s
        self.drift_rad_s = drift_rad_s
        self.abort_after_s = abort_after_s
        self.sent = 0
        self.goals = []                 # every verified goal, in order
        self.cancels = 0
        self.server_ready = True
        self._events = []
        self._active = None             # dict(seq, goal, t_accept, q_start, cancelled)
        self._pending = []              # (wall_due, event)
        self.closed = False

    # -------------------------------------------------------------- transport interface
    def ready(self):
        return self.server_ready

    def _failing(self):
        return self.sent == self.fail_phase

    def send(self, goal, wall):
        if self.scenario == 'send_raises' and self.sent + 1 == self.fail_phase:
            raise RuntimeError('fake transport: send failed')
        loc.verify_goal(goal, self.lib.fingerprints)
        if self._active is not None:
            raise loc.LocomotionError('fake: a goal is already active (one owner, one goal)')
        self.sent += 1
        seq = self.sent
        self.goals.append(goal)
        if self.scenario == 'no_response' and self._failing():
            return seq
        accepted = not (self.scenario == 'reject' and self._failing())
        self._pending.append((wall + self.latency_s, ('goal_response', seq, accepted)))
        if accepted:
            self._active = {'seq': seq, 'goal': goal, 't_accept': wall + self.latency_s,
                            'q_start': list(self.q), 'cancelled': False, 'done': False}
        return seq

    def cancel(self, wall):
        self.cancels += 1
        a = self._active
        if a is None:
            seq = self.sent
            self._pending.append((wall + self.latency_s, ('cancel_response', seq, -1)))
            return
        self._pending.append((wall + self.latency_s, ('cancel_response', a['seq'], 0)))
        a['cancelled'] = True
        if not (self.scenario == 'cancel_ignored'):
            self._pending.append((wall + 2 * self.latency_s,
                                  ('result', a['seq'], loc.STATUS_CANCELED, 0, 'canceled')))
            a['done'] = True

    def poll(self, wall):
        self._advance(wall)
        due = [e for t, e in self._pending if t <= wall]
        self._pending = [(t, e) for t, e in self._pending if t > wall]
        events, self._events = self._events + due, []
        return events

    def close(self):
        self.closed = True

    # -------------------------------------------------------------- the plant
    def _reference(self, a, wall):
        g = a['goal']
        te = wall - a['t_accept']
        pts = g.points
        if te <= pts[0][0]:
            s = te / pts[0][0]
            return [_cubic(q0, q1, s) for q0, q1 in zip(a['q_start'], pts[0][1])]
        for p0, p1 in zip(pts, pts[1:]):
            if te <= p1[0]:
                return _hermite(p0, p1, te)
        return list(pts[-1][1])

    def _advance(self, wall):
        a = self._active
        if a is None or wall < a['t_accept']:
            return
        if a['cancelled']:
            if a['done']:
                self._active = None
            return                                       # hold where it is
        failing = a['seq'] == self.fail_phase
        te = wall - a['t_accept']
        ref = self._reference(a, wall)
        err = 0.0
        if self.scenario == 'tracking_drift' and failing:
            err = self.drift_rad_s * te
        self.q = [r - err for r in ref]
        self._events.append(('feedback', a['seq'], err))
        g = a['goal']
        if self.scenario == 'abort' and failing and te >= self.abort_after_s:
            self._events.append(('result', a['seq'], loc.STATUS_ABORTED, -4, 'path tolerance'))
            self._active = None
            return
        if te >= g.duration_s:
            if self.scenario == 'no_result' and failing:
                return
            self._events.append(('result', a['seq'], loc.STATUS_SUCCEEDED, 0, ''))
            self._active = None

    def joint_positions(self):
        return dict(zip(self.names, self.q))


class FakeWorld:
    """Deterministic loop: plant joint states, graph status, body pose, operator commands and
    session ticks on one simulated monotonic clock (sim time = wall)."""

    def __init__(self, session, plant, dt=0.05, graph=None, pose=True, heartbeat_hz=10.0):
        self.s = session
        self.plant = plant              # anything with joint_positions() -> {name: q}
        self.dt = dt
        self.wall = 100.0
        self.graph = graph or {'joint_state_publishers': 1, 'command_topic_publishers': 0,
                               'foreign_action_clients': 0, 'action_server_present': True}
        self.pose = pose
        self.cmd = None                 # None: no commands at all; else (vx, vy, wz)
        self.heartbeat_dt = 1.0 / heartbeat_hz
        self._next_cmd = self.wall
        self.joint_states = True
        self.sim_advancing = True
        self.sim = 50.0
        self.states = []
        self.tilt = 0.0
        self.pose_codes = ()

    def tick(self):
        self.wall = round(self.wall + self.dt, 9)
        if self.sim_advancing:
            self.sim = round(self.sim + self.dt, 9)
        if self.joint_states:
            q = self.plant.joint_positions()
            self.s.on_joint_state(self.wall, self.sim, list(q), list(q.values()))
        if self.graph is not None:
            self.s.on_graph(self.wall, self.graph)
        if self.pose:
            self.s.on_body_pose(self.wall, self.pose_codes,
                                None if self.pose_codes else
                                {'xy': (0.0, 0.0), 'z': 0.1, 'yaw': 0.0, 'tilt': self.tilt})
        if self.cmd is not None and self.wall >= self._next_cmd - 1e-9:
            vx, vy, wz = self.cmd
            self.s.on_command(self.wall, (vx, vy, 0.0), (0.0, 0.0, wz))
            self._next_cmd = self.wall + self.heartbeat_dt
        self.s.step(self.wall)
        if not self.states or self.states[-1] != self.s.state:
            self.states.append(self.s.state)

    def run(self, seconds):
        for _ in range(int(round(seconds / self.dt))):
            self.tick()

    def run_until(self, predicate, timeout_s):
        for _ in range(int(round(timeout_s / self.dt))):
            self.tick()
            if predicate():
                return True
        return False

    def request(self, name):
        return self.s.request(name, self.wall)


class StaticPlant:
    """Joints that never move (shadow mode: the robot stands still)."""

    def __init__(self, positions):
        self.q = dict(positions)

    def joint_positions(self):
        return dict(self.q)


__all__ = ['FakeLocomotionTransport', 'FakeWorld', 'StaticPlant', 'SCENARIOS']
