"""Deterministic test double for the M6 FollowJointTrajectory action (no ROS graph, no server).

Records every goal, cancel and call so that tests can prove what was (not) dispatched.
"""

from spiderx_controller import m6_action_client as ac


class MockHandle:
    def __init__(self, accepted, index):
        self.accepted = accepted
        self.index = index


class MockActionServer(ac.ActionAdapter):
    """Scripted behaviour. result=None means 'no result before the watchdog'."""

    def __init__(self, ready=True, accept=True, respond=True,
                 result=(ac.STATUS_SUCCEEDED, 0, ''), samples=None, wait_raises=None,
                 send_raises=None, cancel_ok=True):
        self.ready = ready
        self.accept = accept
        self.respond = respond
        self.result = result
        self.samples = samples
        self.wait_raises = wait_raises
        self.send_raises = send_raises
        self.cancel_ok = cancel_ok
        self.sent = []
        self.cancels = []
        self.calls = []

    def server_ready(self, timeout_s):
        self.calls.append(('server_ready', timeout_s))
        return self.ready

    def send_goal(self, goal):
        self.calls.append(('send_goal',))
        self.sent.append(goal)
        if self.send_raises is not None:
            raise self.send_raises
        if not self.respond:
            return None
        return MockHandle(self.accept, len(self.sent))

    def wait_result(self, handle, timeout_s):
        self.calls.append(('wait_result', timeout_s))
        if self.wait_raises is not None:
            raise self.wait_raises
        return self.result

    def cancel(self, handle):
        self.calls.append(('cancel', handle.index))
        self.cancels.append(handle)
        return self.cancel_ok

    def joint_state_samples(self):
        self.calls.append(('joint_state_samples',))
        return list(self.samples or [])


def perfect_samples(trajectory, dt=0.25, settle_s=1.0, offset=0.0):
    """Synthetic /joint_states that follow the commanded reference (plus a constant offset)."""
    names = trajectory['joint_names']
    end = trajectory['points'][-1]['time_from_start_s'] + settle_s
    out, t = [], 0.0
    while t <= end + 1e-9:
        ref = ac.reference_positions(trajectory, t)
        if ref is None:
            ref = trajectory['points'][0]['positions']
        out.append((round(t, 6), {n: q + offset for n, q in zip(names, ref)}))
        t += dt
    return out
