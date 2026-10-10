"""M6.1-A rclpy transport: the M6.1 transport whose body pose is the COMPOSED fixed-base body.

M61AFixedBaseTransport = m61_live_adapter.M61RclpyLiveTransport (itself the unchanged M6.0-D
transport + one ground-truth pose subscription) with:
  - ONE extra read-only subscription: /robot_description (std_msgs/String, transient local, as
    robot_state_publisher publishes it), parsed by m61a_fixed_base for the weld;
  - the pose callback replaced by m61a_fixed_base.FixedBasePoseTracker: the body pose handed to
    the M6.1 gates is T_world_model * T_model_dummy(Gazebo entry) * T_dummy_base, and every sample
    also yields a ('fixed_base', ...) event for G8;
  - fixed_base_snapshot() for the M6.1-A readiness check (m6_gait_replay.with_fixed_base);
  - streams_now(): immediately before the send, a drain of what was queued, then a 1 s spin of
    its own executor, so the M6.1 pre-send check (m61a_fixed_base.streams_at_send) sees what
    arrives NOW (joint states, body pose, sim progress), not a readiness result that may be up
    to 10 s old, and not a backlog that only looks fresh.
It publishes nothing, creates no action client beyond the M6.0-D one, and sends no goal of its
own. Importing this module starts nothing. Live use stays HARD-DISABLED by the M6.1 CLI
(m61_live_contract.M61_LIVE_DISPATCH_ENABLED = False).
"""

import time

from spiderx_controller import m61_live_adapter as la
from spiderx_controller import m61a_fixed_base as fb

DESCRIPTION_TOPIC = '/robot_description'
EVENT_KINDS_DROPPED_AFTER_COLLECT = ('body_pose', 'fixed_base')
STREAM_EVENT_KINDS = ('joint_state',) + EVENT_KINDS_DROPPED_AFTER_COLLECT
DRAIN_MAX_CALLBACKS = 500    # > every subscription's queue depth together, plus new arrivals


def action_status_topic(action=None):
    """The status topic every FollowJointTrajectory client subscribes to (one per client)."""
    from spiderx_controller import m6_envelope as env
    return (action or env.ACTION_NAME).rstrip('/') + '/_action/status'


def description_qos():
    """robot_state_publisher's /robot_description: reliable, transient local, depth 1."""
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class M61AFixedBaseTransport(la.M61RclpyLiveTransport):
    """M6.1 transport for the welded plant. Construct, then open(); always close()."""

    def __init__(self, expected_fingerprint, domain_id, fixed_base, **kw):
        kw.setdefault('pose_topic', fixed_base.pose_topic)
        kw.setdefault('model_name', fixed_base.model_name)
        super().__init__(expected_fingerprint, domain_id, **kw)
        self.cfg = fixed_base
        self.tracker = fb.FixedBasePoseTracker(fixed_base)
        self.description_sub = None
        self._latest_joint_states = None          # (wall, {name: position})

    def open(self):
        super().open()
        try:
            from std_msgs.msg import String
            self.description_sub = self.node.create_subscription(
                String, DESCRIPTION_TOPIC, self._on_description, description_qos())
        except BaseException:
            try:
                self.close()
            except Exception:  # noqa: BLE001 - the original error is the one to report
                pass
            raise
        return self

    def close(self):
        node, sub, self.description_sub = self.node, self.description_sub, None
        first = None
        if node is not None and sub is not None:
            try:
                node.destroy_subscription(sub)
            except Exception as e:  # noqa: BLE001 - keep releasing
                first = e
        try:
            super().close()
        except Exception as e:  # noqa: BLE001
            first = first or e
        if first is not None:
            raise first

    def latest_body_pose(self):
        return self.tracker.latest_body_pose()

    def command_owner_snapshot(self):
        """Other commanders VISIBLE now (graph counts; see fb.command_owner_codes for the limits):
        publishers on the controller's topic, and FollowJointTrajectory clients other than this
        transport's own one."""
        self._require_open()
        return {'command_publishers': self.node.count_publishers(self.cfg.command_topic),
                'foreign_action_clients': max(0, self.node.count_subscribers(
                    action_status_topic(self.action_name)) - 1)}

    def fixed_base_snapshot(self):
        return self.tracker.snapshot()

    def streams_now(self, settle_s=None):
        """The streams NOW, for the check immediately before the send (M6.1, plan E5).

        Nothing spins during the confirmation prompt or the server wait (rclpy wait_for_server
        does not spin), so messages queue up to each subscription's depth. Processed later, they
        would get the processing time as their receipt time and look current. So:
          1. drain: process what is already queued (at most DRAIN_MAX_CALLBACKS callbacks);
          2. observe: spin for settle_s (default cfg.progress_window_s, the readiness window);
             only what is received from here on (since_wall) counts.
        Returns the time, since_wall, the latest joint state and the pose-tracker snapshot.
        Read-only: it sends, cancels and publishes nothing. The stream events received meanwhile
        are evidence for this check only and are dropped, as after a readiness collection, so
        tracking still starts at the dispatch.
        """
        self._require_open()
        settle_s = self.cfg.progress_window_s if settle_s is None else settle_s
        for _ in range(DRAIN_MAX_CALLBACKS):
            self.executor.spin_once(timeout_sec=0.0)
        since = time.monotonic()
        end = since + settle_s
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0:
                break
            self.executor.spin_once(timeout_sec=remaining)
        self._events = [e for e in self._events if e[0] not in STREAM_EVENT_KINDS]
        return {'now': time.monotonic(), 'since_wall': since,
                'joint_states': self._latest_joint_states, 'tracker': self.tracker.snapshot()}

    def graph_collector(self, timeout_s=10.0, window_s=2.0, discovery_s=2.0):
        """The read-only collector; pose and fixed-base events received meanwhile are readiness
        evidence only (latest_body_pose, fixed_base_snapshot) and are dropped afterwards."""
        base = super().graph_collector(timeout_s=timeout_s, window_s=window_s,
                                       discovery_s=discovery_s)

        def collect():
            obs = base()
            self._events = [e for e in self._events
                            if e[0] not in EVENT_KINDS_DROPPED_AFTER_COLLECT]
            return obs
        return collect

    def _on_joint_state(self, msg):
        super()._on_joint_state(msg)
        ev = self._events[-1]
        if ev[0] == 'joint_state':
            self._latest_joint_states = (ev[1], dict(zip(ev[3], ev[4])))

    def _on_description(self, msg):
        self.tracker.on_description(msg.data, time.monotonic())

    def _on_pose(self, msg):
        self.pose_messages += 1
        wall, stamp = time.monotonic(), self.sim_now()
        events = self.tracker.on_transforms(msg.transforms, wall, stamp)
        if not events:
            self.pose_messages_without_model += 1
        for ev in events:
            if ev[0] == 'body_pose':
                self._latest_pose = (ev[1], ev[2], ev[3])
            self._events.append(ev)


__all__ = ['M61AFixedBaseTransport', 'DESCRIPTION_TOPIC', 'description_qos',
           'DRAIN_MAX_CALLBACKS', 'STREAM_EVENT_KINDS']
