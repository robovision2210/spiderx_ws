"""M6.1-A rclpy transport: the M6.1 transport whose body pose is the COMPOSED fixed-base body.

M61AFixedBaseTransport = m61_live_adapter.M61RclpyLiveTransport (itself the unchanged M6.0-D
transport + one ground-truth pose subscription) with:
  - ONE extra read-only subscription: /robot_description (std_msgs/String, transient local, as
    robot_state_publisher publishes it), parsed by m61a_fixed_base for the weld;
  - the pose callback replaced by m61a_fixed_base.FixedBasePoseTracker: the body pose handed to
    the M6.1 gates is T_world_model * T_model_dummy(Gazebo entry) * T_dummy_base, and every sample
    also yields a ('fixed_base', ...) event for G8;
  - fixed_base_snapshot() for the M6.1-A readiness check (m6_gait_replay.with_fixed_base).
It publishes nothing, creates no action client beyond the M6.0-D one, and sends no goal of its
own. Importing this module starts nothing. Live use stays HARD-DISABLED by the M6.1 CLI
(m61_live_contract.M61_LIVE_DISPATCH_ENABLED = False).
"""

import time

from spiderx_controller import m61_live_adapter as la
from spiderx_controller import m61a_fixed_base as fb

DESCRIPTION_TOPIC = '/robot_description'
EVENT_KINDS_DROPPED_AFTER_COLLECT = ('body_pose', 'fixed_base')


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


__all__ = ['M61AFixedBaseTransport', 'DESCRIPTION_TOPIC', 'description_qos']
