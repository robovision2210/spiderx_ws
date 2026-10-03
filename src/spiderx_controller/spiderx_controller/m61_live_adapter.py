"""M6.1 rclpy transport: the unchanged M6.0-D RclpyLiveTransport plus ONE read-only body-pose
subscription. Importing this module starts nothing.

Everything the M6.0-D transport guarantees still holds (own Context with rclpy signal handlers
disabled, single-use send_goal that verifies the goal fingerprint before any ROS call, single-use
cancel, publishes nothing, close() never stops an accepted goal). M6.1 adds:
  - a subscription to the Gazebo ground-truth pose bridged as /spiderx/sim/world_poses
    (tf2_msgs/msg/TFMessage, from /world/spiderx_fortress/pose/info; the bridge exists today only
    in spiderx_bringup fortress_posture_hold.launch.py - see docs/M61_IMPLEMENTATION_NOTES.md);
  - ('body_pose', wall_s, sim_stamp_s, (x, y, z, roll, pitch, yaw)) events and
    latest_body_pose() for readiness. The bridge leaves per-pose stamps at 0, so each sample is
    stamped with this node's /clock time (as in M2 posture_metrics).
Live use is HARD-DISABLED by the CLI (m61_live_contract.M61_LIVE_DISPATCH_ENABLED = False).
"""

import time

from spiderx_controller import m6_live_adapter as la
from spiderx_controller import posture_metrics as pm


class M61RclpyLiveTransport(la.RclpyLiveTransport):
    """M6.0-D LiveTransport + ground-truth body pose. Construct, then open(); always close()."""

    def __init__(self, expected_fingerprint, domain_id, node_name='m61_gait_replay',
                 pose_topic=pm.POSE_TOPIC, model_name=pm.MODEL_NAME, **kw):
        super().__init__(expected_fingerprint, domain_id, node_name=node_name, **kw)
        self.pose_topic = pose_topic
        self.model_name = model_name
        self.pose_sub = None
        self._latest_pose = None
        self.pose_messages = 0
        self.pose_messages_without_model = 0

    def open(self):
        super().open()
        try:
            from tf2_msgs.msg import TFMessage
            self.pose_sub = self.node.create_subscription(TFMessage, self.pose_topic,
                                                          self._on_pose, 10)
        except BaseException:
            try:
                self.close()
            except Exception:  # noqa: BLE001 - the original error is the one to report
                pass
            raise
        return self

    def close(self):
        """Release the pose subscription, then everything the M6.0-D transport created."""
        node, sub, self.pose_sub = self.node, self.pose_sub, None
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
        return self._latest_pose

    def graph_collector(self, timeout_s=10.0, window_s=2.0, discovery_s=2.0):
        """The M6.0-D read-only collector; body poses received meanwhile are readiness evidence
        only (latest_body_pose) and are dropped from the event queue afterwards."""
        base = super().graph_collector(timeout_s=timeout_s, window_s=window_s,
                                       discovery_s=discovery_s)

        def collect():
            obs = base()
            self._events = [e for e in self._events if e[0] != 'body_pose']
            return obs
        return collect

    def _on_pose(self, msg):
        self.pose_messages += 1
        try:
            x, y, z, qx, qy, qz, qw = pm.extract_model_pose(msg.transforms, self.model_name)
            roll, pitch, yaw = pm.quat_to_rpy(qx, qy, qz, qw)
        except pm.ObservationError:
            self.pose_messages_without_model += 1
            return
        wall, stamp = time.monotonic(), self.sim_now()
        pose = (float(x), float(y), float(z), roll, pitch, yaw)
        self._latest_pose = (wall, stamp, pose)
        self._events.append(('body_pose', wall, stamp, pose))


__all__ = ['M61RclpyLiveTransport']
