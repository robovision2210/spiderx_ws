"""M6.1-A READ-ONLY fixed-base observer and preflight (no goal, no command, no publisher).

    ros2 run spiderx_controller m61a_observe_fixed_base --interface-only
    ros2 run spiderx_controller m61a_observe_fixed_base --domain-id 0 --preflight
    ros2 run spiderx_controller m61a_observe_fixed_base --domain-id 0 [--duration 120]

It establishes, against a running fortress_m61a_fixed_base.launch.py, the evidence that the
welded plant is what the M6.1 replay expects (m61a_fixed_base.assess):
  * /robot_description (transient local) is the fixed-base wrapper with the approved mount;
  * Gazebo's pose entries: the model root 'spiderx' at the identity spawn, the body link
    'dummy_link' consistent with the description (identity association), the composed body at
    the weld pose (attachment), fresh by receipt time;
  * /joint_states fresh and complete (12 joints), exactly one publisher;
  * /clock progressing with no reset;
  * both controllers active (controller_manager list_controllers, a read-only service call);
  * NO publisher on the trajectory command topic.
The long mode also records the stationary distribution the provisional tolerances must be judged
against (body z range, attachment deviation, pose receipt gaps, rates).

Its node owns: subscriptions, one service client (list_controllers) and graph queries. It has no
publisher, no action client and no timer that commands anything; a static test checks the source.
The controllers the launch starts HOLD their positions; this tool sends them nothing.
Exit codes: 0 READY (all checks pass); 1 NOT READY (named codes) or interrupted; 2 usage.
"""

from dataclasses import dataclass, field
import math
import os
import time

from spiderx_controller import m61a_fixed_base as fb

SCHEMA = 'spiderx.m61a.observation/1'
DEFAULT_OUT = 'log/m61a_observation'
LIST_CONTROLLERS = '/controller_manager/list_controllers'
EXIT_READY, EXIT_NOT_READY, EXIT_USAGE = 0, 1, 2
MAX_DOMAIN_ID = 232
NON_CLAIMS = ('READY shows a welded, observable, quiet plant; it shows no tracking, no gait, no '
              'contact, no balance and no walking.',
              'Simulator ground truth (Gazebo SceneBroadcaster) is development evidence, not '
              'odometry or state estimation.')


def _pct(values, q):
    if not values:
        return None
    v = sorted(values)
    k = min(len(v) - 1, max(0, int(math.ceil(q * len(v))) - 1))
    return v[k]


@dataclass
class Stats:
    """Pure accumulator for the long observation (times injected; no ROS)."""
    pose_walls: list = field(default_factory=list)
    joint_walls: list = field(default_factory=list)
    clock: list = field(default_factory=list)            # (wall, sim)
    body: list = field(default_factory=list)             # (wall, x, y, z, roll, pitch, yaw, tilt)
    attachment: list = field(default_factory=list)       # (translation m, rotation rad)
    spawn: list = field(default_factory=list)
    link: list = field(default_factory=list)
    codes: dict = field(default_factory=dict)
    frame_ids: dict = field(default_factory=dict)
    joint_stamps: list = field(default_factory=list)

    def on_pose(self, wall, selection, obs):
        self.pose_walls.append(wall)
        for c in (selection.codes if obs is None else obs.codes):
            self.codes[c] = self.codes.get(c, 0) + 1
        for fid in selection.frame_ids:
            self.frame_ids[str(fid)] = self.frame_ids.get(str(fid), 0) + 1
        if obs is None or obs.body is None:
            return
        self.body.append((wall,) + tuple(obs.body.xyz_rpy()) + (obs.body.tilt(),))
        self.attachment.append(obs.attachment[:2])
        self.spawn.append(obs.spawn_deviation)
        self.link.append(obs.link_deviation)

    def on_joint_state(self, wall, stamp):
        self.joint_walls.append(wall)
        if stamp is not None:
            self.joint_stamps.append(stamp)

    def on_clock(self, wall, sim):
        self.clock.append((wall, sim))

    @staticmethod
    def _gaps(walls):
        g = [b - a for a, b in zip(walls, walls[1:])]
        return {'count': len(walls), 'mean_s': (sum(g) / len(g)) if g else None,
                'max_s': max(g) if g else None, 'p99_s': _pct(g, 0.99),
                'p999_s': _pct(g, 0.999)}

    def summary(self, cfg):
        out = {'pose_receipt': self._gaps(self.pose_walls),
               'joint_state_receipt': self._gaps(self.joint_walls),
               'sample_codes': dict(self.codes), 'header_frame_ids': dict(self.frame_ids)}
        js = sorted(self.joint_stamps)
        out['joint_state_sim_gap_max_s'] = (max(b - a for a, b in zip(js, js[1:]))
                                            if len(js) > 1 else None)
        if len(self.clock) > 1:
            (w0, s0), (w1, s1) = self.clock[0], self.clock[-1]
            out['clock'] = {'sim_span_s': s1 - s0, 'wall_span_s': w1 - w0,
                            'real_time_factor': (s1 - s0) / (w1 - w0) if w1 > w0 else None,
                            'poses_per_sim_second': (len(self.pose_walls) / (s1 - s0)
                                                     if s1 > s0 else None)}
        if self.body:
            zs = [b[3] for b in self.body]
            out['body'] = {'samples': len(self.body), 'z_mean_m': sum(zs) / len(zs),
                           'z_min_m': min(zs), 'z_max_m': max(zs), 'z_range_m': max(zs) - min(zs),
                           'xy_drift_m': math.hypot(self.body[-1][1] - self.body[0][1],
                                                    self.body[-1][2] - self.body[0][2]),
                           'roll_max_abs_rad': max(abs(b[4]) for b in self.body),
                           'pitch_max_abs_rad': max(abs(b[5]) for b in self.body),
                           'tilt_max_rad': max(b[7] for b in self.body),
                           'yaw_drift_rad': math.atan2(math.sin(self.body[-1][6] - self.body[0][6]),
                                                       math.cos(self.body[-1][6] - self.body[0][6]))}
            at = max(a[0] for a in self.attachment), max(a[1] for a in self.attachment)
            out['attachment'] = {
                'max_translation_m': at[0], 'max_rotation_rad': at[1],
                'fraction_of_tolerance': max(at[0] / cfg.attachment_translation_tol_m,
                                             at[1] / cfg.attachment_rotation_tol_rad),
                'max_spawn_deviation_m': max(s[0] for s in self.spawn),
                'max_link_deviation_m': max(x[0] for x in self.link)}
        return out


class FixedBaseObserver:
    """The read-only rclpy side. Own Context, explicit domain, no publisher, no action client."""

    def __init__(self, cfg, joint_names, domain_id, node_name='m61a_fixed_base_observer'):
        self.cfg = cfg
        self.joint_names = list(joint_names)
        self.domain_id = domain_id
        self.node_name = node_name
        self.context = self.node = self.executor = None
        self.tracker = fb.FixedBasePoseTracker(cfg)
        self.clock = fb.ClockMonitor(cfg.sim_stall_s, cfg.clock_reset_tol_s)
        self.stats = Stats()
        self.latest_joint_states = None
        self.subs = []
        self.client = None
        self.closed = False

    # ---------------------------------------------------------------- lifecycle
    def open(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from controller_manager_msgs.srv import ListControllers
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from tf2_msgs.msg import TFMessage
        from spiderx_controller import m61a_live
        try:
            self.context = Context()
            rclpy.init(context=self.context, domain_id=self.domain_id,
                       signal_handler_options=SignalHandlerOptions.NO)
            self.node = rclpy.create_node(self.node_name, context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.subs = [
                self.node.create_subscription(TFMessage, self.cfg.pose_topic, self._on_pose, 50),
                self.node.create_subscription(JointState, self.cfg.joint_state_topic,
                                              self._on_joint_states, 50),
                self.node.create_subscription(Clock, '/clock', self._on_clock, best_effort),
                self.node.create_subscription(String, m61a_live.DESCRIPTION_TOPIC,
                                              self._on_description,
                                              m61a_live.description_qos()),
            ]
            self.client = self.node.create_client(ListControllers, LIST_CONTROLLERS)
        except BaseException:
            self.close()
            raise
        return self

    def close(self):
        """Release everything this observer created, in reverse order. Idempotent."""
        import rclpy
        first = None
        node, subs, client = self.node, self.subs, self.client
        self.subs, self.client = [], None
        if node is not None:
            for sub in subs:
                try:
                    node.destroy_subscription(sub)
                except Exception as e:  # noqa: BLE001 - keep releasing
                    first = first or e
            if client is not None:
                try:
                    node.destroy_client(client)
                except Exception as e:  # noqa: BLE001
                    first = first or e
        for step in (lambda: self.executor and self.executor.shutdown(timeout_sec=1.0),
                     lambda: node and node.destroy_node(),
                     lambda: self.context and rclpy.try_shutdown(context=self.context)):
            try:
                step()
            except Exception as e:  # noqa: BLE001
                first = first or e
        self.node = self.executor = self.context = None
        self.closed = True
        if first is not None:
            raise first

    # ---------------------------------------------------------------- callbacks (receipt time)
    def _on_pose(self, msg):
        wall = time.monotonic()
        stamp = self.clock.last_sim
        events = self.tracker.on_transforms(msg.transforms, wall, stamp)
        sel = fb.select_entries(msg.transforms, self.cfg.model_name, self.cfg.body_link)
        obs = None
        if not sel.codes and self.tracker.weld is not None and self.tracker.weld.fixed_base:
            obs = fb.evaluate_sample(sel, self.tracker.weld, self.cfg, wall, stamp)
        self.stats.on_pose(wall, sel, obs)
        return events

    def _on_joint_states(self, msg):
        wall = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latest_joint_states = (wall, dict(zip(msg.name, msg.position)))
        self.stats.on_joint_state(wall, stamp)

    def _on_clock(self, msg):
        wall = time.monotonic()
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.clock.on_clock(wall, sim)
        self.stats.on_clock(wall, sim)

    def _on_description(self, msg):
        self.tracker.on_description(msg.data, time.monotonic())

    # ---------------------------------------------------------------- queries (read-only)
    def spin(self, duration_s, stop=None):
        end = time.monotonic() + duration_s
        while time.monotonic() < end and not (stop and stop()):
            self.executor.spin_once(timeout_sec=0.05)

    def controllers(self, timeout_s=5.0):
        """{name: state} from list_controllers, or None if the service is unavailable."""
        from controller_manager_msgs.srv import ListControllers
        if not self.client.wait_for_service(timeout_sec=timeout_s):
            return None
        fut = self.client.call_async(ListControllers.Request())
        end = time.monotonic() + timeout_s
        while not fut.done() and time.monotonic() < end:
            self.executor.spin_once(timeout_sec=0.05)
        if not fut.done() or fut.result() is None:
            return None
        return {c.name: c.state for c in fut.result().controller}

    def graph(self):
        def names(topic):
            return sorted(f'{i.node_namespace.rstrip("/")}/{i.node_name}'
                          for i in self.node.get_publishers_info_by_topic(topic))
        from spiderx_controller import m61a_live
        return {'joint_state_publishers': self.node.count_publishers(self.cfg.joint_state_topic),
                'command_publishers': self.node.count_publishers(self.cfg.command_topic),
                'pose_publishers': names(self.cfg.pose_topic),
                'description_publishers': names(m61a_live.DESCRIPTION_TOPIC),
                'clock_publishers': names('/clock')}

    def evidence(self, controllers, graph):
        return fb.Evidence(
            description=self.tracker.description,
            description_received_wall=self.tracker.description_wall,
            latest_usable_pose=self.tracker.latest_usable,
            latest_pose_codes=self.tracker.latest_codes,
            latest_joint_states=self.latest_joint_states, clock=self.clock,
            controllers=controllers,
            joint_state_publishers=graph.get('joint_state_publishers'),
            command_publishers=graph.get('command_publishers'))


def run_observation(observer, cfg, duration_s, discovery_s, interrupted=None):
    """Spin, query, assess. Returns the report dict (pure apart from the observer's ROS calls)."""
    observer.spin(discovery_s, interrupted)
    c0, g0 = observer.controllers(), observer.graph()
    observer.spin(duration_s, interrupted)
    c1, g1 = observer.controllers(), observer.graph()
    now = time.monotonic()
    ready, codes, rep = fb.assess(observer.evidence(c1, g1), cfg, now, observer.joint_names)
    if interrupted and interrupted():
        ready, codes = False, codes + ['interrupted']
    return {'ready': ready, 'failure_codes': codes, 'checks': rep['checks'],
            'controllers': {'start': c0, 'end': c1}, 'graph': {'start': g0, 'end': g1},
            'statistics': observer.stats.summary(cfg),
            'tracker': {k: v for k, v in observer.tracker.snapshot().items()
                        if k not in ('description', 'latest_usable_pose')}}


def interface_report(cfg, joint_names):
    from spiderx_controller import m61a_live
    return {'subscriptions': {cfg.pose_topic: 'tf2_msgs/msg/TFMessage',
                              cfg.joint_state_topic: 'sensor_msgs/msg/JointState',
                              '/clock': 'rosgraph_msgs/msg/Clock (best effort)',
                              m61a_live.DESCRIPTION_TOPIC: 'std_msgs/msg/String (transient local)'},
            'service_clients': {LIST_CONTROLLERS: 'controller_manager_msgs/srv/ListControllers'},
            'publishers': {}, 'action_clients': {},
            'graph_queries': ['count_publishers ' + cfg.joint_state_topic,
                              'count_publishers ' + cfg.command_topic,
                              'publisher node names of the pose, description and clock topics'],
            'joint_names': list(joint_names), 'model_name': cfg.model_name,
            'body_link': cfg.body_link, 'mount_xyz_rpy': list(cfg.mount.xyz_rpy())}


def _config_sha(config_dir=None):
    import hashlib
    with open(fb.config_path(config_dir), 'rb') as f:
        return hashlib.sha256(f.read()).hexdigest()


def parse_args(argv):
    import argparse
    p = argparse.ArgumentParser(prog='m61a_observe_fixed_base',
                                description='M6.1-A READ-ONLY fixed-base observer and preflight. '
                                            'Sends no goal and publishes nothing.')
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--interface-only', action='store_true',
                      help='print what would be observed; create no ROS node')
    mode.add_argument('--preflight', action='store_true',
                      help='short readiness observation (config readiness_window_s)')
    p.add_argument('--duration', type=float, default=None,
                   help='observation length in seconds (default: config window_s)')
    p.add_argument('--domain-id', type=int, default=None,
                   help='ROS domain of the running simulation, typed explicitly (never read '
                        'from the environment)')
    p.add_argument('--out', default=DEFAULT_OUT, help=f'evidence root (default {DEFAULT_OUT})')
    p.add_argument('--no-write', action='store_true', help='print only; write no file')
    return p.parse_args(argv)


def main(argv=None, observer_factory=None, joint_names=None, config_dir=None, utc=None):
    import json
    import sys
    from datetime import datetime, timezone
    args = parse_args(sys.argv[1:] if argv is None else argv)
    print('SpiderX M6.1-A fixed-base observer (READ-ONLY: no goal, no command, no publisher)')
    try:
        cfg = fb.load_config(config_dir)
    except (OSError, fb.FixedBaseError) as e:
        print(f'REFUSED: fixed-base configuration: {e}')
        return EXIT_USAGE
    if joint_names is None:
        from spiderx_controller import m6_trajectory as m6t
        from spiderx_controller.config_check import load_urdf
        joint_names = m6t.load_sources(None, load_urdf()).joint_names
    if args.interface_only:
        print(json.dumps(interface_report(cfg, joint_names), indent=1))
        return EXIT_READY
    if args.domain_id is None or not 0 <= args.domain_id <= MAX_DOMAIN_ID:
        print(f'usage: an explicit --domain-id in 0..{MAX_DOMAIN_ID} is required')
        return EXIT_USAGE
    obs_cfg = cfg.raw['observation']
    duration = args.duration if args.duration is not None else (
        obs_cfg['readiness_window_s'] if args.preflight else obs_cfg['window_s'])
    if not (math.isfinite(duration) and duration > 0):
        print('usage: --duration must be a positive number')
        return EXIT_USAGE
    observer = (observer_factory or FixedBaseObserver)(cfg, joint_names, args.domain_id)
    interrupted = [False]
    report = None
    try:
        observer.open()
        try:
            report = run_observation(observer, cfg, duration, obs_cfg['discovery_s'],
                                     lambda: interrupted[0])
        except KeyboardInterrupt:
            interrupted[0] = True
            report = {'ready': False, 'failure_codes': ['interrupted'], 'checks': {},
                      'statistics': observer.stats.summary(cfg)}
    except Exception as e:  # noqa: BLE001 - partial initialization is reported, never hidden
        report = {'ready': False, 'failure_codes': ['observer_failed'],
                  'error': f'{type(e).__name__}: {e}', 'checks': {}}
    finally:
        try:
            if not getattr(observer, 'closed', True):
                observer.close()
        except Exception as e:  # noqa: BLE001
            report = dict(report or {}, close_error=f'{type(e).__name__}: {e}')
    report.update({'schema': SCHEMA, 'milestone': 'M6.1-A',
                   'mode': 'preflight' if args.preflight else 'observe',
                   'duration_s': duration, 'domain_id': args.domain_id,
                   'config_sha256': _config_sha(config_dir), 'non_claims': list(NON_CLAIMS),
                   'goals_sent': 0, 'publishers_created': 0})
    verdict = 'READY' if report['ready'] else 'NOT READY'
    print(f'{verdict}: ' + (', '.join(report['failure_codes']) or 'all fixed-base checks pass'))
    for name, chk in sorted((report.get('checks') or {}).items()):
        if isinstance(chk, dict) and 'ok' in chk:
            print(f'  {"PASS" if chk["ok"] else "FAIL"}  {name}' +
                  ('' if chk['ok'] else f'  ({chk["code"]})'))
    if not args.no_write:
        stamp = utc or datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        d = os.path.join(args.out, stamp)
        try:
            os.makedirs(d, exist_ok=False)
            path = os.path.join(d, 'observation.json')
            with open(path, 'x') as f:
                json.dump(report, f, indent=1, sort_keys=True, default=str)
            print(f'Evidence: {path}')
        except FileExistsError:
            print(f'REFUSED: {d} already exists (evidence is never overwritten)')
            return EXIT_USAGE
    return EXIT_READY if report['ready'] else EXIT_NOT_READY


__all__ = ['FixedBaseObserver', 'Stats', 'run_observation', 'interface_report', 'main',
           'SCHEMA', 'EXIT_READY', 'EXIT_NOT_READY', 'EXIT_USAGE']
