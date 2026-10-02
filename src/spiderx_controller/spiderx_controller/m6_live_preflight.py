"""M6.0-B LIVE READ-ONLY preflight (Batch C). Sends no goal, publishes nothing, launches nothing.

    # terminal 1, started by the operator (this tool never starts Gazebo or a launch file):
    ros2 launch spiderx_bringup fortress_control.launch.py
    # terminal 2:
    ros2 run spiderx_controller m6_live_preflight                    # graph + interface checks
    ros2 run spiderx_controller m6_live_preflight --interface-only   # no ROS node at all

What it inspects (read-only):
  - the FollowJointTrajectory action server (graph query; no action client is created);
  - controller states (one /controller_manager/list_controllers query; nothing is switched);
  - /joint_states: publisher count and source, freshness, names, completeness (subscription);
  - the 12-joint name contract against the canonical controller order;
  - the observed start pose against neutral (start-pose tolerance, a separate quantity);
  - installed package versions and the installed action interface (D6).
A report is printed and saved under log/m6_playback/live_preflight/ (git-ignored).

PASSING THIS PREFLIGHT DOES NOT PROVE TRACKING, MOVEMENT, CONTACT OR WALKING. It only shows that
the interfaces M6.0-D would use are present, and nothing was sent.

Exit codes: 0 = READY (every requirement met), 1 = NOT READY (a requirement is missing),
2 = refused (invalid arguments or an existing report directory).

The decision logic (evaluate) is pure Python and unit-tested with mocks. ROS calls are confined to
RosApi and GraphProbe; tests replace both.
"""

import argparse
from dataclasses import dataclass, field
import datetime
import hashlib
import json
import math
import os
import sys
import time

from spiderx_controller import m6_envelope as env

REPORT_SCHEMA = 'spiderx.m6.live_preflight/1'
EXIT_READY, EXIT_NOT_READY, EXIT_REFUSED = 0, 1, 2
DEFAULT_OUT = os.path.join('log', 'm6_playback', 'live_preflight')
LIST_CONTROLLERS_SERVICE = '/controller_manager/list_controllers'
JTC_TYPE = 'joint_trajectory_controller/JointTrajectoryController'
JOINT_STATE_TYPE = 'sensor_msgs/msg/JointState'
MIN_JOINT_STATE_MESSAGES = 2
MAX_KEPT_MESSAGES = 200

# Cloud reference stack (docs/M6_GAIT_PLAYBACK_SAFETY_PLAN.md section 2.2). A difference is
# reported visibly (D6); the interface-contract check is what must pass.
REFERENCE_VERSIONS = {
    'joint_trajectory_controller': '2.48.0',
    'control_msgs': '4.8.0',
    'trajectory_msgs': '4.9.0',
    'controller_manager': '2.51.0',
    'controller_manager_msgs': '2.51.0',
    'hardware_interface': '2.51.0',
    'gz_ros2_control': '0.7.15',
    'rclpy': '3.3.16',
    'sensor_msgs': '4.9.0',
    'action_msgs': '1.2.1',
}
REQUIRED_GOAL_FIELDS = ('trajectory', 'path_tolerance', 'goal_tolerance', 'goal_time_tolerance')
REQUIRED_ERROR_CODES = {'SUCCESSFUL': 0, 'INVALID_GOAL': -1, 'INVALID_JOINTS': -2,
                        'OLD_HEADER_TIMESTAMP': -3, 'PATH_TOLERANCE_VIOLATED': -4,
                        'GOAL_TOLERANCE_VIOLATED': -5}
REQUIRED_TOLERANCE_FIELDS = ('name', 'position', 'velocity', 'acceleration')

NOT_PROOF = ('Passing this preflight does NOT prove tracking, movement, contact, body support, '
             'balance or walking. No goal was sent and nothing was published.')

FAILURE_CODES = {
    'action_server_missing': 'no server for the FollowJointTrajectory action',
    'action_server_ambiguous': 'more than one server for the action',
    'action_type_mismatch': 'the action has an unexpected type',
    'controller_manager_unavailable': 'list_controllers did not answer within the timeout',
    'controller_missing': 'a required controller is not loaded',
    'controller_not_active': 'a required controller is not active',
    'controller_type_mismatch': 'leg_trajectory_controller is not a JointTrajectoryController',
    'joint_states_no_publisher': '/joint_states has no publisher',
    'joint_states_multiple_publishers': '/joint_states has more than one publisher',
    'joint_states_type_mismatch': '/joint_states has an unexpected message type',
    'joint_states_no_messages': 'too few /joint_states messages within the window',
    'joint_states_stale': '/joint_states header stamps are zero or not increasing',
    'joint_names_mismatch': '/joint_states names differ from the 12-joint contract',
    'joint_states_incomplete': '/joint_states positions are missing or non-finite',
    'start_pose_not_neutral': 'the observed pose is not within the start-pose tolerance',
    'interface_contract_mismatch': 'the installed action interface differs from the contract',
    'probe_error': 'the read-only probe raised an error',
}


# ---------------------------------------------------------------- observations
@dataclass
class Observations:
    """What the probe saw. None = not observed (timeout or skipped)."""

    action_servers: object = None        # [(node_fqn, [type, ...])] for env.ACTION_NAME
    controllers: object = None           # [(name, type, state)]
    joint_state_publishers: object = None    # [(node_fqn, topic_type)]
    joint_state_messages: list = field(default_factory=list)  # [(stamp_s, names, positions)]
    versions: dict = field(default_factory=dict)
    interface: dict = field(default_factory=dict)
    probe_errors: list = field(default_factory=list)
    graph_probed: bool = True


# ---------------------------------------------------------------- offline collectors (no node)
def collect_versions(packages=tuple(REFERENCE_VERSIONS)):
    """{package: version or None} from installed package.xml files (ament index; no node)."""
    import xml.etree.ElementTree as ET
    from ament_index_python.packages import get_package_share_directory
    out = {}
    for pkg in packages:
        try:
            path = os.path.join(get_package_share_directory(pkg), 'package.xml')
            out[pkg] = ET.parse(path).getroot().findtext('version')
        except Exception:  # noqa: BLE001 - a missing package is a visible 'None'
            out[pkg] = None
    out['ROS_DISTRO'] = os.environ.get('ROS_DISTRO')
    return out


def collect_interface():
    """The installed FollowJointTrajectory contract as plain data (imports message types only)."""
    from ament_index_python.packages import get_package_share_directory
    from control_msgs.action import FollowJointTrajectory
    from control_msgs.msg import JointTolerance
    path = os.path.join(get_package_share_directory('control_msgs'), 'action',
                        'FollowJointTrajectory.action')
    with open(path, 'rb') as f:
        digest = hashlib.sha256(f.read()).hexdigest()
    result = FollowJointTrajectory.Result
    return {
        'action_file_sha256': digest,
        'goal_fields': sorted(FollowJointTrajectory.Goal.get_fields_and_field_types()),
        'tolerance_fields': sorted(JointTolerance.get_fields_and_field_types()),
        'error_codes': {n: getattr(result, n, None) for n in REQUIRED_ERROR_CODES},
    }


def compare_versions(versions):
    rows = {}
    for pkg, ref in REFERENCE_VERSIONS.items():
        got = versions.get(pkg)
        rows[pkg] = {'reference': ref, 'installed': got,
                     'status': 'unavailable' if got is None else
                     ('matches' if got == ref else 'differs')}
    overall = ('matches' if all(r['status'] == 'matches' for r in rows.values())
               else 'differs')
    return overall, rows


# ---------------------------------------------------------------- decision logic (pure)
class _Checks:
    def __init__(self):
        self.checks = {}
        self.failures = []

    def fail(self, check, code, message):
        if code not in FAILURE_CODES:
            raise KeyError(f'undocumented failure code {code!r}')
        self.checks[check] = 'failed'
        self.failures.append((code, message))

    def ok(self, check):
        self.checks.setdefault(check, 'passed')

    def skip(self, check):
        self.checks.setdefault(check, 'not_run')


def evaluate(obs, canonical_names, neutral):
    """Apply the M6.0-B requirements to observations. Returns the report dict (never raises)."""
    c = _Checks()
    for msg in obs.probe_errors:
        c.fail('probe', 'probe_error', msg)
    c.ok('probe')

    # interface contract (installed message definitions)
    itf = obs.interface or {}
    if not itf:
        c.fail('interface_contract', 'interface_contract_mismatch', 'interface not collected')
    else:
        missing = [f for f in REQUIRED_GOAL_FIELDS if f not in itf.get('goal_fields', [])]
        tol_missing = [f for f in REQUIRED_TOLERANCE_FIELDS
                       if f not in itf.get('tolerance_fields', [])]
        codes = itf.get('error_codes', {})
        wrong = {n: codes.get(n) for n, v in REQUIRED_ERROR_CODES.items() if codes.get(n) != v}
        if missing or tol_missing or wrong:
            c.fail('interface_contract', 'interface_contract_mismatch',
                   f'missing goal fields {missing}, tolerance fields {tol_missing}, '
                   f'error codes {wrong}')
    c.ok('interface_contract')

    if not obs.graph_probed:
        for name in ('action_server', 'controllers', 'joint_states_source',
                     'joint_states_fresh', 'joint_names_contract', 'start_pose'):
            c.skip(name)
        return _report(c, obs, canonical_names, neutral, graph=False)

    # action server
    servers = obs.action_servers
    if not servers:
        c.fail('action_server', 'action_server_missing',
               f'no server for {env.ACTION_NAME} (is the controller stack running?)')
    else:
        if len(servers) > 1:
            c.fail('action_server', 'action_server_ambiguous',
                   f'{len(servers)} servers: {[s[0] for s in servers]}')
        for node_name, types in servers:
            if list(types) != [env.ACTION_TYPE]:
                c.fail('action_server', 'action_type_mismatch', f'{node_name}: {list(types)}')
    c.ok('action_server')

    # controllers
    if obs.controllers is None:
        c.fail('controllers', 'controller_manager_unavailable',
               f'{LIST_CONTROLLERS_SERVICE} did not answer')
    else:
        by_name = {n: (t, s) for n, t, s in obs.controllers}
        for name in env.REQUIRED_CONTROLLERS:
            if name not in by_name:
                c.fail('controllers', 'controller_missing', f'{name} is not loaded')
            elif by_name[name][1] != 'active':
                c.fail('controllers', 'controller_not_active', f'{name} is {by_name[name][1]}')
        if env.CONTROLLER_NAME in by_name and by_name[env.CONTROLLER_NAME][0] != JTC_TYPE:
            c.fail('controllers', 'controller_type_mismatch',
                   f'{env.CONTROLLER_NAME} type {by_name[env.CONTROLLER_NAME][0]}')
    c.ok('controllers')

    # /joint_states source
    pubs = obs.joint_state_publishers or []
    if not pubs:
        c.fail('joint_states_source', 'joint_states_no_publisher', 'no publisher')
    else:
        if len(pubs) > 1:
            c.fail('joint_states_source', 'joint_states_multiple_publishers',
                   f'{len(pubs)} publishers: {[p[0] for p in pubs]}')
        bad = [p for p in pubs if p[1] != JOINT_STATE_TYPE]
        if bad:
            c.fail('joint_states_source', 'joint_states_type_mismatch', f'{bad}')
    c.ok('joint_states_source')

    # freshness
    msgs = obs.joint_state_messages or []
    if len(msgs) < MIN_JOINT_STATE_MESSAGES:
        c.fail('joint_states_fresh', 'joint_states_no_messages',
               f'{len(msgs)} messages received (< {MIN_JOINT_STATE_MESSAGES})')
    else:
        stamps = [m[0] for m in msgs]
        if any(not (isinstance(s, (int, float)) and s > 0) for s in stamps) or \
                any(b <= a for a, b in zip(stamps, stamps[1:])):
            c.fail('joint_states_fresh', 'joint_states_stale',
                   f'header stamps not positive and strictly increasing: '
                   f'{stamps[:3]}...{stamps[-2:]}')
    c.ok('joint_states_fresh')

    # names, completeness, start pose (latest message)
    if not msgs:
        c.skip('joint_names_contract')
        c.skip('start_pose')
    else:
        _, names, positions = msgs[-1]
        names = list(names)
        if len(set(names)) != len(names) or set(names) != set(canonical_names):
            c.fail('joint_names_contract', 'joint_names_mismatch',
                   f'missing {sorted(set(canonical_names) - set(names))}, unknown '
                   f'{sorted(set(names) - set(canonical_names))}, '
                   f'duplicates {len(names) - len(set(names))}')
        if len(positions) != len(names) or not all(
                isinstance(p, (int, float)) and math.isfinite(p) for p in positions):
            c.fail('joint_names_contract', 'joint_states_incomplete',
                   f'{len(positions)} positions for {len(names)} names, or non-finite values')
        c.ok('joint_names_contract')
        if c.checks['joint_names_contract'] == 'failed':
            c.skip('start_pose')
        else:
            pos = dict(zip(names, positions))
            dev = {n: abs(pos[n] - q0) for n, q0 in zip(canonical_names, neutral)}
            worst = max(dev, key=dev.get)
            if dev[worst] > env.START_POSE_TOLERANCE_RAD:
                c.fail('start_pose', 'start_pose_not_neutral',
                       f'{worst} is {dev[worst]:.4f} rad from neutral > start-pose tolerance '
                       f'{env.START_POSE_TOLERANCE_RAD} rad (run test_neutral_pose.py first)')
            c.ok('start_pose')
    return _report(c, obs, canonical_names, neutral, graph=True)


def _json_number(v):
    return v if isinstance(v, (int, float)) and math.isfinite(v) else repr(v)


def _report(c, obs, canonical_names, neutral, graph):
    version_status, version_rows = compare_versions(obs.versions or {})
    ready = graph and not c.failures
    last = obs.joint_state_messages[-1] if obs.joint_state_messages else None
    return {
        'schema': REPORT_SCHEMA,
        'verdict': 'READY' if ready else ('NOT READY' if graph else 'INTERFACE ONLY'),
        'ready': ready,
        'goals_sent': 0,
        'messages_published': 0,
        'failure_codes': sorted({code for code, _ in c.failures}),
        'failures': [{'code': code, 'message': m} for code, m in c.failures],
        'checks': dict(c.checks),
        'observed': {
            'action_servers': obs.action_servers,
            'controllers': obs.controllers,
            'joint_state_publishers': obs.joint_state_publishers,
            'joint_state_messages': len(obs.joint_state_messages or []),
            'last_joint_state': ({'stamp_s': last[0], 'names': list(last[1]),
                                  'positions': [_json_number(p) for p in last[2]]}
                                 if last else None),
        },
        'contract': {'action_name': env.ACTION_NAME, 'action_type': env.ACTION_TYPE,
                     'required_controllers': list(env.REQUIRED_CONTROLLERS),
                     'joint_names': list(canonical_names),
                     'start_pose_tolerance_rad': env.START_POSE_TOLERANCE_RAD,
                     'neutral': list(neutral)},
        'versions': dict(obs.versions or {}),
        'version_check': {'status': version_status, 'packages': version_rows,
                          'note': 'visible outcome (D6); the interface contract must pass'},
        'interface': dict(obs.interface or {}),
        'not_proof': NOT_PROOF,
        'non_claims': list(env.NON_CLAIMS),
    }


# ---------------------------------------------------------------- read-only ROS probe
class RosApi:
    """The only rclpy entry points the probe uses. Tests substitute a fake."""

    def __init__(self):
        import rclpy
        from rclpy.action import get_action_server_names_and_types_by_node
        from controller_manager_msgs.srv import ListControllers
        from sensor_msgs.msg import JointState
        self.rclpy = rclpy
        self.get_action_server_names_and_types_by_node = get_action_server_names_and_types_by_node
        self.ListControllers = ListControllers
        self.JointState = JointState

    def spin_once(self, node, timeout_sec):
        self.rclpy.spin_once(node, timeout_sec=timeout_sec)

    def spin_until_future_complete(self, node, future, timeout_sec):
        self.rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)


def _fqn(name, ns):
    return (ns.rstrip('/') + '/' + name) if ns else '/' + name


class GraphProbe:
    """Read-only observation of the ROS graph. Creates exactly one service client (for the
    list_controllers query) and one subscription (/joint_states). No publisher, no action client,
    no parameter or lifecycle change, no child process."""

    def __init__(self, node, ros, timeout_s=10.0, window_s=2.0, discovery_s=2.0,
                 monotonic=time.monotonic):
        self.node = node
        self.ros = ros
        self.timeout_s = timeout_s
        self.window_s = window_s
        self.discovery_s = discovery_s
        self.monotonic = monotonic

    def _spin_for(self, seconds):
        end = self.monotonic() + seconds
        while self.monotonic() < end:
            self.ros.spin_once(self.node, 0.05)

    def action_servers(self):
        out = []
        for name, ns in self.node.get_node_names_and_namespaces():
            for action, types in self.ros.get_action_server_names_and_types_by_node(
                    self.node, name, ns):
                if action == env.ACTION_NAME:
                    out.append((_fqn(name, ns), list(types)))
        return out

    def controllers(self):
        client = self.node.create_client(self.ros.ListControllers, LIST_CONTROLLERS_SERVICE)
        try:
            if not client.wait_for_service(timeout_sec=self.timeout_s):
                return None
            future = client.call_async(self.ros.ListControllers.Request())
            self.ros.spin_until_future_complete(self.node, future, self.timeout_s)
            res = future.result() if future.done() else None
            if res is None:
                return None
            return [(c.name, c.type, c.state) for c in res.controller]
        finally:
            self.node.destroy_client(client)

    def joint_state_publishers(self):
        return [(_fqn(i.node_name, i.node_namespace), i.topic_type)
                for i in self.node.get_publishers_info_by_topic(env.JOINT_STATES_TOPIC)]

    def joint_state_messages(self):
        msgs = []

        def on_msg(m):
            if len(msgs) < MAX_KEPT_MESSAGES:
                msgs.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                             list(m.name), [float(p) for p in m.position]))
        sub = self.node.create_subscription(self.ros.JointState, env.JOINT_STATES_TOPIC,
                                            on_msg, 10)
        try:
            self._spin_for(self.window_s)
        finally:
            self.node.destroy_subscription(sub)
        return msgs

    def collect(self):
        obs = Observations()
        self._spin_for(self.discovery_s)                  # let graph discovery settle
        for attr in ('action_servers', 'controllers', 'joint_state_publishers',
                     'joint_state_messages'):
            try:
                setattr(obs, attr, getattr(self, attr)())
            except Exception as e:  # noqa: BLE001 - recorded, never hidden
                obs.probe_errors.append(f'{attr}: {type(e).__name__}: {e}')
        return obs


# ---------------------------------------------------------------- CLI
def parse_args(argv):
    p = argparse.ArgumentParser(description='SpiderX M6.0-B live READ-ONLY preflight '
                                '(sends no goal)')
    p.add_argument('--config-dir', default=None, help='configuration directory (default: '
                   'installed)')
    p.add_argument('--interface-only', action='store_true',
                   help='collect installed versions and the action interface only; no ROS node')
    p.add_argument('--timeout', type=float, default=10.0, help='per-query wall timeout, s')
    p.add_argument('--window', type=float, default=2.0, help='/joint_states sampling window, s')
    p.add_argument('--out', default=DEFAULT_OUT, help=f'report root (default: {DEFAULT_OUT})')
    p.add_argument('--no-write', action='store_true', help='print only')
    return p.parse_args(argv)


def contract_from_config(config_dir=None):
    """(canonical joint names, neutral) from the shipped configuration (no URDF, no node)."""
    from spiderx_controller.joint_safety import load_pose
    from spiderx_controller.posture_config import load_yaml_strict
    from spiderx_controller.m6_trajectory import default_config_dir
    config_dir = config_dir or default_config_dir()
    ctrl = load_yaml_strict(os.path.join(config_dir, 'spiderx_ros2_controllers.yaml'))
    names = list(ctrl['leg_trajectory_controller']['ros__parameters']['joints'])
    pose = load_pose('cad_neutral', config_dir)
    return names, [float(pose[n]) for n in names]


def print_report(report):
    print(f'M6.0-B live read-only preflight: {report["verdict"]}')
    for name, state in report['checks'].items():
        print(f'  [{state:7s}] {name}')
    for f in report['failures']:
        print(f'  NOT READY {f["code"]}: {f["message"]}')
    vc = report['version_check']
    print(f'  installed versions vs cloud reference: {vc["status"]}')
    for pkg, row in vc['packages'].items():
        if row['status'] != 'matches':
            print(f'    {pkg}: installed {row["installed"]}, reference {row["reference"]} '
                  f'({row["status"]})')
    print(f'  goals sent: {report["goals_sent"]}; messages published: '
          f'{report["messages_published"]}')
    print('  ' + NOT_PROOF)


def write_report(report, out_root, now=None):
    now = now or datetime.datetime.now(datetime.timezone.utc)
    d = os.path.join(out_root, now.strftime('%Y%m%dT%H%M%SZ'))
    if os.path.exists(d):
        raise FileExistsError(d)
    os.makedirs(d)
    path = os.path.join(d, 'live_preflight.json')
    with open(path, 'w') as f:
        f.write(json.dumps(report, indent=1, sort_keys=True, allow_nan=False, default=str) + '\n')
    return path


def main(argv=None, probe_factory=None, versions=None, interface=None, now=None):
    argv = list(argv if argv is not None else sys.argv)
    args = parse_args(argv[1:])
    print('SpiderX M6.0-B live READ-ONLY preflight. Sends no goal; publishes nothing; '
          'never starts Gazebo or a launch file.')
    if args.timeout <= 0 or args.window <= 0:
        print('REFUSED: --timeout and --window must be > 0')
        return EXIT_REFUSED
    try:
        names, neutral = contract_from_config(args.config_dir)
    except Exception as e:  # noqa: BLE001
        print(f'REFUSED: cannot read the joint contract: {type(e).__name__}: {e}')
        return EXIT_REFUSED
    obs = Observations()
    try:
        obs.versions = versions if versions is not None else collect_versions()
        obs.interface = interface if interface is not None else collect_interface()
    except Exception as e:  # noqa: BLE001
        obs.probe_errors.append(f'interface: {type(e).__name__}: {e}')
    if args.interface_only:
        obs.graph_probed = False
    else:
        factory = probe_factory or _ros_probe
        try:
            graph = factory(args)
            for attr in ('action_servers', 'controllers', 'joint_state_publishers',
                         'joint_state_messages'):
                setattr(obs, attr, getattr(graph, attr))
            obs.probe_errors += graph.probe_errors
        except KeyboardInterrupt:
            obs.probe_errors.append('interrupted by the operator')
        except Exception as e:  # noqa: BLE001
            obs.probe_errors.append(f'probe: {type(e).__name__}: {e}')
    report = evaluate(obs, names, neutral)
    print_report(report)
    if not args.no_write:
        try:
            print(f'Report: {write_report(report, args.out, now)}')
        except FileExistsError as e:
            print(f'REFUSED: {e} already exists (reports are never overwritten)')
            return EXIT_REFUSED
    if args.interface_only:
        failed = bool(report['failures'])
        return EXIT_NOT_READY if failed else EXIT_READY
    return EXIT_READY if report['ready'] else EXIT_NOT_READY


def _ros_probe(args):
    """Create one read-only node, observe the graph, destroy the node."""
    ros = RosApi()
    ros.rclpy.init()
    node = ros.rclpy.create_node('m6_live_preflight')
    try:
        return GraphProbe(node, ros, timeout_s=args.timeout, window_s=args.window).collect()
    finally:
        node.destroy_node()
        ros.rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
