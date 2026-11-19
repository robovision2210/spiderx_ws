#!/usr/bin/env python3
"""S4e observer for human keyboard input (docs/S4E_CONTRACT.md 4.2, 6, 7 and 12; frozen first).

The operator runs this in the terminal they will type into. It starts the real teleop
(`ros2 run spiderx_controller m55_teleop_keyboard`) on a pseudo-terminal of its own and sits on
the teleop's input path:
  - every chunk the operator's terminal delivers is timestamped (t_read), written UNCHANGED and in
    order to the teleop (t_written), and recorded with its bytes and the actions the teleop's own
    KeyDecoder decodes from it;
  - everything the teleop prints is passed to the operator's screen and recorded with its time.

IT NEVER SENDS A KEY. Its only write to the teleop is in Relay._relay_operator(): the bytes it has
just read from the operator's terminal. It has no key list, no repeat generator and no option to
supply keys; it publishes nothing and requests no motion. After it relays a quit (q, Ctrl+C,
Ctrl+D), and after any stop condition, it relays nothing more (input closed).

Fails closed. It refuses to start (exit 5; nothing started, the terminal untouched) unless its
standard input and output are an interactive terminal in the foreground, --out is new, rclpy is
available, and the three dispatch gates match --mode in a fresh interpreter AND in this process:
  shadow: all three False;  live: exactly M55 True, M61 False, M6.0-D False.
While running it reads /spiderx/locomotion/status and /joint_states (read-only) and stops on the
contract's section 3 conditions: travel > 0.15 m from the arm pose; tilt > 0.10 rad or z < 0.045
m in 2 consecutive fresh samples; cumulative |lateral| > 50 mm or |heading| > 0.15 rad from the
arm pose; a joint within 0.02 rad of its URDF limit; tracking > 0.05 rad; any command rejection;
FAULTED before the person's estop; more than 30 goals, a goal or level other than 0; a status in
the wrong mode; no status within 60 s, or none for 5 s; the 20-minute session cap; the teleop
exiting other than 0 after q, or not within 15 s of q; the operator's terminal closing; an
observer error; SIGINT, SIGTERM or SIGHUP.
Stop handling: input closed; the node's `estop` service called once (the S2-S4 fallback path; it
only stops and holds); the teleop ended with SIGTERM (its clean quit: stop, disarm), SIGKILL
after 15 s; the operator's terminal restored.

It prints one line of its own when 10 s of sim have passed in FAULTED after the goal's result
(the K5 hold), so the operator knows K6 may follow.

Usage: s4e_observer.py --out <new dir> --mode {shadow,live} [--domain-id 0]
Writes <out>/observer.jsonl and <out>/observer_summary.json.
Exit: 0 session complete (the teleop exited 0 after q), 2 stop condition, 3 observer error,
4 interrupted, 5 refused."""
import argparse
import fcntl
import json
import math
import os
import re
import select
import signal
import sys
import termios
import threading
import time
import tty

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.join(HERE, '..', 's3'), os.path.join(HERE, '..', 's2')]
import s4e_terminal_preflight as pf  # noqa: E402  (terminal checks; standalone)
import s3_driver as s3  # noqa: E402  (unchanged: fresh samples, debounce, body frame, gates)

SCHEMA = 'spiderx.s4e_observer/1'
HUMAN = pf.HUMAN
LEVEL = 0
MAX_GOALS = 30
TRAVEL_MAX_M = s3.TRAVEL_MAX_M                       # 0.15
TILT_MAX_RAD, Z_MIN_M = s3.TILT_MAX_RAD, s3.Z_MIN_M  # 0.10, 0.045
LATERAL_STOP_M, HEADING_STOP_RAD = 0.050, 0.15       # S4d OD4 cumulative live stops
JOINT_LIMIT_STOP_RAD = s3.JOINT_LIMIT_STOP_RAD       # 0.02
TRACKING_ABORT_RAD, TRACKING_FINDING_RAD = s3.TRACKING_ABORT_RAD, s3.TRACKING_FINDING_RAD
FIRST_STATUS_WITHIN_S, STATUS_GAP_MAX_S = 60.0, 5.0
SESSION_CAP_S = 20 * 60.0
TELEOP_EXIT_WAIT_S = 15.0
HOLD_SIM_S = 10.0
TELEOP_ARGV = ('ros2', 'run', 'spiderx_controller', 'm55_teleop_keyboard')
STATUS_TOPIC = '/spiderx/locomotion/status'
JOINT_STATES_TOPIC = '/joint_states'
ESTOP_SERVICE = '/spiderx/locomotion/estop'
GATE_NAMES = [n for _, n in s3.GATES]
MODES = {
    'shadow': {'status_mode': 'shadow', 'dispatch_gate': False,
               'gates': dict.fromkeys(GATE_NAMES, False)},
    'live': {'status_mode': 'dispatch', 'dispatch_gate': True,
             'gates': {'M55_LOCOMOTION_DISPATCH_ENABLED': True,
                       'M61_LIVE_DISPATCH_ENABLED': False, 'LIVE_DISPATCH_ENABLED': False}},
}
DRIVERS = ('s2_driver.py', 's3_driver.py', 's4a_driver.py', 's4b_driver.py', 's4c_driver.py',
           's4d_driver.py', 's4d_e_driver.py')
ESTOP, QUIT = 'estop', 'quit'          # = m55_teleop.ESTOP / QUIT (checked by the tests)
HOLD_COMPLETE = ('hold complete: 10 s of sim in FAULTED since the goal\'s result; you may now '
                 'press q (K6)')


class Stop(Exception):
    """A contract stop condition."""


class Interrupted(Exception):
    """SIGINT, SIGTERM or SIGHUP to the observer."""


def quit_cut(data, decode):
    """The length of the prefix of `data` that ends with the first byte the teleop's decoder reads
    as a quit (q, Ctrl+C, Ctrl+D), or None if there is none."""
    def has_quit(b):
        return QUIT in [a for a, _ in decode(b.decode(errors='replace'))]
    if not has_quit(data):
        return None
    return next(k for k in range(1, len(data) + 1) if has_quit(data[:k]))


def gate_refusal(gates, mode):
    """None if `gates` ({name: [value, file]}) are exactly the mode's values, else the reason."""
    want = MODES[mode]['gates']
    got = {n: (gates.get(n) or [None])[0] for n in GATE_NAMES}
    bad = {n: v for n, v in got.items() if v is not want[n]}
    return None if not bad else f'{mode}: dispatch gates not as required {want}: {bad}'


# ------------------------------------------------------------------ records
class Recorder:
    """observer.jsonl: one JSON object per line, each with the observer's monotonic time."""

    def __init__(self, path, clock=time.monotonic):
        self.f = open(path, 'x')
        self.lock = threading.Lock()
        self.clock = clock

    def __call__(self, kind, **k):
        k['kind'], k['mono'] = kind, self.clock()
        with self.lock:
            self.f.write(json.dumps(k, sort_keys=True, default=repr) + '\n')
            self.f.flush()

    def close(self):
        with self.lock:
            self.f.close()


# ------------------------------------------------------------------ the relay (no ROS)
class Relay:
    """The operator's terminal <-> the teleop's pseudo-terminal master. The ONLY write to the
    teleop is in _relay_operator(): the bytes just read from the operator, unchanged, in order."""

    def __init__(self, fd_in, fd_out, master, rec, decode, clock=time.monotonic, on_action=None):
        self.fd_in, self.fd_out, self.master = fd_in, fd_out, master
        self.rec, self.decode, self.clock = rec, decode, clock
        self.on_action = on_action or (lambda action, t: None)
        self.input_open, self.closed_why = True, None
        self.operator_gone = self.teleop_output_closed = False
        self.bytes_read = self.bytes_relayed = self.chunks = 0

    def pump(self, timeout):
        fds = [fd for fd, ok in ((self.fd_in, not self.operator_gone),
                                 (self.master, not self.teleop_output_closed)) if ok]
        if not fds:
            time.sleep(timeout)
            return
        r, _, _ = select.select(fds, [], [], timeout)
        if self.fd_in in r:
            self._relay_operator()
        if self.master in r:
            self._teleop_output()

    def _relay_operator(self):
        try:
            data = os.read(self.fd_in, 1024)
        except OSError as e:
            data, err = b'', repr(e)
        else:
            err = None
        t_read = self.clock()
        if not data:
            self.operator_gone = True
            self.rec('operator_terminal_gone', t=t_read, error=err)
            return
        self.bytes_read += len(data)
        if not self.input_open:
            self.rec('not_relayed', t_read=t_read, hex=data.hex(), n=len(data),
                     actions=self.actions(data), why=self.closed_why)
            return
        cut = quit_cut(data, self.decode)          # nothing after a quit is relayed (H8)
        data, rest = (data, b'') if cut is None else (data[:cut], data[cut:])
        view = memoryview(data)
        while view:                                   # the observer's only write to the teleop
            view = view[os.write(self.master, view):]
        t_written = self.clock()
        self.bytes_relayed += len(data)
        self.chunks += 1
        actions = self.actions(data)
        self.rec('key', t_read=t_read, t_written=t_written, hex=data.hex(), n=len(data),
                 actions=actions)
        for a in actions:
            self.on_action(a, t_read)
        if rest:
            self.close_input('after quit (q, Ctrl+C or Ctrl+D was relayed)')
            self.rec('not_relayed', t_read=t_read, hex=rest.hex(), n=len(rest),
                     actions=self.actions(rest), why=self.closed_why)

    def actions(self, data):
        return [a for a, _ in self.decode(data.decode(errors='replace'))]

    def _teleop_output(self):
        try:
            data = os.read(self.master, 65536)
        except OSError:
            data = b''
        t = self.clock()
        if not data:
            self.teleop_output_closed = True
            self.rec('teleop_output_closed', t=t)
            return
        self.show(data)
        self.rec('teleop_out', t=t, text=data.decode(errors='replace'))

    def show(self, data):
        try:
            os.write(self.fd_out, data)
        except OSError:
            pass

    def close_input(self, why):
        if self.input_open:
            self.input_open, self.closed_why = False, why
            self.rec('input_closed', t=self.clock(), why=why)


# ------------------------------------------------------------------ the stop conditions (no ROS)
class Monitor:
    """Contract section 3 over the node's statuses and the joint states. check() raises Stop."""

    def __init__(self, mode, limits, t_start, clock=time.monotonic, rec=None):
        self.mode, self.limits, self.t_start = mode, dict(limits), t_start
        self.clock, self.rec = clock, rec or (lambda *a, **k: None)
        self.lock = threading.Lock()
        self.pending, self.n_status = [], 0
        self.first_status_t = self.last_status_t = None
        self.last_fresh_sim = None
        self.tilt_db, self.fall_db = s3.Debounce(), s3.Debounce()
        self.arm_ref = None
        self.person_estop_t = None
        self.findings, self.joint_stop, self.min_joint_margin = {}, None, None
        self.max = {'travel_m': 0.0, 'abs_lateral_m': 0.0, 'abs_heading_rad': 0.0, 'tilt_rad': 0.0}
        self.min_z = None
        self.hold_from_sim, self.hold_done = None, False

    def on_status(self, st, t):
        with self.lock:
            self.pending.append((t, st))

    def on_joint_state(self, names, positions):
        worst = None
        for n, q in zip(names, positions):
            lim = self.limits.get(n)
            if lim is not None:
                m = min(q - lim[0], lim[1] - q)
                if worst is None or m < worst[0]:
                    worst = (m, n, q)
        if worst is None:
            return
        with self.lock:
            if self.min_joint_margin is None or worst[0] < self.min_joint_margin[0]:
                self.min_joint_margin = worst
            if worst[0] < JOINT_LIMIT_STOP_RAD and self.joint_stop is None:
                self.joint_stop = (f'joint {worst[1]} at {worst[2]:.4f} rad is {worst[0]:.4f} '
                                   f'rad from its URDF limit (< {JOINT_LIMIT_STOP_RAD})')

    def person_estop(self, t):
        if self.person_estop_t is None:
            self.person_estop_t = t

    def check(self, now):
        """Raise Stop on a contract stop condition; return the messages for the operator."""
        with self.lock:
            batch, self.pending = self.pending, []
            joint_stop = self.joint_stop
        msgs = []
        for t, st in batch:
            msgs += self._status(t, st)
        if joint_stop is not None:
            raise Stop(joint_stop)
        if now - self.t_start > SESSION_CAP_S:
            raise Stop(f'session cap {SESSION_CAP_S:.0f} s of wall reached')
        if self.last_status_t is None:
            if now - self.t_start > FIRST_STATUS_WITHIN_S:
                raise Stop(f'no node status within {FIRST_STATUS_WITHIN_S:.0f} s')
        elif now - self.last_status_t > STATUS_GAP_MAX_S:
            raise Stop(f'no node status for {now - self.last_status_t:.1f} s '
                       f'(> {STATUS_GAP_MAX_S:.0f} s)')
        return msgs

    def _status(self, t, st):
        m = MODES[self.mode]
        self.n_status += 1
        self.first_status_t = self.first_status_t or t
        self.last_status_t = t
        if st.get('mode') != m['status_mode'] or st.get('dispatch_gate') is not m['dispatch_gate']:
            raise Stop(f'{self.mode}: status mode {st.get("mode")!r}, dispatch_gate '
                       f'{st.get("dispatch_gate")!r} (required {m["status_mode"]!r}, '
                       f'{m["dispatch_gate"]})')
        if st.get('command_rejections'):
            raise Stop(f'command rejections {st["command_rejections"]}')
        if st.get('state') == 'FAULTED' and self.person_estop_t is None:
            raise Stop(f'FAULTED before the person\'s estop: {st.get("faults")}')
        if st.get('level') != LEVEL:
            raise Stop(f'node level {st.get("level")!r} (only level {LEVEL})')
        g = st.get('goal')
        if g is not None:
            if g.get('seq', 0) > MAX_GOALS:
                raise Stop(f'goal seq {g.get("seq")} above the budget of {MAX_GOALS}')
            if g.get('level') != LEVEL:
                raise Stop(f'goal {g.get("seq")} at level {g.get("level")!r} (only {LEVEL})')
            err = g.get('max_tracking_error_rad')
            if err is not None and err > TRACKING_ABORT_RAD:
                raise Stop(f'tracking error {err} rad above {TRACKING_ABORT_RAD} on goal '
                           f'{g.get("seq")}')
            if err is not None and err > TRACKING_FINDING_RAD and \
                    err > self.findings.get(g.get('seq'), 0.0):
                self.findings[g.get('seq')] = err
        return self._pose(st)

    def _pose(self, st):
        fp = s3.fresh_pose(st, self.last_fresh_sim)
        if fp is None:
            return []
        gt, sim = fp
        self.last_fresh_sim = sim
        msgs = []
        if st.get('state') == 'FAULTED' and st.get('goal') is None and \
                self.person_estop_t is not None and not self.hold_done:
            if self.hold_from_sim is None:
                self.hold_from_sim = sim
            elif sim - self.hold_from_sim >= HOLD_SIM_S:
                self.hold_done = True
                self.rec('hold_complete', sim_from=self.hold_from_sim, sim_to=sim)
                msgs.append(HOLD_COMPLETE)
        if self.arm_ref is None:
            if st.get('state') == 'DISARMED':
                return msgs
            self.arm_ref = {'xy': tuple(gt['xy']), 'yaw': gt['yaw']}
            self.rec('arm_reference', pose=self.arm_ref, sim=sim)
        self.min_z = gt['z'] if self.min_z is None else min(self.min_z, gt['z'])
        self.max['tilt_rad'] = max(self.max['tilt_rad'], gt['tilt'])
        if self.tilt_db.update(gt['tilt'] > TILT_MAX_RAD):
            raise Stop(f'tilt {gt["tilt"]:.4f} rad > {TILT_MAX_RAD} in 2 consecutive fresh '
                       'samples')
        if self.fall_db.update(gt['z'] < Z_MIN_M):
            raise Stop(f'body z {gt["z"]:.4f} m < {Z_MIN_M} in 2 consecutive fresh samples')
        d = gt.get('delta_since_arm_xy')
        if d is not None:
            travel = math.hypot(d[0], d[1])
            self.max['travel_m'] = max(self.max['travel_m'], travel)
            if travel > TRAVEL_MAX_M:
                raise Stop(f'travel {travel:.4f} m from the arm pose > {TRAVEL_MAX_M}')
        _, lat, dyaw = s3.body_delta(self.arm_ref, {'xy': tuple(gt['xy']), 'yaw': gt['yaw']})
        self.max['abs_lateral_m'] = max(self.max['abs_lateral_m'], abs(lat))
        self.max['abs_heading_rad'] = max(self.max['abs_heading_rad'], abs(dyaw))
        if abs(lat) > LATERAL_STOP_M or abs(dyaw) > HEADING_STOP_RAD:
            raise Stop(f'cumulative drift from the arm pose: lateral {lat * 1000:.1f} mm, heading '
                       f'{dyaw:.4f} rad (stops: {LATERAL_STOP_M * 1000:.0f} mm, '
                       f'{HEADING_STOP_RAD} rad)')
        return msgs

    def summary(self):
        return {'statuses': self.n_status, 'arm_reference': self.arm_ref,
                'person_estop_t': self.person_estop_t, 'maxima': self.max, 'min_z_m': self.min_z,
                'min_joint_margin': self.min_joint_margin, 'tracking_findings': self.findings,
                'hold': {'from_sim': self.hold_from_sim, 'complete': self.hold_done}}


# ------------------------------------------------------------------ ROS (read-only + one estop)
class RosLink:
    """Subscribes to the status and the joint states; one client: the node's estop service."""

    def __init__(self, domain_id, monitor, rec):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        self.monitor, self.rec, self.Trigger = monitor, rec, Trigger
        self.ctx = rclpy.Context()
        rclpy.init(context=self.ctx, domain_id=domain_id)
        self.node = rclpy.create_node('s4e_observer', context=self.ctx)
        self.node.create_subscription(String, STATUS_TOPIC, self._on_status, 10)
        self.node.create_subscription(JointState, JOINT_STATES_TOPIC, self._on_joint_state, 50)
        self.estop_client = self.node.create_client(Trigger, ESTOP_SERVICE)
        self.ex = SingleThreadedExecutor(context=self.ctx)
        self.ex.add_node(self.node)
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        try:
            self.ex.spin()
        except Exception:  # noqa: BLE001 - shutdown during spin
            pass

    def _on_status(self, msg):
        t = time.monotonic()
        try:
            st = json.loads(msg.data)
        except ValueError:
            self.rec('status_unparsable', t_receipt=t)
            return
        self.rec('status', t_receipt=t, status=st)
        self.monitor.on_status(st, t)

    def _on_joint_state(self, msg):
        self.monitor.on_joint_state(list(msg.name), list(msg.position))

    def call_estop(self):
        ok = self.estop_client.wait_for_service(timeout_sec=5.0)
        res = None
        if ok:
            fut = self.estop_client.call_async(self.Trigger.Request())
            end = time.monotonic() + 5.0
            while not fut.done() and time.monotonic() < end:
                time.sleep(0.02)
            res = fut.result() if fut.done() else None
        return {'available': ok, 'success': None if res is None else res.success,
                'message': None if res is None else res.message}

    def close(self):
        import rclpy
        self.ex.shutdown(timeout_sec=2.0)
        self.node.destroy_node()
        rclpy.try_shutdown(context=self.ctx)


# ------------------------------------------------------------------ processes
def spawn(argv, winsize=None):
    """The teleop on a new pseudo-terminal, as its session leader and foreground.
    Returns (pid, master fd, slave path)."""
    master, slave = os.openpty()
    path = os.ttyname(slave)
    if winsize:
        fcntl.ioctl(slave, termios.TIOCSWINSZ, winsize)
    sync_r, sync_w = os.pipe2(os.O_CLOEXEC)        # closed by the exec: the child is set up
    pid = os.fork()
    if pid == 0:
        try:
            os.close(master)
            os.close(sync_r)
            os.setsid()
            fcntl.ioctl(slave, termios.TIOCSCTTY, 0)
            for fd in (0, 1, 2):
                os.dup2(slave, fd)
            if slave > 2:
                os.close(slave)
            os.execvp(argv[0], list(argv))
        except BaseException:
            os.write(sync_w, b'exec failed')
        finally:
            os._exit(127)
    os.close(slave)
    os.close(sync_w)
    while os.read(sync_r, 64):                     # until the exec (or the child's exit)
        pass
    os.close(sync_r)
    return pid, master, path


def cmdline(pid):
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as f:
            return [a.decode(errors='replace') for a in f.read().split(b'\0') if a]
    except OSError:
        return None


def session_of(pid):
    try:
        return os.getsid(pid)
    except OSError:
        return None


def pty_holders(slave_path):
    """Processes holding the pseudo-terminal: its master (an fd to /dev/ptmx whose fdinfo
    tty-index is this pty's number) and its slave (an fd linking to slave_path). Only processes
    this user may inspect are seen."""
    index = int(slave_path.rsplit('/', 1)[1])
    masters, slaves = set(), set()
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            fds = os.listdir(f'/proc/{pid}/fd')
        except OSError:
            continue
        for fd in fds:
            try:
                link = os.readlink(f'/proc/{pid}/fd/{fd}')
            except OSError:
                continue
            if link == slave_path:
                slaves.add(int(pid))
            elif link in ('/dev/ptmx', '/dev/pts/ptmx'):
                try:
                    with open(f'/proc/{pid}/fdinfo/{fd}') as f:
                        m = re.search(r'tty-index:\s*(\d+)', f.read())
                except OSError:
                    continue
                if m and int(m.group(1)) == index:
                    masters.add(int(pid))
    return {'slave': slave_path,
            'master_holders': [{'pid': p, 'cmd': cmdline(p)} for p in sorted(masters)],
            'slave_holders': [{'pid': p, 'sid': session_of(p), 'cmd': cmdline(p)}
                              for p in sorted(slaves)]}


def driver_scan():
    """Running S2-S4d drivers (their script name as an argument), excluding nothing."""
    out = []
    for pid in filter(str.isdigit, os.listdir('/proc')):
        c = cmdline(pid)
        if c and any(os.path.basename(a) in DRIVERS for a in c):
            out.append({'pid': int(pid), 'cmd': c})
    return out


# ------------------------------------------------------------------ the session
class Observer:
    def __init__(self, out_dir, mode, limits, fd_in=0, fd_out=1, teleop_argv=TELEOP_ARGV,
                 ros_factory=None, gates=None, tty=None, clock=time.monotonic,
                 teleop_exit_wait_s=TELEOP_EXIT_WAIT_S, input_source=HUMAN):
        from spiderx_controller import m55_teleop as tp       # pure: the teleop's own decoder
        self.decoder = tp.KeyDecoder()
        self.out_dir, self.mode, self.limits = out_dir, mode, limits
        self.fd_in, self.fd_out, self.teleop_argv = fd_in, fd_out, tuple(teleop_argv)
        self.ros_factory, self.gates, self.tty = ros_factory, gates, tty
        self.clock, self.teleop_exit_wait_s = clock, teleop_exit_wait_s
        self.input_source = input_source
        self.rec = Recorder(os.path.join(out_dir, 'observer.jsonl'), clock)
        self.pid = self.master = self.slave_path = None
        self.ros = None
        self.quit_t = self.exit_status = self.exit_t = None
        self.interrupted_by = None
        self.stop_reason = None
        self.monitor = None
        self.relay = None

    def say(self, text):
        self.rec('say', text=text)
        if self.relay is not None:
            self.relay.show(f'\r\n[observer] {text}\r\n'.encode())

    def on_action(self, action, t):
        if action == ESTOP:
            self.monitor.person_estop(t)
        elif action == QUIT and self.quit_t is None:
            self.quit_t = t
            self.relay.close_input('after quit (q, Ctrl+C or Ctrl+D was relayed)')

    def interrupt(self, name):
        self.interrupted_by = name

    def poll_teleop(self):
        if self.exit_status is not None or self.pid is None:
            return
        p, st = os.waitpid(self.pid, os.WNOHANG)
        if p:
            self.exit_status, self.exit_t = os.waitstatus_to_exitcode(st), self.clock()
            self.rec('teleop_exit', t=self.exit_t, code=self.exit_status)

    def loop(self):
        while True:
            self.relay.pump(0.05)
            now = self.clock()
            if self.interrupted_by:
                raise Interrupted(self.interrupted_by)
            for m in self.monitor.check(now):
                self.say(m)
            if self.relay.operator_gone:
                raise Stop('the operator\'s terminal closed (end of input or hang-up)')
            self.poll_teleop()
            if self.exit_status is not None:
                if self.quit_t is not None and self.exit_status == 0:
                    return
                raise Stop(f'the teleop exited with {self.exit_status} '
                           f'{"after" if self.quit_t is not None else "without"} a quit')
            if self.quit_t is not None and now - self.quit_t > self.teleop_exit_wait_s:
                raise Stop(f'the teleop did not exit within {self.teleop_exit_wait_s:.0f} s of '
                           'the quit')

    def end_teleop(self):
        """SIGTERM to the teleop's session (its clean quit), SIGKILL after the wait."""
        self.poll_teleop()
        if self.exit_status is not None or self.pid is None:
            return
        for sig, wait in ((signal.SIGTERM, self.teleop_exit_wait_s), (signal.SIGKILL, 5.0)):
            try:
                os.killpg(self.pid, sig)
            except ProcessLookupError:
                pass
            self.rec('teleop_signal', signal=sig.name)
            end = self.clock() + wait
            while self.clock() < end:
                self.relay.pump(0.05)
                self.poll_teleop()
                if self.exit_status is not None:
                    return

    def emergency(self, why):
        self.relay.close_input(f'stop: {why}')
        res = None
        if self.ros is not None:
            try:
                res = self.ros.call_estop()
            except Exception as e:  # noqa: BLE001
                res = {'error': repr(e)}
        self.rec('estop_service', why=why, result=res)
        self.say(f'STOP: {why}. Input closed; estop requested; the teleop is being ended.')
        self.end_teleop()

    def run(self):
        self.rec('start', schema=SCHEMA, mode=self.mode, input_source=self.input_source,
                 tty=self.tty, gates=self.gates, teleop_argv=list(self.teleop_argv),
                 observer_pid=os.getpid(), contract='docs/S4E_CONTRACT.md')
        saved = termios.tcgetattr(self.fd_in)
        rc, why = 0, 'session complete'
        try:
            tty.setraw(self.fd_in)
            winsize = fcntl.ioctl(self.fd_in, termios.TIOCGWINSZ, b'\0' * 8)
            self.pid, self.master, self.slave_path = spawn(self.teleop_argv, winsize)
            t0 = self.clock()
            self.rec('teleop_started', pid=self.pid, slave=self.slave_path, t=t0)
            self.relay = Relay(self.fd_in, self.fd_out, self.master, self.rec,
                               self.decoder.decode, self.clock, self.on_action)
            self.monitor = Monitor(self.mode, self.limits, t0, self.clock, self.rec)
            self.rec('pty_holders', at='start', **pty_holders(self.slave_path))
            self.rec('driver_scan', at='start', drivers=driver_scan())
            self.say(f'S4e observer ({self.mode}): your keys go to the teleop unchanged and are '
                     'recorded; the observer never sends a key.')
            if self.ros_factory is not None:
                self.ros = self.ros_factory(self.monitor, self.rec)
            self.loop()
        except Stop as e:
            rc, why = 2, f'STOP CONDITION: {e}'
        except Interrupted as e:
            rc, why = 4, f'INTERRUPTED: {e}'
        except Exception as e:  # noqa: BLE001 - an observer bug must still close everything
            rc, why = 3, f'OBSERVER ERROR: {e!r}'
        if rc != 0:
            self.stop_reason = why
            if self.relay is not None:
                try:
                    self.emergency(why)
                except Exception as e:  # noqa: BLE001
                    self.rec('emergency_failed', error=repr(e))
        try:
            if self.slave_path:
                self.rec('pty_holders', at='end', **pty_holders(self.slave_path))
            self.rec('driver_scan', at='end', drivers=driver_scan())
        except Exception as e:  # noqa: BLE001
            self.rec('end_scan_failed', error=repr(e))
        try:
            termios.tcsetattr(self.fd_in, termios.TCSADRAIN, saved)
            restored = termios.tcgetattr(self.fd_in) == saved
        except termios.error as e:          # e.g. the operator's terminal has hung up
            restored = False
            self.rec('terminal_restore_failed', error=repr(e))
        summary = {
            'schema': SCHEMA, 'result': why, 'exit_code': rc, 'mode': self.mode,
            'input_source': self.input_source, 'tty': self.tty, 'gates': self.gates,
            'teleop': {'pid': self.pid, 'exit_code': self.exit_status, 'exit_t': self.exit_t,
                       'quit_t': self.quit_t},
            'relay': None if self.relay is None else {
                'bytes_read': self.relay.bytes_read, 'bytes_relayed': self.relay.bytes_relayed,
                'chunks_relayed': self.relay.chunks, 'input_closed_why': self.relay.closed_why},
            'monitor': None if self.monitor is None else self.monitor.summary(),
            'terminal_restored': restored}
        self.rec('end', summary=summary)
        with open(os.path.join(self.out_dir, 'observer_summary.json'), 'x') as f:
            json.dump(summary, f, indent=1, sort_keys=True, default=repr)
        if self.ros is not None:
            try:
                self.ros.close()
            except Exception as e:  # noqa: BLE001
                self.rec('ros_close_failed', error=repr(e))
        self.rec.close()
        if self.master is not None:
            try:
                os.close(self.master)
            except OSError:
                pass
        return rc, summary


def refuse(out_dir, why, extra):
    print(f'REFUSED (nothing started): {why}', file=sys.stderr, flush=True)
    if out_dir is not None:
        os.makedirs(out_dir)
        with open(os.path.join(out_dir, 'observer_summary.json'), 'x') as f:
            json.dump({'schema': SCHEMA, 'result': f'REFUSED: {why}', 'exit_code': 5, **extra},
                      f, indent=1, sort_keys=True, default=repr)
    return 5


def main(argv=None):
    a = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    a.add_argument('--out', required=True, help='a NEW directory for the record')
    a.add_argument('--mode', required=True, choices=sorted(MODES))
    a.add_argument('--domain-id', type=int, default=0)
    o = a.parse_args(argv)
    if os.path.exists(o.out):
        return refuse(None, f'{o.out} exists (a record is never overwritten)', {})
    checks = pf.tty_checks()
    extra = {'mode': o.mode, 'tty': checks}
    why = pf.refusal(checks)
    if why:
        return refuse(o.out, why, extra)
    try:
        gates = {'fresh_interpreter': s3.fresh_gates(), 'observer_process': s3.imported_gates()}
    except Exception as e:  # noqa: BLE001 - no gate evidence: refuse
        return refuse(o.out, f'gate import failed: {e!r}', extra)
    extra['gates'] = gates
    why = (gate_refusal(gates['fresh_interpreter'], o.mode)
           or gate_refusal(gates['observer_process'], o.mode))
    if why:
        return refuse(o.out, why, extra)
    try:
        import rclpy  # noqa: F401
        from spiderx_controller import joint_safety
        limits, _ = joint_safety.load_limits()
    except Exception as e:  # noqa: BLE001
        return refuse(o.out, f'ROS or the joint limits unavailable: {e!r}', extra)
    os.makedirs(o.out)
    obs = Observer(o.out, o.mode, limits, gates=gates, tty=checks,
                   ros_factory=lambda mon, rec: RosLink(o.domain_id, mon, rec))
    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(s, lambda signum, frame: obs.interrupt(signal.Signals(signum).name))
    rc, summary = obs.run()
    print(f'\n[observer] {summary["result"]} (exit {rc}); record: {o.out}', flush=True)
    sys.stdout.flush()
    os._exit(rc)        # skip interpreter teardown: the C++ executor can abort there (as S3)


if __name__ == '__main__':
    sys.exit(main())
