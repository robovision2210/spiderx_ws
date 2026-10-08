"""M5.5 keyboard teleoperation for the SpiderX locomotion node (`m55_teleop_keyboard`).

Keys (single key presses; no Enter):
    a        arm                    z      disarm (after a controlled stop)
    w        walk forward (hold)    s      walk in reverse (hold)
    space    controlled stop        x/Esc  EMERGENCY STOP (cancel and hold; needs r, then a)
    r        reset after a fault    h      home (walk to the nearest cycle boundary)
    + or =   faster speed level     -      slower speed level
    j / l    turning: REJECTED (the current gait walks forward and backward only)
    q        quit (controlled stop, disarm, exit)

Hold semantics never rely on key-release events (terminals do not report them): a motion key
counts as held for hold_s after its last (auto-repeated) press. The teleop publishes /cmd_vel at
heartbeat_hz: the held motion at the selected level's validated speed, otherwise a zero Twist.
The zero heartbeat renews the node's lease; if this program dies or its terminal closes, the
heartbeats stop and the node stops and disarms by itself. Reconnecting never re-arms: press a.

Speeds come from the node's status (`levels_m_s`); until a status arrives, motion keys send zero.
Only linear.x is ever set: lateral and yaw components stay zero.
"""

import json
import math
import time

from spiderx_controller import m55_contract as c55

ARM, DISARM, STOP, ESTOP, RESET, HOME = c55.SERVICES
FORWARD, REVERSE = 'forward', 'reverse'
FASTER, SLOWER, TURN, QUIT = 'faster', 'slower', 'turn_rejected', 'quit'

KEYMAP = {
    'a': ARM, 'z': DISARM, ' ': STOP, 'x': ESTOP, '\x1b': ESTOP, 'r': RESET, 'h': HOME,
    'w': FORWARD, 's': REVERSE, '+': FASTER, '=': FASTER, '-': SLOWER,
    'j': TURN, 'l': TURN, 'q': QUIT,
}
SERVICE_ACTIONS = frozenset(c55.SERVICES)

HELP = """SpiderX M5.5 keyboard teleop  (simulation development; dispatch gate: {gate})
  a arm | z disarm | w forward (hold) | s reverse (hold) | space stop | x/Esc EMERGENCY STOP
  r reset after a fault | h home | +/- speed level | j/l turning: not supported | q quit"""

TURN_MESSAGE = ('turning is not supported by the current gait (forward and reverse only); '
                'nothing was sent')


class TeleopState:
    """Pure keyboard state: the held motion, the selected level and the Twist to publish."""

    def __init__(self, hold_s):
        self.hold_s = float(hold_s)
        self.motion = 0                  # +1 forward, -1 reverse, 0 none
        self.motion_wall = None
        self.level = 0
        self.levels = None               # validated speeds from the node status (m/s)

    def on_status(self, status):
        levels = status.get('levels_m_s')
        if isinstance(levels, list) and levels and all(
                isinstance(v, (int, float)) and math.isfinite(v) and v > 0 for v in levels):
            self.levels = [float(v) for v in levels]
            self.level = min(self.level, len(self.levels) - 1)

    def key(self, action, wall):
        """Apply one decoded key. Returns (service to call or None, message or None)."""
        if action in (FORWARD, REVERSE):
            self.motion = 1 if action == FORWARD else -1
            self.motion_wall = wall
            if self.levels is None:
                return None, 'no locomotion status yet: speed levels unknown, sending zero'
            return None, None
        if action == FASTER or action == SLOWER:
            if self.levels is None:
                return None, 'no locomotion status yet: speed levels unknown'
            step = 1 if action == FASTER else -1
            self.level = max(0, min(len(self.levels) - 1, self.level + step))
            return None, (f'speed level {self.level + 1}/{len(self.levels)}: '
                          f'{self.levels[self.level] * 1000:.2f} mm/s')
        if action == TURN:
            return None, TURN_MESSAGE
        if action in (STOP, ESTOP, DISARM, QUIT):
            self.motion = 0
            self.motion_wall = None
        if action in SERVICE_ACTIONS:
            return action, None
        return None, None

    def twist(self, wall):
        """(linear.x, linear.y, angular.z) to publish now. Never lateral, never yaw."""
        if self.motion and self.levels is not None and self.motion_wall is not None and \
                wall - self.motion_wall <= self.hold_s:
            return (self.motion * self.levels[self.level], 0.0, 0.0)
        if self.motion and (self.motion_wall is None or wall - self.motion_wall > self.hold_s):
            self.motion = 0                 # released (no repeat within hold_s)
        return (0.0, 0.0, 0.0)


class KeyDecoder:
    """Bytes from a cbreak terminal -> key actions. A lone Esc is the emergency stop; an escape
    SEQUENCE (arrow and function keys) is ignored and reported."""

    def __init__(self):
        self.ignored = 0

    def decode(self, data):
        out, i = [], 0
        while i < len(data):
            ch = data[i]
            if ch == '\x1b' and i + 1 < len(data) and data[i + 1] in '[O':
                j = i + 2
                while j < len(data) and not ('@' <= data[j] <= '~'):
                    j += 1
                self.ignored += 1
                out.append(('ignored', 'escape sequence (arrow/function key) ignored: use w/s'))
                i = j + 1
                continue
            if ch in KEYMAP:
                out.append((KEYMAP[ch], None))
            elif ch.lower() in ('w', 's') and ch.isupper():
                out.append((KEYMAP[ch.lower()], None))
            elif ch in ('\x03', '\x04'):            # Ctrl+C / Ctrl+D in cbreak: quit
                out.append((QUIT, None))
            i += 1
        return out


def status_line(status):
    if not status:
        return 'waiting for /spiderx/locomotion/status ...'
    intent = status.get('intent') or {}
    lv = status.get('levels_m_s') or []
    lvl = status.get('level')
    speed = f'{lv[lvl] * 1000:.2f} mm/s' if isinstance(lvl, int) and 0 <= lvl < len(lv) else '?'
    parts = [f'state {status.get("state")}', f'mode {str(status.get("mode")).upper()}',
             f'level {lvl} ({speed})', f'boundary {status.get("boundary")}',
             f'intent {intent.get("direction")}']
    if status.get('faults'):
        parts.append(f'FAULTS {",".join(status["faults"])}')
    if status.get('last_refusal'):
        parts.append(f'last refusal {status["last_refusal"].get("reason")}')
    return ' | '.join(parts)


# ======================================================================== ROS side
class TeleopNode:
    """Publishes /cmd_vel heartbeats, calls the services asynchronously, reads the status."""

    def __init__(self, cfg, context=None, node_name='spiderx_teleop_keyboard', out=print):
        import rclpy
        from geometry_msgs.msg import Twist
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        self.cfg = cfg
        self.out = out
        self.Twist, self.Trigger = Twist, Trigger
        self.node = rclpy.create_node(node_name, context=context)
        self.pub = self.node.create_publisher(Twist, cfg.command.topic, 10)
        self.clients = {name: self.node.create_client(Trigger, f'{c55.SERVICE_PREFIX}/{name}')
                        for name in c55.SERVICES}
        self.status = None
        self.node.create_subscription(String, c55.STATUS_TOPIC, self._on_status, 10)
        self.state = TeleopState(cfg.teleop.hold_s)
        self.published = 0
        self.responses = []

    def _on_status(self, msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(st, dict):
            self.status = st
            self.state.on_status(st)

    def publish(self, wall):
        vx, vy, wz = self.state.twist(wall)
        t = self.Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self.pub.publish(t)
        self.published += 1
        return vx

    def call(self, name):
        client = self.clients[name]
        if not client.service_is_ready():
            self.out(f'[{name}] service {c55.SERVICE_PREFIX}/{name} not available')
            self.responses.append((name, None, 'unavailable'))
            return None
        fut = client.call_async(self.Trigger.Request())

        def done(f, name=name):
            r = f.result()
            ok, msg = (None, 'no response') if r is None else (r.success, r.message)
            self.responses.append((name, ok, msg))
            self.out(f'[{name}] {"ok" if ok else "REFUSED"}: {msg}')
        fut.add_done_callback(done)
        return fut

    def handle(self, action, wall, note=None):
        """Apply one decoded key; returns False when the program should quit."""
        if action == 'ignored':
            self.out(note)
            return True
        service, message = self.state.key(action, wall)
        if message:
            self.out(message)
        if action == QUIT:
            self.call(STOP)
            self.call(DISARM)
            return False
        if service:
            self.call(service)
        return True

    def close(self):
        if self.node is not None:
            self.node.destroy_node()
            self.node = None


def _shutdown_sequence(tn, executor, out):
    """Best effort on quit/EOF/hang-up: zero command, stop, disarm (the lease covers the rest)."""
    try:
        tn.state.motion = 0
        tn.publish(time.monotonic())
        futs = [tn.call(STOP), tn.call(DISARM)]
        end = time.monotonic() + 1.0
        while time.monotonic() < end and any(f is not None and not f.done() for f in futs):
            executor.spin_once(timeout_sec=0.05)
    except Exception as e:  # noqa: BLE001 - leaving anyway; the node's lease stops the robot
        out(f'shutdown: {e!r} (the node stops on lease expiry)')


def main(argv=None):
    import os
    import select
    import signal
    import sys
    import termios
    import tty
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from spiderx_controller import m55_locomotion as loc

    if not sys.stdin.isatty():
        print('m55_teleop_keyboard needs an interactive terminal (key auto-repeat drives the '
              'hold window)', file=sys.stderr)
        return 2
    cfg = loc.load_config()
    rclpy.init(args=argv)
    tn = TeleopNode(cfg)
    executor = SingleThreadedExecutor()
    executor.add_node(tn.node)
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    hung_up = []
    signal.signal(signal.SIGHUP, lambda *a: hung_up.append(True))
    signal.signal(signal.SIGTERM, lambda *a: hung_up.append(True))
    decoder = KeyDecoder()
    period = 1.0 / cfg.teleop.heartbeat_hz
    print(HELP.format(gate=c55.M55_LOCOMOTION_DISPATCH_ENABLED))
    if not c55.M55_LOCOMOTION_DISPATCH_ENABLED:
        print('NOTE: the node runs in SHADOW mode in this build - nothing moves.')
    last_line = None
    try:
        tty.setcbreak(fd)
        running, next_beat = True, time.monotonic()
        while running and not hung_up and rclpy.ok():
            timeout = max(0.0, min(period, next_beat - time.monotonic()))
            ready, _, _ = select.select([fd], [], [], timeout)
            if ready:
                data = os.read(fd, 64).decode(errors='replace')
                if not data:                       # EOF: the terminal went away
                    break
                if data == '\x1b':                 # a lone Esc may be the start of a sequence
                    more, _, _ = select.select([fd], [], [], 0.03)
                    if more:
                        data += os.read(fd, 64).decode(errors='replace')
                for action, note in decoder.decode(data):
                    if not tn.handle(action, time.monotonic(), note):
                        running = False
                        break
            now = time.monotonic()
            if now >= next_beat:
                tn.publish(now)
                next_beat = now + period
            executor.spin_once(timeout_sec=0.0)
            line = status_line(tn.status)
            if line != last_line:
                print(line)
                last_line = line
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        _shutdown_sequence(tn, executor, print)
        tn.close()
        rclpy.try_shutdown()
    return 0


__all__ = ['TeleopState', 'KeyDecoder', 'TeleopNode', 'KEYMAP', 'TURN_MESSAGE', 'status_line',
           'main']
