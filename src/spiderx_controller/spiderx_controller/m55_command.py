"""M5.5 operator intent: velocity-command validation, frame mapping and the renewable lease.

Pure Python (times are injected; no ROS graph).

Command convention (REP-103, the repository's /cmd_vel architecture, geometry_msgs/Twist):
    linear.x  forward  (m/s, + = robot front)
    linear.y  left     (m/s)
    angular.z yaw rate (rad/s, + = counter-clockwise seen from above)
SpiderX body frame (base_link, NOT REP-103): +x = robot RIGHT, +y = robot FRONT, +z = up
(verified: lidar_link has yaw +90 deg in base_link and its +x is the robot front). Hence
    v_base = R(+90 deg) . v_rep:   forward (1, 0) -> base +y,   left (0, 1) -> base -x,
and a yaw rate is the same number in both frames (both right-handed about +z).

Supported by the current gait (m55_crawl): forward and reverse along base +y only, at the discrete
validated speed levels. Lateral (linear.y) and turning (angular.z) are REJECTED explicitly, never
silently executed as something else. A requested speed maps to the fastest validated level that
does not exceed it (never faster than asked); below the deadband, or below the slowest level, it
means "stop".

Lease (operator intent must be renewed): a valid command renews the lease at its RECEIPT time
(monotonic; Twist has no stamp). If no valid command arrives for lease_s the intent is
'lease_expired' and the session must stop - the last non-zero command is never kept alive.
Invalid commands (non-finite, unsupported components, too fast a rate) do not renew the lease.
"""

from dataclasses import dataclass
import math

FORWARD, REVERSE, STOPPED = 1, -1, 0

# rejection codes (each counted separately)
NONFINITE = 'command_nonfinite'
UNSUPPORTED_LATERAL = 'command_unsupported_lateral'
UNSUPPORTED_YAW = 'command_unsupported_yaw'
UNSUPPORTED_OTHER = 'command_unsupported_axis'
RATE_EXCEEDED = 'command_rate_exceeded'


def rep103_to_base(vx, vy):
    """REP-103 (forward, left) -> base_link (x right, y front)."""
    return (-vy, vx)


def base_to_rep103(bx, by):
    return (by, -bx)


@dataclass(frozen=True)
class Intent:
    """The validated operator request: direction and speed level index (or stop)."""
    direction: int                 # FORWARD, REVERSE or STOPPED
    level: int = None              # index into the validated speed levels (None when stopped)
    requested_m_s: float = 0.0     # |forward speed| asked for
    granted_m_s: float = 0.0       # speed of the chosen level

    @property
    def moving(self):
        return self.direction != STOPPED


STOP = Intent(STOPPED)


class CommandMapper:
    """Maps one Twist-like command to an Intent, or rejects it with a code."""

    def __init__(self, level_speeds_m_s, deadband_m_s, unsupported_tol):
        speeds = list(level_speeds_m_s)
        if not speeds or any(not (math.isfinite(s) and s > 0) for s in speeds) or \
                any(b <= a for a, b in zip(speeds, speeds[1:])):
            raise ValueError('speed levels must be positive, finite and strictly increasing')
        self.speeds = speeds
        self.deadband = float(deadband_m_s)
        self.tol = float(unsupported_tol)

    def map(self, linear, angular):
        """(Intent or None, rejection code or None). linear/angular: (x, y, z) tuples."""
        vals = tuple(linear) + tuple(angular)
        if len(vals) != 6 or not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                                     and math.isfinite(v) for v in vals):
            return None, NONFINITE
        vx, vy, vz = linear
        wx, wy, wz = angular
        if abs(vy) > self.tol:
            return None, UNSUPPORTED_LATERAL
        if abs(wz) > self.tol:
            return None, UNSUPPORTED_YAW
        if abs(vz) > self.tol or abs(wx) > self.tol or abs(wy) > self.tol:
            return None, UNSUPPORTED_OTHER
        speed = abs(vx)
        if speed < self.deadband:
            return STOP, None
        allowed = [i for i, s in enumerate(self.speeds) if s <= speed + 1e-12]
        if not allowed:
            # above the deadband but slower than the slowest validated level: a valid request
            # that is honoured as a stop (never faster than asked); the request is recorded
            return Intent(STOPPED, None, speed, 0.0), None
        i = allowed[-1]
        return Intent(FORWARD if vx > 0 else REVERSE, i, speed, self.speeds[i]), None


class CommandLease:
    """Renewable operator intent with a monotonic lease and a minimum message spacing."""

    def __init__(self, mapper, lease_s, max_rate_hz):
        self.mapper = mapper
        self.lease_s = float(lease_s)
        self.min_dt = 1.0 / float(max_rate_hz)
        self.intent = STOP
        self.last_valid_wall = None
        self.last_msg_wall = None
        self.rejections = {}
        self.accepted = 0

    def _reject(self, code):
        self.rejections[code] = self.rejections.get(code, 0) + 1
        return code

    def on_command(self, wall, linear, angular):
        """Process one received command (receipt time `wall`). Returns a rejection code or None.

        A rejected command does NOT renew the lease and sets the intent to STOP: an invalid
        message never leaves an earlier non-zero intent active."""
        if self.last_msg_wall is not None and wall - self.last_msg_wall < self.min_dt - 1e-9:
            self.last_msg_wall = wall
            self.intent = STOP
            return self._reject(RATE_EXCEEDED)
        self.last_msg_wall = wall
        intent, code = self.mapper.map(linear, angular)
        if code is not None:
            self.intent = STOP
            return self._reject(code)
        self.intent = intent
        self.last_valid_wall = wall
        self.accepted += 1
        return None

    def current(self, now_wall):
        """(Intent, reason). reason: None, 'no_command_yet' or 'lease_expired' (intent = STOP)."""
        if self.last_valid_wall is None:
            return STOP, 'no_command_yet'
        if now_wall - self.last_valid_wall > self.lease_s:
            return STOP, 'lease_expired'
        return self.intent, None

    def clear(self):
        """Forget the intent (used on disarm, fault and reset: re-arming needs fresh input)."""
        self.intent = STOP
        self.last_valid_wall = None


__all__ = ['FORWARD', 'REVERSE', 'STOPPED', 'Intent', 'STOP', 'CommandMapper', 'CommandLease',
           'rep103_to_base', 'base_to_rep103', 'NONFINITE', 'UNSUPPORTED_LATERAL',
           'UNSUPPORTED_YAW', 'UNSUPPORTED_OTHER', 'RATE_EXCEEDED']
