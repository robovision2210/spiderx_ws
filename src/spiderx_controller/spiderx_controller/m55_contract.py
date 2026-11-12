"""M5.5 continuous locomotion and keyboard teleoperation: the dispatch gate and fixed contract.

CONTINUOUS LOCOMOTION DISPATCH IS HARD-DISABLED (M55_LOCOMOTION_DISPATCH_ENABLED = False).

This is a SEPARATE gate from the M6.0-D gate (m6_live_contract.LIVE_DISPATCH_ENABLED) and the
M6.1 gate (m61_live_contract.M61_LIVE_DISPATCH_ENABLED). It is a single module-level literal: no
CLI flag, environment variable, parameter, service, keyboard key or arming sequence reads or
changes it. While it is False:
  * m55_rclpy.RclpyPhaseTransport refuses to exist, so no trajectory action client is created;
  * the locomotion session runs in SHADOW mode: it validates commands, runs the state machine and
    the gait sequencing and records the phase goals it WOULD send, but sends nothing.
Tests exercise the enabled logic only against the in-memory FakeLocomotionTransport, with the
gate set True inside the test (tests/m55_gate.py pins the literal).

Enabling it is a separate, reviewed, owner-approved change after the local acceptance phases
(docs/M55_KEYBOARD_WALKING.md); it is not part of this milestone.
"""

M55_LOCOMOTION_DISPATCH_ENABLED = False

M55_DISPATCH_DISABLED_MESSAGE = (
    'continuous locomotion dispatch is hard-disabled in this build '
    '(m55_contract.M55_LOCOMOTION_DISPATCH_ENABLED = False); the session runs in shadow mode '
    'and sends nothing')

MILESTONE = 'M5.5'
NODE_NAME = 'spiderx_locomotion'
SERVICE_PREFIX = '/spiderx/locomotion'
SERVICES = ('arm', 'disarm', 'stop', 'estop', 'reset', 'home')
STATUS_TOPIC = '/spiderx/locomotion/status'

NON_CLAIMS = (
    'Offline, mock and isolated-domain verification only: nothing here has moved a robot.',
    'The crawl static margin is a quasi-static approximation (CAD masses, point feet, flat '
    'ground, no slip, no dynamics); it is not a proof of balance.',
    'Fault cancellation holds the joints; it does not guarantee physical balance or a safe hold.',
    'Simulator ground truth used by the development monitor is not odometry or state estimation '
    '(M7).',
    'The joint-speed bound is a development bound, not an actuator rating; URDF effort/velocity '
    'values are placeholders.',
)

__all__ = ['M55_LOCOMOTION_DISPATCH_ENABLED', 'M55_DISPATCH_DISABLED_MESSAGE', 'MILESTONE',
           'NODE_NAME', 'SERVICE_PREFIX', 'SERVICES', 'STATUS_TOPIC', 'NON_CLAIMS']
