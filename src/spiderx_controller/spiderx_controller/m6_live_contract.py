"""M6.0-D live-playback contract: owner-approved limits D1-D17 and the operator confirmation.

docs/M6D_LIVE_PLAYBACK_PLAN.md section 16 is the source of every value here. The M6.0 envelope
(m6_envelope) still governs the trajectory itself; this module adds only the live-dispatch limits.
Pure Python: importing it starts nothing.

LIVE DISPATCH IS HARD-DISABLED (LIVE_DISPATCH_ENABLED = False). Enabling it requires a separate,
owner-approved change after a final audit; no option, flag or environment variable overrides it.
The enabling change is exactly the one line below plus the test expectation in
test/m6d_gate.py (docs/M6D_LIVE_ENABLING_DESIGN.md); every other gate stays in force.
"""

from spiderx_controller import m6_envelope as env

# ---- THE single live gate: no live goal can be dispatched while this is False ----
# Read only by m6_live_playback.main() (before any ROS import or input), _live_main() and
# _run_live() (defensive re-checks).
LIVE_DISPATCH_ENABLED = False
LIVE_DISPATCH_DISABLED_MESSAGE = (
    'Live M6.0-D dispatch is HARD-DISABLED in this build. Sending the one live goal needs a '
    'separate owner authorization and an enabling code change after a final audit '
    '(docs/M6D_LIVE_PLAYBACK_PLAN.md section 16). Nothing was sent.')

# ---- D16: explicit goal velocity tolerance (M6.0-D goal only; no controller-YAML change) ----
GOAL_VELOCITY_TOLERANCE_RAD_S = 0.05
# D8: controller-side path and goal POSITION tolerances = the tracking tolerance
GOAL_POSITION_TOLERANCE_RAD = env.TRACKING_TOLERANCE_RAD
PATH_POSITION_TOLERANCE_RAD = env.TRACKING_TOLERANCE_RAD
# D8: client-side independent abort threshold (one cancel, terminal failure)
CLIENT_TRACKING_ABORT_RAD = env.TRACKING_TOLERANCE_RAD
GOAL_TIME_TOLERANCE_S = env.GOAL_TIME_TOLERANCE_S          # 1.0 s

# ---- D9: waits ----
GOAL_RESPONSE_TIMEOUT_S = 10.0
CANCEL_RESPONSE_TIMEOUT_S = 5.0
FINAL_STATUS_AFTER_CANCEL_S = 5.0
RESULT_WATCHDOG_S = 120.0
SERVER_WAIT_S = 10.0

# ---- D3: same-process readiness ----
READINESS_MAX_AGE_S = 10.0

# ---- D6 / D7: live stream thresholds ----
JOINT_STATES_STALE_S = 0.5          # wall time without a /joint_states message -> cancel only
SIM_STALL_S = 5.0                   # wall time without sim-time progress -> cancel only
SAMPLE_GAP_S = 0.25                 # sim-time gap between samples -> tracking failure
CONTROLLER_CHECK_PERIOD_S = 1.0     # action-server / controller presence check while in flight
POST_RESULT_SETTLE_S = env.SETTLE_S  # sim seconds observed after the result
FEEDBACK_OFFSET_LIMIT_S = 0.1       # |feedback start estimate - acceptance| above this is reported

# ---- the one approved content (D1 of M6.0; plan section 1) ----
APPROVED_POINT_TIMES_S = (3.0, 6.0, 9.0)
APPROVED_LABELS = env.WAYPOINT_LABELS
APPROVED_GOAL_COUNT = 1

# ---- D10: typed operator confirmation (no --yes bypass) ----
CONFIRMATION_WORD = 'SEND-ONE-CROUCH-GOAL'


def parse_confirmation(read_line):
    """True only if read_line() returns exactly CONFIRMATION_WORD (one trailing newline allowed).

    Refused: empty input, whitespace, any other text, a different case, leading or trailing
    spaces, EOF (EOFError or '') and an interrupt (KeyboardInterrupt). Never raises.
    """
    try:
        line = read_line()
    except (EOFError, KeyboardInterrupt):
        return False
    except Exception:  # noqa: BLE001 - any input failure refuses
        return False
    if not isinstance(line, str) or line == '':
        return False
    if line.endswith('\n'):
        line = line[:-1]
        if line.endswith('\r'):
            line = line[:-1]
    return line == CONFIRMATION_WORD


def as_dict():
    return {
        'live_dispatch_enabled': LIVE_DISPATCH_ENABLED,
        'goal_velocity_tolerance_rad_s': GOAL_VELOCITY_TOLERANCE_RAD_S,
        'goal_position_tolerance_rad': GOAL_POSITION_TOLERANCE_RAD,
        'path_position_tolerance_rad': PATH_POSITION_TOLERANCE_RAD,
        'client_tracking_abort_rad': CLIENT_TRACKING_ABORT_RAD,
        'goal_time_tolerance_s': GOAL_TIME_TOLERANCE_S,
        'goal_response_timeout_s': GOAL_RESPONSE_TIMEOUT_S,
        'cancel_response_timeout_s': CANCEL_RESPONSE_TIMEOUT_S,
        'final_status_after_cancel_s': FINAL_STATUS_AFTER_CANCEL_S,
        'result_watchdog_s': RESULT_WATCHDOG_S,
        'readiness_max_age_s': READINESS_MAX_AGE_S,
        'joint_states_stale_s': JOINT_STATES_STALE_S,
        'sim_stall_s': SIM_STALL_S,
        'sample_gap_s': SAMPLE_GAP_S,
        'controller_check_period_s': CONTROLLER_CHECK_PERIOD_S,
        'approved_point_times_s': list(APPROVED_POINT_TIMES_S),
        'approved_goal_count': APPROVED_GOAL_COUNT,
    }
