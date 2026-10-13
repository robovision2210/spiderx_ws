"""M6.1 live contract: the M6.1 gate, the confirmation word and the owner-approved values.

Owner approval (2026-10-03, docs/M61_TROT_REPLAY_DESIGN.md section 7): 2 cm step, 6 mm lift,
4.0 s cycle, fixed / clamped base, 4.5 cm body-height gate, a separate m61_limits.yaml. Pure
Python: importing it starts nothing.

LIVE M6.1 DISPATCH IS HARD-DISABLED (M61_LIVE_DISPATCH_ENABLED = False). It is a SEPARATE gate from
the M6.0-D gate (m6_live_contract.LIVE_DISPATCH_ENABLED), which M6.1 never reads or changes.
Enabling M6.1 needs a separate owner approval, a final audit and an enabling change on a branch
that is never merged; no option, flag or environment variable overrides it. The test expectation
is pinned in test/m61_gate.py.
"""

from spiderx_controller import m6_live_contract as m6d

# ---- THE M6.1 live gate: no M6.1 goal can be dispatched while this is False ----
# Read only by m6_gait_replay.main() (before any ROS import or input), _live_main() and
# _run_live() (defensive re-checks).
M61_LIVE_DISPATCH_ENABLED = False
M61_LIVE_DISPATCH_DISABLED_MESSAGE = (
    'Live M6.1 trot-cycle dispatch is HARD-DISABLED in this build. Sending the one live goal '
    'needs a separate owner authorization and an enabling code change after a final audit '
    '(docs/M61_IMPLEMENTATION_NOTES.md, verification checklist). Nothing was sent.')

# ---- typed operator confirmation (no --yes bypass) ----
CONFIRMATION_WORD = 'SEND-ONE-TROT-CYCLE'
APPROVED_GOAL_COUNT = 1

# ---- the owner-approved values; config/m61_limits.yaml must equal these exactly ----
APPROVED_LIMITS = {
    'max_trajectory_points': 9,
    'min_segment_duration_s': 0.5,
    'max_duration_s': 10.0,
    'lead_in_s': 3.0,
    'max_joint_displacement_rad': 0.1223,
    'displacement_epsilon_rad': 1e-9,
    'max_planned_joint_speed_rad_s': 0.25,
    'source_match_rad': 1e-8,
    'spline_check_samples_per_segment': 50,
    'path_position_tolerance_rad': 0.05,
    'goal_position_tolerance_rad': 0.05,
    'goal_velocity_tolerance_rad_s': 0.05,
    'goal_time_tolerance_s': 1.0,
    'client_tracking_abort_rad': 0.05,
    'body_min_height_m': 0.045,
    'body_max_tilt_rad': 0.26,
    'body_gate_debounce_samples': 2,
    'body_pose_required': True,
    'body_pose_stale_s': 1.0,
    'joint_limit_fraction': 0.8,
    'sim_stall_s': 5.0,
    'joint_state_gap_s': 0.25,
    'joint_states_stale_s': 0.5,
    'drift_lateral_report_m': 0.02,
    'drift_yaw_report_rad': 0.1,
    'contact_sensing': 'not_measured',
    'base_constraint': 'fixed',
}

# ---- the owner-approved gait; config/m61_trot_cycle.yaml must equal these exactly ----
APPROVED_GAIT = {
    'gait_name': 'trot',
    'variant': 'start_stop_single_cycle',
    'num_cycles': 1,
    'num_waypoints': 9,
    'cycle_duration_s': 4.0,
    'step_length_m': 0.02,
    'lift_height_m': 0.006,
    'stance_height_offset_m': 0.0,
    'lead_in_s': 3.0,
    'segments': 8,
    'duty_factor': 0.5,
    'swing_pair_first': ['front_left', 'rear_right'],
    'swing_pair_second': ['front_right', 'rear_left'],
    'base_constraint': 'fixed',
}
APPROVED_CONTENT_SHA256 = '94a492c43fcc046125d6bc71ee7f1d9a8a5d8466f1a1bdd9958a16044dd58e64'
# Quoted in the implementation request; identifies the design-note table for 2.5 cm / 10 mm, which
# was NOT approved (0.1517 rad > the 0.1223 rad cap). Recorded for traceability only.
DESIGN_NOTE_REFERENCE_SHA256 = 'd03e80a0e733ae756ed95433514aea5490ac627ffb6b18f5b47d2816174e876c'

# ---- M6.0-D supervision re-used unchanged (m6_live_playback.LiveSession / StreamMonitor) ----
# The re-used loop reads these from m6_live_contract; m61_limits.yaml must agree with them.
REUSED_M6D_VALUES = {
    'sim_stall_s': ('SIM_STALL_S', m6d.SIM_STALL_S),
    'joint_state_gap_s': ('SAMPLE_GAP_S', m6d.SAMPLE_GAP_S),
    'joint_states_stale_s': ('JOINT_STATES_STALE_S', m6d.JOINT_STATES_STALE_S),
    'client_tracking_abort_rad': ('CLIENT_TRACKING_ABORT_RAD', m6d.CLIENT_TRACKING_ABORT_RAD),
    'goal_time_tolerance_s': ('GOAL_TIME_TOLERANCE_S', m6d.GOAL_TIME_TOLERANCE_S),
    'goal_velocity_tolerance_rad_s': ('GOAL_VELOCITY_TOLERANCE_RAD_S',
                                      m6d.GOAL_VELOCITY_TOLERANCE_RAD_S),
}

NON_CLAIMS = (
    'M6.1 replays ONE offline-validated trot-cycle joint trajectory and checks joint tracking and '
    'the gait safety gates. Simulation only, placeholder actuators.',
    'It does not show walking, locomotion, translation, balance, dynamic stability, contact or '
    'slip (contact is not measured), energy, real-time behaviour, actuator capability or hardware '
    'readiness.',
)


def limits_problems(limits):
    """[(code, message)] for every limit that differs from the approved value. Empty = approved."""
    have = limits.as_dict()
    out = []
    for k, v in APPROVED_LIMITS.items():
        if k not in have or type(have[k]) is not type(v) or have[k] != v:
            out.append(('limits_not_approved', f'{k} = {have.get(k)!r}, approved {v!r}'))
    for k, (name, value) in REUSED_M6D_VALUES.items():
        if have.get(k) != value:
            out.append(('limits_inconsistent_with_m6d_monitor',
                        f'{k} = {have.get(k)!r} but the re-used M6.0-D monitor uses '
                        f'm6_live_contract.{name} = {value!r}'))
    return out


def parse_confirmation(read_line):
    """True only if read_line() returns exactly CONFIRMATION_WORD (one trailing newline allowed).

    Same rules as the M6.0-D confirmation: refused for empty input, whitespace, other text, a
    different case, surrounding spaces, EOF ('' or EOFError) and an interrupt. Never raises.
    The M6.0-D word (SEND-ONE-CROUCH-GOAL) is refused here like any other text.
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
        'm61_live_dispatch_enabled': M61_LIVE_DISPATCH_ENABLED,
        'confirmation_word': CONFIRMATION_WORD,
        'approved_goal_count': APPROVED_GOAL_COUNT,
        'approved_content_sha256': APPROVED_CONTENT_SHA256,
        'design_note_reference_sha256': DESIGN_NOTE_REFERENCE_SHA256,
        'reused_m6d_supervision': {
            'goal_response_timeout_s': m6d.GOAL_RESPONSE_TIMEOUT_S,
            'cancel_response_timeout_s': m6d.CANCEL_RESPONSE_TIMEOUT_S,
            'final_status_after_cancel_s': m6d.FINAL_STATUS_AFTER_CANCEL_S,
            'result_watchdog_s': m6d.RESULT_WATCHDOG_S,
            'server_wait_s': m6d.SERVER_WAIT_S,
            'readiness_max_age_s': m6d.READINESS_MAX_AGE_S,
            'joint_states_stale_s': m6d.JOINT_STATES_STALE_S,
            'sim_stall_s': m6d.SIM_STALL_S,
            'sample_gap_s': m6d.SAMPLE_GAP_S,
            'controller_check_period_s': m6d.CONTROLLER_CHECK_PERIOD_S,
            'post_result_settle_s': m6d.POST_RESULT_SETTLE_S,
        },
    }


__all__ = ['M61_LIVE_DISPATCH_ENABLED', 'M61_LIVE_DISPATCH_DISABLED_MESSAGE', 'CONFIRMATION_WORD',
           'APPROVED_LIMITS', 'APPROVED_GAIT', 'APPROVED_CONTENT_SHA256', 'NON_CLAIMS',
           'limits_problems', 'parse_confirmation', 'as_dict']
