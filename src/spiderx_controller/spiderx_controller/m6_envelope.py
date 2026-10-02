"""M6.0 safety envelope: the fixed limits for the ONE approved M6.0-D playback.

Owner decisions D1-D7 and option (i) are recorded in docs/M6_GAIT_PLAYBACK_SAFETY_PLAN.md
section 14. Three quantities are deliberately SEPARATE and must never be swapped:

  1. M6_0_D_MAX_DISPLACEMENT_RAD (0.1223 rad): the maximum COMMANDED per-joint displacement from
     verified neutral, for the single neutral -> crouch_10mm -> neutral trajectory only. It is the
     exact M4-validated crouch_10mm delta (0.12229413600889982 rad) and never widens on its own.
  2. TRACKING_TOLERANCE_RAD (0.05 rad): observed /joint_states versus commanded positions, and the
     per-joint path/goal tolerances placed in the FollowJointTrajectory goal. Not a command cap.
  3. The joint-limit soft margin (0.05 rad): read from spiderx_legs.yaml soft_limit_margin_rad
     (= joint_safety.DEFAULT_MARGIN_RAD) and used only for the client-side URDF-limit check.

Values that come from earlier milestones are pinned to their sources by unit tests.
Pure Python constants: no ROS imports.
"""

# 1. Commanded displacement cap (owner option (i), M6.0-D only)
M6_0_D_MAX_DISPLACEMENT_RAD = 0.1223
# Documented numerical epsilon: a displacement d is accepted iff d <= 0.1223 + 1e-9. It absorbs
# float round-off only; it is many orders of magnitude below any physical joint motion.
DISPLACEMENT_EPSILON_RAD = 1e-9

# 2. Tracking tolerance (D3 channels 2 and 3; = the M1 tracking tolerance)
TRACKING_TOLERANCE_RAD = 0.05

# 3. Joint-limit soft margin: NOT defined here. It is loaded from spiderx_legs.yaml
#    (soft_limit_margin_rad) and must equal joint_safety.DEFAULT_MARGIN_RAD.

# D4 structural envelope
MAX_POINTS = 5
MAX_DURATION_S = 30.0

# Pacing (existing M3/M4 motion rules, m3_kinematics_targets.yaml motion and m4_pose_validation)
MIN_SEGMENT_S = 3.0          # = m4_pose_validation.MIN_DURATION_S
SPEED_FACTOR = 2.0           # = m4_pose_validation.SPEED_FACTOR
SETTLE_S = 1.0               # = m4_pose_validation.SETTLE_S
# Joint speed limit: loaded from spiderx_legs.yaml motion_constraints.max_joint_velocity_rad_s
# (0.5 rad/s SIMULATION_PLACEHOLDER), never defined here.

# Start-pose tolerance for the LIVE read-only preflight only (= m4_pose_validation
# START_POSE_TOL_RAD). A fourth, separate quantity: how close the observed start must be to
# neutral before M6.0-D may be considered. Not a command cap, not tracking, not a limit margin.
START_POSE_TOLERANCE_RAD = 0.05

# Goal-message goal_time_tolerance: how long after the trajectory end the controller waits for the
# goal tolerance before aborting with GOAL_TOLERANCE_VIOLATED. Set to the existing settle time; a
# value of 0 would mean "unchecked" in the installed joint_trajectory_controller.
GOAL_TIME_TOLERANCE_S = SETTLE_S

# Controller interface (audited, docs/M6_GAIT_PLAYBACK_SAFETY_PLAN.md section 3)
CONTROLLER_NAME = 'leg_trajectory_controller'
ACTION_NAME = '/leg_trajectory_controller/follow_joint_trajectory'
ACTION_TYPE = 'control_msgs/action/FollowJointTrajectory'
JOINT_STATES_TOPIC = '/joint_states'
REQUIRED_CONTROLLERS = ('joint_state_broadcaster', 'leg_trajectory_controller')

# The single approved M6.0-D content (D1)
NEUTRAL_LABEL = 'neutral'
CROUCH_POSE = 'crouch_10mm'
WAYPOINT_LABELS = (NEUTRAL_LABEL, CROUCH_POSE, NEUTRAL_LABEL)

NON_CLAIMS = (
    'M6.0 is a trajectory-execution and observability check only.',
    'It does not show gait playback, walking, locomotion, contact, body support, balance, '
    'dynamic stability, navigation, real-time behaviour, actuator capability or hardware '
    'readiness.',
)


def displacement_within_cap(displacement_rad):
    """True iff |displacement| <= the M6.0-D cap plus the documented epsilon."""
    return abs(displacement_rad) <= M6_0_D_MAX_DISPLACEMENT_RAD + DISPLACEMENT_EPSILON_RAD


def as_dict():
    """The envelope as recorded in every preflight report (never read back from a file)."""
    return {
        'max_commanded_displacement_rad': M6_0_D_MAX_DISPLACEMENT_RAD,
        'displacement_epsilon_rad': DISPLACEMENT_EPSILON_RAD,
        'tracking_tolerance_rad': TRACKING_TOLERANCE_RAD,
        'max_points': MAX_POINTS,
        'max_duration_s': MAX_DURATION_S,
        'min_segment_s': MIN_SEGMENT_S,
        'speed_factor': SPEED_FACTOR,
        'start_pose_tolerance_rad': START_POSE_TOLERANCE_RAD,
        'goal_time_tolerance_s': GOAL_TIME_TOLERANCE_S,
    }
