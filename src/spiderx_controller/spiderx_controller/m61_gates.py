"""M6.1 gait-specific safety gates (design section 3.3). Pure Python; times are injected.

  G1 body height     base_link z < body_min_height_m for body_gate_debounce_samples samples
  G2 body tilt       max(|roll|, |pitch|) > body_max_tilt_rad, same debounce
  G3 joint limit     observed q beyond joint_limit_fraction of its URDF limit (one sample)
  G4 contact         NOT MEASURED: the model has no contact sensor; G1/G2 are the only proxies
  G5 sim stall       re-used M6.0-D StreamMonitor reason 'sim_time_stalled' (5.0 s)
  G6 joint-state gap re-used M6.0-D StreamMonitor reason 'sample_gap' (0.25 s)
  G7 body drift      report only: lateral |dx| and |dyaw| over the run; never a cancel
  G8 attachment      M6.1-A fixed base only: the composed body pose left the weld pose, the model
                     root left the identity spawn, or Gazebo's body-link entry no longer matches the
                     description, for attachment debounce samples -> cancel (m61a_fixed_base)
  pose freshness     while the goal runs, no body-pose sample for body_pose_stale_s -> cancel

G1 (height), G2 (tilt) and G8 (attachment displacement) are separate checks: on a raised weld a
detached body could settle near 0.0545 m, still above the 0.045 m G1 threshold, so only G8 sees it.

A trip returns a cancel reason; the session sends at most ONE cancel (the first reason) and never
a return goal. Every trip is recorded, including later ones, with its value and threshold.
"""

import math

from spiderx_controller import m61_trot_cycle as tc

G1, G2, G3, G4, G5, G6, G7 = ('G1_body_height', 'G2_body_tilt', 'G3_joint_near_limit',
                              'G4_unexpected_contact', 'G5_sim_stall', 'G6_joint_state_gap',
                              'G7_body_drift')
G8 = 'G8_attachment'
POSE_FRESHNESS = 'body_pose_freshness'
GATE_IDS = (G1, G2, G3, G4, G5, G6, G7, G8, POSE_FRESHNESS)

# cancel reasons raised by these gates (the session maps them to its GATE_TRIPPED state)
BODY_TOO_LOW, BODY_TILT, JOINT_NEAR_LIMIT, BODY_POSE_STALE = (
    'body_too_low', 'body_tilt', 'joint_near_limit', 'body_pose_stale')
ATTACHMENT_LOST = 'attachment_lost'
M61_REASONS = (BODY_TOO_LOW, BODY_TILT, JOINT_NEAR_LIMIT, BODY_POSE_STALE, ATTACHMENT_LOST)
REASON_GATE = {BODY_TOO_LOW: G1, BODY_TILT: G2, JOINT_NEAR_LIMIT: G3, BODY_POSE_STALE:
               POSE_FRESHNESS, ATTACHMENT_LOST: G8, 'sim_time_stalled': G5, 'sample_gap': G6}
# m61a_fixed_base sample codes that mean "this is no longer the verified fixed base"
INTEGRITY_CODES = ('attachment_displaced', 'frame_spawn_not_identity',
                   'frame_body_link_inconsistent')
FIXED_BASE_FRAME = ('Gazebo world; base_link = T_world_model * T_model_dummy (Gazebo entry) * '
                    'T_dummy_base (M6.1-A fixed base)')
FREE_BASE_FRAME = 'Gazebo world (model "spiderx" = base_link)'

CONTACT_NOTE = ('Not measured: the SpiderX model and world have no contact sensor. Body height '
                '(G1) and tilt (G2) are proxies only; no contact or slip claim is made.')


def _finite(*vals):
    return all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
               for v in vals)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class GateMonitor:
    """G1-G3, G7 and pose freshness over the live streams. Gates are evaluated only when the
    caller says so (gate=True: the goal is running); every sample still feeds the statistics."""

    def __init__(self, limits, joint_limits, fixed_base=None):
        self.lim = limits
        self.joint_limits = dict(joint_limits)            # {joint: (lower, upper)} URDF
        self.fb = fixed_base                               # m61a FixedBaseConfig or None (no G8)
        self._integrity = 0
        self.fb_samples = self.fb_gated_samples = 0
        self.fb_codes_seen = {}
        self.attachment_max = None                         # (translation m, rotation rad)
        self.armed_wall = None
        self.last_pose_wall = None
        self._low = self._tilt = 0
        self.trips = []
        self.pose_samples = self.gated_pose_samples = self.invalid_pose_samples = 0
        self.joint_samples = self.gated_joint_samples = 0
        self.first_pose = self.last_pose = None
        self.z_min = self.z_max = None
        self.tilt_max = None
        self.roll_max_abs = self.pitch_max_abs = None
        self.max_fraction = (0.0, None, None)              # (fraction, joint, value)
        self.extremes = {}                                 # {joint: [min, max]}

    # ------------------------------------------------------------------ state
    def arm(self, wall):
        """Start the pose-freshness clock (at goal acceptance)."""
        if self.armed_wall is None:
            self.armed_wall = wall

    @property
    def armed(self):
        return self.armed_wall is not None

    def tripped(self, gate):
        return any(t['gate'] == gate for t in self.trips)

    def _trip(self, gate, reason, value, threshold, wall, stamp, detail=None):
        if self.tripped(gate):
            return None
        self.trips.append({'gate': gate, 'reason': reason, 'value': value,
                           'threshold': threshold, 'wall_s': wall, 'stamp_s': stamp,
                           'detail': detail})
        return reason

    # ------------------------------------------------------------------ G1, G2, G7
    def on_body_pose(self, wall, stamp, pose, gate):
        """One ground-truth body pose (x, y, z, roll, pitch, yaw). Returns a cancel reason."""
        if not (isinstance(pose, (tuple, list)) and len(pose) == 6 and _finite(*pose)):
            self.invalid_pose_samples += 1                 # never refreshes the freshness clock
            return None
        x, y, z, roll, pitch, yaw = (float(v) for v in pose)
        self.pose_samples += 1
        self.last_pose_wall = wall
        if self.first_pose is None:
            self.first_pose = (x, y, z, roll, pitch, yaw)
        self.last_pose = (x, y, z, roll, pitch, yaw)
        tilt = max(abs(roll), abs(pitch))
        self.z_min = z if self.z_min is None else min(self.z_min, z)
        self.z_max = z if self.z_max is None else max(self.z_max, z)
        self.tilt_max = tilt if self.tilt_max is None else max(self.tilt_max, tilt)
        self.roll_max_abs = max(self.roll_max_abs or 0.0, abs(roll))
        self.pitch_max_abs = max(self.pitch_max_abs or 0.0, abs(pitch))
        if not gate:
            return None
        self.gated_pose_samples += 1
        self._low = self._low + 1 if z < self.lim.body_min_height_m else 0
        self._tilt = self._tilt + 1 if tilt > self.lim.body_max_tilt_rad else 0
        reason = None
        if self._low >= self.lim.body_gate_debounce_samples:
            reason = self._trip(G1, BODY_TOO_LOW, z, self.lim.body_min_height_m, wall, stamp,
                                f'{self._low} consecutive samples below the threshold')
        if self._tilt >= self.lim.body_gate_debounce_samples:
            r = self._trip(G2, BODY_TILT, tilt, self.lim.body_max_tilt_rad, wall, stamp,
                           f'roll {roll:.4f} rad, pitch {pitch:.4f} rad')
            reason = reason or r
        return reason

    # ------------------------------------------------------------------ G8 (M6.1-A)
    def on_fixed_base(self, wall, stamp, codes, attachment, gate):
        """One fixed-base sample (codes from m61a_fixed_base.evaluate_sample, attachment =
        (translation m, rotation rad, dz, dxy) or None). Returns a cancel reason (G8) or None."""
        if self.fb is None:
            return None
        self.fb_samples += 1
        for c in codes or ():
            self.fb_codes_seen[c] = self.fb_codes_seen.get(c, 0) + 1
        if attachment is not None and _finite(*attachment[:2]):
            t, r = float(attachment[0]), float(attachment[1])
            a = self.attachment_max or (0.0, 0.0)
            self.attachment_max = (max(a[0], t), max(a[1], r))
        if not gate:
            return None
        self.fb_gated_samples += 1
        bad = [c for c in (codes or ()) if c in INTEGRITY_CODES]
        self._integrity = self._integrity + 1 if bad else 0
        if self._integrity >= self.fb.attachment_debounce_samples:
            return self._trip(G8, ATTACHMENT_LOST, None if attachment is None else attachment[0],
                              self.fb.attachment_translation_tol_m, wall, stamp,
                              f'{self._integrity} consecutive samples: {", ".join(bad)}')
        return None

    # ------------------------------------------------------------------ G3
    def on_joint_state(self, wall, stamp, names, positions, gate):
        """One /joint_states sample. Returns a cancel reason (G3) or None."""
        self.joint_samples += 1
        if gate:
            self.gated_joint_samples += 1
        worst = None
        for n, v in zip(names or (), positions or ()):
            if n not in self.joint_limits or not _finite(v):
                continue                    # missing / non-finite joints: the tracking check
            lo, hi = self.joint_limits[n]
            ext = self.extremes.setdefault(n, [v, v])
            ext[0], ext[1] = min(ext[0], v), max(ext[1], v)
            frac = tc.limit_fraction(v, lo, hi)
            if frac > self.max_fraction[0]:
                self.max_fraction = (frac, n, v)
            if frac > self.lim.joint_limit_fraction and (worst is None or frac > worst[0]):
                worst = (frac, n, v)
        if gate and worst is not None:
            return self._trip(G3, JOINT_NEAR_LIMIT, worst[0], self.lim.joint_limit_fraction,
                              wall, stamp, f'{worst[1]} at {worst[2]:.4f} rad')
        return None

    # ------------------------------------------------------------------ pose freshness
    def check_pose_stale(self, now_wall):
        """While armed: no valid body-pose sample for body_pose_stale_s (wall) -> cancel."""
        if not self.lim.body_pose_required or not self.armed:
            return None
        ref = self.armed_wall if self.last_pose_wall is None else max(self.armed_wall,
                                                                      self.last_pose_wall)
        age = now_wall - ref
        if age > self.lim.body_pose_stale_s:
            return self._trip(POSE_FRESHNESS, BODY_POSE_STALE, age, self.lim.body_pose_stale_s,
                              now_wall, None, 'no valid ground-truth body pose')
        return None

    # ------------------------------------------------------------------ report
    def drift(self):
        if self.first_pose is None or self.last_pose is None:
            return None
        a, b = self.first_pose, self.last_pose
        return {'dx_m': b[0] - a[0], 'dy_m': b[1] - a[1], 'dz_m': b[2] - a[2],
                'dyaw_rad': _wrap(b[5] - a[5])}

    def summary(self, cancel_reason=None):
        """Per-gate threshold, worst observed value, margin and trip record."""
        lim = self.lim
        trips = {t['gate']: t for t in self.trips}
        if cancel_reason in ('sim_time_stalled', 'sample_gap'):
            gate = REASON_GATE[cancel_reason]
            trips.setdefault(gate, {'gate': gate, 'reason': cancel_reason, 'value': None,
                                    'threshold': None, 'wall_s': None, 'stamp_s': None,
                                    'detail': 're-used M6.0-D stream monitor'})
        drift = self.drift()
        g7_flags = []
        if drift is not None:
            if abs(drift['dx_m']) > lim.drift_lateral_report_m:
                g7_flags.append(f'lateral |dx| {abs(drift["dx_m"]):.4f} m > '
                                f'{lim.drift_lateral_report_m} m')
            if abs(drift['dyaw_rad']) > lim.drift_yaw_report_rad:
                g7_flags.append(f'|dyaw| {abs(drift["dyaw_rad"]):.4f} rad > '
                                f'{lim.drift_yaw_report_rad} rad')

        def row(gate, threshold, worst, margin, active=True, note=None):
            t = trips.get(gate)
            return {'active': active, 'threshold': threshold, 'worst_observed': worst,
                    'margin': margin, 'tripped': t is not None, 'trip': t, 'note': note}

        frac = self.max_fraction
        return {
            G1: row(G1, lim.body_min_height_m, self.z_min,
                    None if self.z_min is None else self.z_min - lim.body_min_height_m,
                    note=f'debounce {lim.body_gate_debounce_samples} samples'),
            G2: row(G2, lim.body_max_tilt_rad, self.tilt_max,
                    None if self.tilt_max is None else lim.body_max_tilt_rad - self.tilt_max,
                    note=f'debounce {lim.body_gate_debounce_samples} samples'),
            G3: row(G3, lim.joint_limit_fraction, {'fraction': frac[0], 'joint': frac[1],
                                                   'position_rad': frac[2]},
                    lim.joint_limit_fraction - frac[0]),
            G4: row(G4, None, None, None, active=False, note=CONTACT_NOTE),
            G5: row(G5, lim.sim_stall_s, None, None,
                    note='m6_live_contract.SIM_STALL_S via the re-used StreamMonitor'),
            G6: row(G6, lim.joint_state_gap_s, None, None,
                    note='m6_live_contract.SAMPLE_GAP_S via the re-used StreamMonitor'),
            G7: {'active': True, 'report_only': True, 'drift': drift, 'flags': g7_flags,
                 'thresholds': {'lateral_m': lim.drift_lateral_report_m,
                                'yaw_rad': lim.drift_yaw_report_rad}},
            G8: row(G8, None if self.fb is None else
                    {'translation_m': self.fb.attachment_translation_tol_m,
                     'rotation_rad': self.fb.attachment_rotation_tol_rad},
                    None if self.attachment_max is None else
                    {'translation_m': self.attachment_max[0],
                     'rotation_rad': self.attachment_max[1]},
                    None if (self.fb is None or self.attachment_max is None) else
                    {'translation_m': self.fb.attachment_translation_tol_m
                     - self.attachment_max[0],
                     'rotation_rad': self.fb.attachment_rotation_tol_rad
                     - self.attachment_max[1]},
                    active=self.fb is not None,
                    note=('M6.1-A fixed base: composed body vs weld, spawn identity, link '
                          f'consistency; codes seen {self.fb_codes_seen}' if self.fb is not None
                          else 'inactive: no fixed-base configuration')),
            POSE_FRESHNESS: row(POSE_FRESHNESS, lim.body_pose_stale_s, None, None,
                                active=lim.body_pose_required),
            'trips_in_order': [dict(t) for t in self.trips],
        }

    def body_summary(self):
        return {'samples': self.pose_samples, 'gated_samples': self.gated_pose_samples,
                'invalid_samples': self.invalid_pose_samples,
                'z_min_m': self.z_min, 'z_max_m': self.z_max, 'tilt_max_rad': self.tilt_max,
                'roll_max_abs_rad': self.roll_max_abs, 'pitch_max_abs_rad': self.pitch_max_abs,
                'start_pose': self.first_pose, 'end_pose': self.last_pose,
                'drift': self.drift(),
                'frame': FIXED_BASE_FRAME if self.fb is not None else FREE_BASE_FRAME,
                'fixed_base_samples': self.fb_samples}

    def joint_extremes(self):
        return {n: {'min_rad': e[0], 'max_rad': e[1],
                    'max_limit_fraction': max(tc.limit_fraction(e[0], *self.joint_limits[n]),
                                              tc.limit_fraction(e[1], *self.joint_limits[n]))}
                for n, e in sorted(self.extremes.items())}


def pose_readiness(latest, now_wall, limits):
    """(ok, failure_code, detail) for the body pose at readiness time (before any goal)."""
    if not limits.body_pose_required:
        return True, None, {'required': False}
    detail = {'required': True, 'max_age_s': limits.body_pose_stale_s}
    if latest is None:
        return False, 'body_pose_missing', dict(detail, reason='no ground-truth body pose '
                                                'received (is the pose bridge running?)')
    wall, stamp, pose = latest
    detail.update(age_s=now_wall - wall, stamp_s=stamp, pose=list(pose) if pose else None)
    if not (isinstance(pose, (tuple, list)) and len(pose) == 6 and _finite(*pose)):
        return False, 'body_pose_invalid', dict(detail, reason='non-finite or malformed pose')
    if now_wall - wall > limits.body_pose_stale_s:
        return False, 'body_pose_stale', dict(detail, reason='body pose older than the limit')
    if pose[2] < limits.body_min_height_m:
        return False, 'body_too_low', dict(detail, reason=f'z {pose[2]:.4f} m < '
                                                          f'{limits.body_min_height_m} m')
    if max(abs(pose[3]), abs(pose[4])) > limits.body_max_tilt_rad:
        return False, 'body_tilted', dict(detail, reason='tilt above the limit')
    return True, None, detail


__all__ = ['GateMonitor', 'pose_readiness', 'GATE_IDS', 'M61_REASONS', 'REASON_GATE',
           'G1', 'G2', 'G3', 'G4', 'G5', 'G6', 'G7', 'G8', 'POSE_FRESHNESS', 'CONTACT_NOTE',
           'ATTACHMENT_LOST', 'INTEGRITY_CODES', 'FIXED_BASE_FRAME', 'FREE_BASE_FRAME']
