# M6.1 – Protected Trot Gait Replay: Design Note

**Status: DESIGN APPROVED; IMPLEMENTED for offline, mock and isolated-domain use only. Live
dispatch is HARD-DISABLED.** See [§10](#10-implementation-status) and
[`M61_IMPLEMENTATION_NOTES.md`](M61_IMPLEMENTATION_NOTES.md).
- The M6.1 gate `M61_LIVE_DISPATCH_ENABLED` exists and is `False`.
- No goal has been sent, and no Gazebo, launch or ROS runtime was started.
- Every step after the implementation needs a separate owner approval (§7).
- The design text below is unchanged from `7b8798c`. Its tables in §2.2 describe the 2.5 cm /
  10 mm example; the owner approved 2 cm / 6 mm (§10).

**Scope.** Replay **one** trot gait cycle, as **one** `FollowJointTrajectory` goal, in Gazebo
Fortress, with the M6.0-D safety envelope adapted for gait-specific risks.

**Basis.** Every number below was computed **offline** at `77fd171` (`main`) with the existing M3/M4.5
code (`leg_kinematics.inverse`/`forward`, `gait_metrics.load_inputs`, `MassModel.com`). No ROS
runtime was used. The generator in §2.3 reproduces the tables and the hash.

---

## 0. Summary and corrections to the draft request

The draft request had several assumptions that the code and the earlier results contradict. The
design follows the facts, and each conflict becomes an owner decision in §7.

| # | Draft assumption | What the repo / offline computation shows | Design consequence |
|---|---|---|---|
| C1 | "About 10 cm body height" | Neutral `base_link` height is **0.0545 m** (M2 measured 0.05450 m). Raising the body by 4.55 cm to reach 10 cm is **IK-infeasible** (`rear_right` hits its joint limits; a 4.0 cm raise is the most that solves). A 4 cm raise also moves joints by up to 0.59 rad before the cycle even starts | The cycle runs at **neutral height (0.0545 m)**. No pre-pose goal |
| C2 | Body-z gate "< 5 cm" | At 0.0545 m nominal, 5 cm leaves 4.5 mm of margin. The `crouch_10mm` pose (4.45 cm) would trip it | Gate at **0.045 m**, the M2 `min_body_height_m` threshold (D-M61-5) |
| C3 | "Same safety envelope as M6.0-D", including the 0.1223 rad cap | The cap belongs to **M6.0-D only** (owner option (i), `bef3c25`). Every 2–3 cm step with a ≥ 1 cm lift exceeds it: the recommended cycle reaches **0.1517 rad** on its continuous path. The M6.0-D envelope also forbids more than 5 points (`MAX_POINTS`) and segments shorter than 3.0 s (`MIN_SEGMENT_S`) | A **separate M6.1 envelope** with its own owner-approved cap (D-M61-2). The M6.0-D constants and tests stay unchanged |
| C4 | 2.0 s cycle | Peak joint speed **0.391 rad/s** at 2.0 s. That is under the 0.5 rad/s placeholder, but **above the 0.25 rad/s** M6.0-D planning speed (0.5 / `SPEED_FACTOR` 2). At 4.0 s the peak is **0.196 rad/s** | Recommend **T = 4.0 s**, with 2.0 s as an owner option or a later step (D-M61-3). The positions are identical for both; only the times and velocities scale |
| C5 | Trot is "the safest first gait" | M5: trot static margin **N/A** (two-foot line support). The CoM is **4.0 / 3.8 mm** off the two diagonal support lines, so the free-base robot **will tip** during each swing (§1.4). A wave crawl keeps 3 feet down | Trot is the **simplest** gait to replay and verify. It is the **safest** only on a fixed base. See §1.4 and D-M61-1 |
| C6 | Free base on the ground, implied by the body-z and contact gates | `M6_GAIT_PLAYBACK_SAFETY_PLAN.md` D2 recommends a **raised fixed base** for M6.1. It calls a free base "a locomotion experiment, out of scope". D2 was deferred, not decided | The owner must decide D2 explicitly (D-M61-1). This design supports both bases |
| C7 | "Unexpected contact" gate | The model and the world have **no contact sensors** | Contact is **not measured**. Without a model change only proxies (body z, tilt) exist (D-M61-6) |
| C8 | Body pose available | The ground-truth pose bridge exists only in `fortress_posture_hold.launch.py`, not in `fortress_control.launch.py` | A launch change (or a new M6.1 launch file) is needed. It is a separate approval (D-M61-7) |
| C9 | `src/spiderx_gaits/config/trot_cycle_01.yaml` | The `spiderx_gaits` package does not exist. Gait configs live in `src/spiderx_controller/config/` | Use `src/spiderx_controller/config/m61_trot_cycle.yaml`. No new package |
| C10 | About 1 week | New body and tilt monitors, a launch change, a new envelope, and mock plus isolated-domain tests | About **2.5 weeks** of implementation, plus owner-side runs (§9) |

**Frame.** `base_link`: +x right, **+y forward**, +z up (`M3_FRAME_CONVENTIONS.md`). The robot travels
along +y.

---

## 1. Trot gait cycle specification

### 1.1 Pattern

| Item | Value |
|---|---|
| Diagonal pairs | **A = LF + RR** (`front_left`, `rear_right`); **B = RF + LR** (`front_right`, `rear_left`) |
| Duty factor | **0.5**. In each half cycle one pair swings while the other is in stance |
| Phase offsets (M4.5 `trot`) | FL 0.0, FR 0.5, RL 0.5, RR 0.0 |
| Cycle type | **Start-stop single cycle**: it starts and ends at CAD neutral (all 12 joints 0 rad) with zero velocity. Pair A swings in the first half and pair B in the second |
| Cycle period T | **4.0 s recommended**; 2.0 s as an option (C4) |
| Step length (world foot advance) | **0.025 m** (inside the 2–3 cm request). Each foot moves 2.5 cm forward in the world once |
| Body advance per cycle | **0.025 m** along +y (nominal; on a fixed base the body does not move) |
| Step height (lift) | **0.010 m** apex (cycloid) |
| Stance height | Neutral: feet at the CAD-neutral tip height. `base_link` is 0.0545 m above the ground |
| Swing profile | World-frame cycloid: zero foot velocity at lift-off and touchdown |
| Body profile | Cycloid advance `b(t) = L (t/T − sin(2πt/T)/2π)`: zero body speed at t = 0 and t = T, peak at T/2 |

**Why start-stop.** The M4.5 periodic trot (`gait_trajectory.sample_cycle`) is a steady-state cycle.
At u = 0 its feet are at ±L/2 from neutral and the body is already moving at v. Starting it from
neutral would need a pre-positioning goal (feet sliding on the ground) or a velocity jump. The
start-stop cycle avoids both:
- one goal goes from neutral to neutral;
- every foot touches down exactly once;
- all foot velocities are zero at both ends.

The waypoint velocities are therefore exactly zero at the first and last points, as in M6.0-D.

### 1.2 Foot trajectory (offset from the CAD-neutral tip, `base_link`)

Half period τ = T/2. All offsets have x = 0. The motion is purely sagittal, so the hips stay at 0.

| Leg group | 0 ≤ t ≤ τ | τ ≤ t ≤ T |
|---|---|---|
| Pair A (LF, RR) | **Swing**: y 0 → +L/2, z 0 → h → 0 | **Stance**: y +L/2 → 0 |
| Pair B (RF, LR) | **Stance**: y 0 → −L/2 | **Swing**: y −L/2 → 0, z 0 → h → 0 |

### 1.3 Per-joint neutral, lift, forward displacement and touchdown (rad, T = 4.0 s timings)

Mirror symmetry holds:
- `rr_* = −lf_*` (thigh and foot);
- `lr_thigh = −rf_thigh` and `lr_foot = rf_foot`.

All four hip joints stay at **0.0000** throughout.

| Key frame | Cycle time | `lf_thigh` | `lf_foot` | `rf_thigh` | `rf_foot` |
|---|---|---|---|---|---|
| Neutral (start) | 0.0 s | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| Pair A lift-off / pair B stance start | 0.0 s | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| Pair A lift apex (10 mm) | 1.0 s | −0.0245 | +0.1145 | −0.0161 | −0.0005 |
| Pair A touchdown (+12.5 mm fwd) / pair B lift-off (−12.5 mm) | 2.0 s | −0.0918 | −0.0106 | −0.0857 | +0.0031 |
| Pair B lift apex (10 mm) | 3.0 s | −0.0163 | −0.0009 | −0.1314 | −0.1207 |
| Pair B touchdown / neutral (end) | 4.0 s | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| Continuous-path extreme | — | −0.1057 / +0.0138 | −0.0108 / +0.1149 | **−0.1517** / 0.0000 | −0.1208 / +0.0032 |
| Fraction of URDF limit used (max) | — | 17.3 % | 26.3 % | 19.3 % | **27.7 %** |

### 1.4 Why trot first, and what "safe" means here

**For trot:**
- **Simplest timing:** two phases, and one cycle has only two swing events (a crawl has four).
- **Symmetric:** the diagonal pairs mirror each other (§1.3), so a sign or joint-order error shows
  up as broken symmetry in the joint-state CSV.
- **Shortest goal for a given step:** less sim time is exposed to faults.
- **Already analysed:** it is the M4.5 reference gait and passed M5 `valid_pass`, kinematics PASS.

**Against trot:**
- **Not statically stable.** During each half cycle, support is a **line** (two diagonal feet). At
  neutral, the whole-robot CoM (`MassModel.com`) is at (0.0510, −0.0576, 0.0717) m. That is
  **4.0 mm** from the LF–RR line and **3.8 mm** from the RF–LR line.
- **The free-base robot will tip.** Treated as an inverted pendulum at a CoM height of 0.072 m, its
  time constant is about 0.085 s. In each swing it tips toward the CoM side until a swinging foot
  touches. A swinging foot is about 0.126 m from the support line, so at the 10 mm apex the
  expected tilt is up to **≈ 4.5°**, with the swing foot scuffing. This is expected behaviour and is
  not a fault. The tilt gate (§3.3) must allow it.
- A **wave crawl** (β ≥ 0.75, 3 feet down) is the safer free-base gait for stability.

**Conclusion.**
- On a **raised fixed base** (D2 option (a)), trot is the safest first gait: simplest, shortest,
  symmetric, and with no contact.
- On a **free base**, trot is the **simplest** gait but not the safest. Its outcome includes tipping
  and scuffing, so the only claims allowed are joint-tracking claims.

---

## 2. Trajectory design

### 2.1 Waypoints and timing

| Item | Value |
|---|---|
| Points | **9** (1 lead-in neutral + 7 interior + 1 final neutral), in the 8–12 range |
| Spacing | T/8 = **0.5 s** at T = 4.0 s (0.25 s at T = 2.0 s) |
| Lead-in | **3.0 s** hold at neutral before the cycle (the M6.0-D lead-in rule: `max(MIN_SEGMENT_S, 2 × 0.05 / 0.5)`) |
| Total duration | 7.0 s at T = 4.0 s (5.0 s at T = 2.0 s), in **sim time**. M6.0-D ran at about 0.23× real time, so this is about 30 s wall time |
| Cycles | **Exactly one**. The final point is neutral; there is no second cycle and no return goal |
| Velocities | Analytic path derivative at the interior points (central difference of the IK path); **0** at the first and last points |
| Interpolation | JTC default `splines`: cubic Hermite with positions and velocities. Checked offline against the true path (§2.4) |
| Joint order | `spiderx_ros2_controllers.yaml#leg_trajectory_controller.joints` (same as M6.0-D) |

### 2.2 Waypoint table (T = 4.0 s, L = 0.025 m, h = 0.010 m, neutral height)

Positions in rad. Hip columns are omitted because all four hips are **0.0000** at every point. For
the full 12-joint vector, regenerate with §2.3.

| # | t (s) | lf_thigh | lf_foot | rf_thigh | rf_foot | lr_thigh | lr_foot | rr_thigh | rr_foot | Phase |
|---|---|---|---|---|---|---|---|---|---|---|
| 0 | 3.00 | +0.0000 | +0.0000 | +0.0000 | +0.0000 | +0.0000 | +0.0000 | +0.0000 | +0.0000 | neutral, A lift-off |
| 1 | 3.50 | +0.0137 | +0.0612 | −0.0022 | −0.0001 | +0.0022 | −0.0001 | −0.0137 | −0.0612 | A swing ¼ |
| 2 | 4.00 | −0.0245 | +0.1145 | −0.0161 | −0.0005 | +0.0161 | −0.0005 | +0.0245 | −0.1145 | A apex |
| 3 | 4.50 | −0.0955 | +0.0455 | −0.0458 | −0.0001 | +0.0458 | −0.0001 | +0.0955 | −0.0455 | A swing ¾ |
| 4 | 5.00 | −0.0918 | −0.0106 | −0.0857 | +0.0031 | +0.0857 | +0.0031 | +0.0918 | +0.0106 | A touchdown, B lift-off |
| 5 | 5.50 | −0.0475 | −0.0038 | −0.1417 | −0.0553 | +0.1417 | −0.0553 | +0.0475 | +0.0038 | B swing ¼ |
| 6 | 6.00 | −0.0163 | −0.0009 | −0.1314 | −0.1207 | +0.1314 | −0.1207 | +0.0163 | +0.0009 | B apex |
| 7 | 6.50 | −0.0022 | −0.0001 | −0.0426 | −0.0624 | +0.0426 | −0.0624 | +0.0022 | +0.0001 | B swing ¾ |
| 8 | 7.00 | +0.0000 | +0.0000 | +0.0000 | +0.0000 | +0.0000 | +0.0000 | +0.0000 | +0.0000 | neutral, B touchdown |

Velocities in rad/s at T = 4.0 s; multiply by 2 for T = 2.0 s. Hips are 0.

| # | lf_thigh | lf_foot | rf_thigh | rf_foot | lr_thigh | lr_foot | rr_thigh | rr_foot |
|---|---|---|---|---|---|---|---|---|
| 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 1 | +0.0061 | +0.1869 | −0.0130 | −0.0005 | +0.0130 | −0.0005 | −0.0061 | −0.1869 |
| 2 | −0.1499 | −0.0228 | −0.0439 | −0.0006 | +0.0439 | −0.0006 | +0.1499 | +0.0228 |
| 3 | −0.0885 | −0.1947 | −0.0730 | +0.0029 | +0.0730 | +0.0029 | +0.0885 | +0.1947 |
| 4 | +0.0944 | +0.0174 | −0.0822 | +0.0099 | +0.0822 | +0.0099 | −0.0944 | −0.0174 |
| 5 | +0.0785 | +0.0093 | −0.0881 | −0.1939 | +0.0881 | −0.1939 | −0.0785 | −0.0093 |
| 6 | +0.0450 | +0.0031 | +0.1350 | −0.0114 | −0.1350 | −0.0114 | −0.0450 | −0.0031 |
| 7 | +0.0130 | +0.0006 | +0.1665 | +0.1934 | −0.1665 | +0.1934 | −0.0130 | −0.0006 |
| 8 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |

**Displacement relative to the 0.1223 rad M6.0-D cap**

| Measure | T = 4.0 s | Versus 0.1223 rad |
|---|---|---|
| Max \|q − neutral\| at the 9 waypoints | 0.1417 rad (`rf/lr_thigh`, point 5) | **+15.9 % over** |
| Max \|q − neutral\| on the continuous path (400 samples) | **0.1517 rad** (`rf/lr_thigh`) | **+24.0 % over** |
| Peak joint speed (continuous path) | 0.196 rad/s (0.391 at T = 2.0 s) | n/a; M6.0-D planning speed is 0.25 rad/s |
| Max fraction of the URDF limit | 27.7 % (`rf/lr_foot`) | far below the 80 % gate |
| Min distance to a URDF limit | 0.3155 rad | – |

**Variants that fit the 0.1223 rad cap** (all at T = 4.0 s; the continuous-path maximum is shown):

| Step L | Lift h | Max \|q\| | Peak speed |
|---|---|---|---|
| 0.020 m | 0.005 m | 0.1072 | 0.132 |
| 0.020 m | 0.006 m | 0.1117 | 0.139 |
| 0.020 m | 0.008 m | 0.1213 | 0.157 (only 0.8 % below the cap) |
| 0.025 m | ≥ 0.005 m | ≥ 0.128 | over the cap |
| 0.030 m | ≥ 0.005 m | ≥ 0.148 | over the cap |

The lift contributes more joint motion than the step does. A 2.5 cm step therefore needs either a
new M6.1 cap (proposal: **0.16 rad**, about 5 % above the 0.1517 path maximum) or a smaller cycle
(L = 2 cm, h = 6 mm, max 0.1117 rad). This is owner decision D-M61-2.

### 2.3 Generator (reference; offline; IK only)

This is the design reference, not an implementation. It needs the workspace sourced
(`source install/setup.bash`) only so that it can import `spiderx_controller`. It starts no ROS
node.

```python
"""M6.1 design reference: one start-stop trot cycle, neutral -> neutral (offline, IK only)."""
import hashlib, json, math, sys
from spiderx_controller import gait_metrics as gm, leg_kinematics as lk

T, STEP, LIFT, Z0, LEAD, SEGMENTS = 4.0, 0.025, 0.010, 0.0, 3.0, 8   # s, m, m, m, s, -
PAIR_A = ('front_left', 'rear_right')            # swings in the first half cycle
ORDER = ['lf_hip', 'lf_thigh_joint', 'lf_foot_joint', 'rf_hip', 'rf_thigh_joint', 'rf_foot_joint',
         'lr_hip', 'lr_thigh_joint', 'lr_foot_joint', 'rr_hip', 'rr_thigh_joint', 'rr_foot_joint']


def body(t):                                     # body advance along +y; zero speed at 0 and T
    return STEP * (t / T - math.sin(2 * math.pi * t / T) / (2 * math.pi))


def offset(leg, t):                              # foot offset from the CAD-neutral tip (base_link)
    tau, a = T / 2, leg in PAIR_A
    t0, y0 = (0.0, 0.0) if a else (tau, -STEP / 2)
    if t0 <= t <= t0 + tau:                      # swing: world cycloid, STEP forward, LIFT high
        u = (t - t0) / tau
        y = y0 + STEP * (u - math.sin(2 * math.pi * u) / (2 * math.pi)) - (body(t) - body(t0))
        return (0.0, y, Z0 + LIFT * (1 - math.cos(2 * math.pi * u)) / 2)
    return (0.0, (STEP / 2 - (body(t) - body(tau))) if a else -body(t), Z0)   # stance


def q_at(geoms, t):
    out = {}
    for leg in lk.ALL_LEGS:
        g = geoms[leg]
        r = lk.inverse(g, tuple(p + o for p, o in zip(g.tip0, offset(leg, t))),
                       margin=lk.DEFAULT_MARGIN_RAD)
        if not r['ok']:
            sys.exit(f'IK failed: {leg} t={t} {r["reason"]}')
        out.update(zip(g.joint_names, r['solution']))
    return [out[n] for n in ORDER]


def build():
    _, geoms, _ = gm.load_inputs()
    pts, eps = [], 1e-5
    for k in range(SEGMENTS + 1):
        t = k * T / SEGMENTS
        q = q_at(geoms, t)
        v = ([0.0] * 12 if k in (0, SEGMENTS) else
             [(b - a) / (2 * eps) for a, b in zip(q_at(geoms, t - eps), q_at(geoms, t + eps))])
        pts.append({'time_from_start_s': round(LEAD + t, 6),
                    'positions': [round(x, 9) for x in q], 'velocities': [round(x, 9) for x in v]})
    doc = {'schema': 'spiderx.m61.trot_cycle.design/0', 'joint_names': ORDER, 'points': pts,
           'params': {'cycle_period_s': T, 'step_length_m': STEP, 'step_height_m': LIFT,
                      'stance_height_offset_m': Z0, 'lead_in_s': LEAD, 'segments': SEGMENTS}}
    blob = json.dumps(doc, sort_keys=True, separators=(',', ':')).encode()
    return doc, hashlib.sha256(blob).hexdigest()


if __name__ == '__main__':
    doc, digest = build()
    print(digest)
    print('max |q| =', max(abs(x) for p in doc['points'] for x in p['positions']))
```

### 2.4 Content hash and spline fidelity

**Design content hashes.** These are SHA-256 hashes of the canonical JSON above (`sort_keys`, compact
separators, values rounded to 1e-9):

| Variant | SHA-256 |
|---|---|
| **T = 4.0 s** (recommended) | `d03e80a0e733ae756ed95433514aea5490ac627ffb6b18f5b47d2816174e876c` |
| T = 2.0 s | `328f0535951edc9ed936a75db227078bc8daaa4f371ade9d43a279b8b4be7c01` |

These are **design-time** hashes, and they depend on libm round-off at the 1e-9 level. The
implementation will compute its own `trajectory_id` and goal fingerprint with the M6.0-D scheme
(`m6_trajectory.compute_trajectory_id`, `m6_goal_fingerprint`). It will pin them in a test, exactly
as `TRAJECTORY_ID`/`FINGERPRINT` are pinned for M6.0-D, and record them in the implementation
results. The hashes above only identify this design's numbers.

**Spline fidelity.** The JTC cubic Hermite through the 9 points was compared with the true IK path
offline (800 samples, FK of the interpolated joints):

| Points | Max joint deviation | Max foot-position deviation | Min swing clearance (u ∈ [¼, ¾]) | Stance foot z error |
|---|---|---|---|---|
| **9** (T/8) | 0.00079 rad | 0.08 mm | 5.0 mm | < 0.01 mm |
| 11 (T/10) | 0.00040 rad | 0.03 mm | 5.0 mm | < 0.01 mm |
| 13 (T/12) | 0.00018 rad | 0.02 mm | 5.0 mm | < 0.01 mm |

Nine points are enough. The interpolation error is about 60× below the 0.05 rad tolerance.

---

## 3. Safety envelope (adapted from M6.0-D)

### 3.1 Carried over unchanged from M6.0-D

| Item | Value |
|---|---|
| Goals per run | **1**. No retries, no second goal, `automatic_return_goals = 0` |
| Readiness | Read-only readiness collected **before** and **after** confirmation, as in M6.0-D |
| Freshness at send | Readiness no older than **10.0 s** (`READINESS_MAX_AGE_S`) |
| Fingerprint | The goal is verified against the pinned fingerprint immediately before `send_goal` |
| Tolerances | Path and goal position **0.05 rad**; goal velocity **0.05 rad/s**; goal time **1.0 s** |
| Client tracking abort | **0.05 rad** in-flight error → cancel |
| Watchdogs | Goal response 10 s; cancel response 5 s; result watchdog **120 s wall**; controller presence every 1 s |
| Stale `/joint_states` | No message for **0.5 s wall** → cancel |
| Interrupts | Ctrl+C / SIGTERM before send: abort, nothing sent. After send: one cancel, wait for the final status, record it, exit. No return motion |
| Evidence | `EvidenceFile` written at every phase. Exceptions are recorded, never swallowed |
| Isolation for tests | Isolated domain 150–199, `ROS_LOCALHOST_ONLY=1`, a graph probe, and the fake stack only |

### 3.2 New for M6.1

| Item | Value |
|---|---|
| Live gate | **`M61_LIVE_DISPATCH_ENABLED = False`**: a new constant in a new `m61_live_contract.py`, separate from the M6.0-D gate, with its own pin test (`test/m61_gate.py`). It is enabled only on a separate, never-merged branch, as for M6.0-D |
| Confirmation word | **`SEND-ONE-TROT-CYCLE`**, typed exactly. Any other input aborts with 0 goals |
| Envelope module | New `m61_envelope.py` with its own constants: cap (D-M61-2), `MAX_POINTS = 9`, `MIN_SEGMENT_S = 0.5` (0.25 if D-M61-3 = 2.0 s), max speed 0.25 rad/s planning (0.5 placeholder), 80 % URDF fraction. **`m6_envelope.py` is not modified** |
| No automatic return | After any gate trip or cancel the robot stays where the controller stopped it. A return to neutral is a separate, separately approved goal |
| Body pose required | Readiness is **NOT READY** without fresh ground-truth body pose (free base) |

### 3.3 Gait-specific gates

All gates run in the client during the goal. They start at acceptance and run until 1.0 s (sim time)
after the result. A trip triggers **one cancel**, records the gate, its value and the threshold, and
ends with FAIL. **No return goal** is sent.

| Gate | Threshold | Detection method | Debounce | On trip | Fixed base |
|---|---|---|---|---|---|
| **G1 Body height** | `base_link` z **< 0.045 m** (nominal 0.0545; M2 threshold; D-M61-5). The 0.05 m draft value is rejected (C2) | Ground-truth pose from `/world/spiderx_fortress/pose/info` via the bridge (C8), entry `spiderx`/`dummy_link`; z of the model pose | 2 consecutive samples (≥ 0.1 s sim) | Cancel, FAIL `body_low` | N/A (logged only) |
| **G2 Tilt** (added) | \|roll\| or \|pitch\| **> 0.26 rad (15°)**. The expected tilt is ≤ 4.5° (§1.4) | Same pose source; quaternion to roll/pitch | 2 samples | Cancel, FAIL `body_tilt` | N/A |
| **G3 Joint near limit** | Observed \|q\| **> 80 % of the URDF limit** on its side, per joint (the trajectory itself peaks at 27.7 %) | `/joint_states` positions against limits from the URDF (the same loader as M6.0-D) | none (single sample) | Cancel, FAIL `joint_near_limit` | Active |
| **G4 Unexpected contact** | **Not measurable** (no contact sensors, C7). Proxies: G1 (belly near the ground) and G2 (tip-over). The report states `contact: not_measured` | Proxy only, unless D-M61-6 approves adding a contact sensor on the `base_link` collision (a model change) | – | Via G1/G2 | N/A |
| **G5 Sim stall** | No `/clock` advance for **> 5.0 s wall** (`SIM_STALL_S`, already in M6.0-D) | `/clock` vs monotonic wall clock | – | Cancel, FAIL `sim_stall` | Active |
| **G6 Joint-state gap** | Gap between consecutive `/joint_states` stamps **> 0.25 s sim** (`SAMPLE_GAP_S`, already in M6.0-D) | Header stamps (sim time) | – | Tracking failure → cancel, FAIL `joint_state_gap` | Active |
| **G7 Body drift** (report only) | Lateral \|Δx\| > 0.02 m or \|Δyaw\| > 0.1 rad | Pose start vs end | – | Flag in the report; no cancel | N/A |

**Gate ordering when several trip.** The first gate to trip is recorded as the cause. Every later
gate is recorded too. Exactly one cancel is sent.

---

## 4. Architecture: Option B, a new `m6_gait_replay.py`

The M6.0-D modules are reused **by import**, not edited. The M6.0-D trajectory ID, fingerprint, tests
and reports must stay byte-identical; the existing pins prove this.

```
config/m61_trot_cycle.yaml          parameters only (T, L, h, z0, lead-in, segments, pair order)
m61_envelope.py                     M6.1 constants (cap, MAX_POINTS, speeds, gate thresholds)
m61_trot_cycle.py                   generator (§2.3 logic) + M6.1 offline preflight
m61_live_contract.py                M61_LIVE_DISPATCH_ENABLED = False, CONFIRMATION_WORD, gates
m61_body_monitor.py                 pose subscriber, G1/G2/G7; pure functions + a thin rclpy adapter
m6_gait_replay.py                   CLI: --dry-run | --mock | --live; reuses m6_live_playback's
                                    state machine, RclpyLiveTransport, EvidenceFile, readiness
test/m61_gate.py                    EXPECTED_M61_LIVE_DISPATCH_ENABLED = False
test/test_m61_*.py                  offline, mock, mutation, isolated-domain (fake stack + fake pose)
```

**Config file.** Use `src/spiderx_controller/config/m61_trot_cycle.yaml`, not the
`src/spiderx_gaits/...` path suggested in the request (C9). It holds parameters, not joint values.
The joint trajectory is derived deterministically from the parameters plus the URDF and the configs,
as M6.0-D derives its trajectory from `m4_pose_targets.yaml`. All inputs are SHA-256-recorded in
`provenance.inputs_sha256`.

**Same as M6.0-D:**
- fingerprint scheme;
- tolerances;
- readiness;
- the transport;
- the exit codes (0 OK, 2 refused/not ready, 3 gate disabled, …).

**Extended report fields** in `live_outcome.json`:
- `gait` — name, parameters, `cycle_period_s`, `cycles: 1`;
- `envelope` — the M6.1 constants used;
- `gates` — per gate: threshold, worst observed value, margin, tripped, first-trip sim time;
- `body` — z min/max, roll/pitch max, start/end pose, Δy, Δx, Δyaw;
- `base_constraint` — `raised_fixed` | `free`;
- `contact: "not_measured"`;
- `joint_extremes_observed` — per joint min/max and fraction of the limit;
- `spline_check` — offline deviation from §2.4.

**Logs:**
- `joint_states.csv`: sim stamp, wall stamp, 12 positions, 12 velocities. All messages, not
  decimated.
- `body_pose.csv`: sim stamp, x, y, z, roll, pitch, yaw. Free base only; the fixed base logs a
  header plus `N/A`.

**Launch.** A new `fortress_m61.launch.py` reuses `fortress_control.launch.py` and adds the
ground-truth pose bridge (C8). A raised fixed base needs a model/launch variant (D2 option (a)). Both
changes are separate approvals (D-M61-1, D-M61-7). `fortress_control.launch.py` is not edited.

---

## 5. Evidence requirements

Each run writes `log/m61_run/<UTC YYYYMMDDTHHMMSSZ>/` (git-ignored, never overwritten):

| File | Content | Required for |
|---|---|---|
| `live_outcome.json` | The extended report (§4), with `evidence.phase = final` | Every run, including aborts |
| `trajectory.json` | The exact goal content: points, times, velocities, `trajectory_id` | Every run |
| `goal_fingerprint.txt` | The pinned fingerprint and the one verified at send | Every run |
| `readiness_before.json`, `readiness_after.json` | The two readiness snapshots and the freshness age at send | Every run |
| `joint_states.csv` | Every `/joint_states` message, from acceptance − 1 s to result + 1 s | Any sent goal |
| `commanded_vs_observed.csv` | Per sample: commanded (spline) vs observed, per joint, and the error | Any sent goal |
| `body_pose.csv` | Ground-truth pose at the bridge rate | Free base |
| `gates.json` | Per-gate trace: worst value, margin, trips | Any sent goal |
| `console.log` | Full stdout/stderr of the tool | Every run |
| `git_state.txt` | Commit SHA, branch, `git status --porcelain`, gate value | Every run |
| `environment.txt` | ROS distro, Gazebo version, RTF estimate, `ROS_DOMAIN_ID` | Every run |
| `video.mp4` (optional) | Screen recording of the Gazebo GUI | **Required before any visual claim** ("it stepped", "it did not tip") |

**Allowed claim on success (fixed base):** "One offline-validated trot-cycle joint trajectory was
replayed in Gazebo on a raised fixed base; joint tracking within X rad (simulation, placeholder
actuators)."

**Allowed claim on success (free base):** the same, plus "the body stayed above 0.045 m and within
15° tilt; measured body displacement was Δy = … m". The words **walking** and **locomotion** are not
allowed:
- friction is μ 0.2;
- there is no contact sensing;
- the actuators are placeholders (100 N·m, 100 rad/s);
- there is one cycle only.

**Prohibited claims:** walking, locomotion, stability, contact or slip, energy, hardware, real-time,
and any performance at speeds other than the one run.

---

## 6. Top-5 risk register

| # | Risk | Likelihood / impact | Mitigation | Residual |
|---|---|---|---|---|
| R1 | **Tip-over / scuffing** on a free base (line support, CoM 4 mm off the diagonals; expected tilt ≈ 4.5°) | High / medium (sim only) | Raised fixed base first (D-M61-1); h = 10 mm keeps the tilt bounded; G1/G2 cancel; no stability claim | The free-base outcome is uncertain by design |
| R2 | **Envelope creep**: relaxing M6.0-D constants or the 0.1223 cap for M6.1 | Medium / high (process) | A separate `m61_envelope.py`; M6.0-D pins (trajectory ID, fingerprint, hashes) must stay unchanged; an explicit owner-approved M6.1 cap | Low |
| R3 | **Missing observability**: no pose bridge, no contact sensor, so the body gates are blind | High (it exists today) / high | Readiness NOT READY without fresh pose (free base); `contact: not_measured`; launch change approved separately | Contact stays unmeasured unless D-M61-6 |
| R4 | **Gate or enablement mistake**: M6.1 gate merged `True`, enabling branch merged, M6.0-D gate touched | Low / critical | Separate gate constant plus pin test; enabling only on a never-merged branch; the M6.0-D gate test is unchanged; no merges by the agent | Low |
| R5 | **Low sim RTF (≈ 0.23×) and timing**: false sim-stall trips or goal-time-tolerance failures; speed effects hidden | Medium / medium | All trajectory times in sim time; stall gate on `/clock` progress, not on the rate; RTF recorded; T = 4.0 s keeps speeds at ≤ 0.2 rad/s | Results valid only at the tested RTF |

Also tracked, outside the top 5:
- foot slip at μ 0.2 (report Δy only);
- JTC spline vs offline path (§2.4: 0.0008 rad);
- placeholder actuators idealise tracking (M2 limitation 1).

---

## 7. Decision gate

Nothing proceeds without these explicit owner decisions. **Safe default for every item: no M6.1
work.**

| ID | Decision | Options | Recommendation |
|---|---|---|---|
| D-M61-1 | Base constraint (resolves D2) | (a) raised fixed base: model/launch variant, no contact; (c) free base on the ground | **(a) first.** Free base only as a separate later step |
| D-M61-2 | M6.1 displacement cap | (i) new cap **0.16 rad** for L 2.5 cm / h 1 cm; (ii) keep 0.1223 with L 2 cm / h 6 mm (max 0.1117) | (ii) if "same envelope" is binding; otherwise (i) |
| D-M61-3 | Cycle period | 4.0 s (peak 0.196 rad/s) or 2.0 s (0.391 rad/s) | **4.0 s** |
| D-M61-4 | Body height | Neutral 0.0545 m (10 cm infeasible, C1) | Neutral |
| D-M61-5 | Body-z threshold | 0.045 m (M2) or 0.05 m (draft) | **0.045 m** |
| D-M61-6 | Contact | Proxy only, or add a contact sensor to the model (model change) | Proxy only for M6.1 |
| D-M61-7 | Pose bridge | New `fortress_m61.launch.py`, or edit `fortress_control.launch.py` | New launch file |
| D-M61-8 | Confirmation word | `SEND-ONE-TROT-CYCLE` | As proposed |
| D-M61-9 | Config location | `spiderx_controller/config/m61_trot_cycle.yaml` | As proposed |

**Approval sequence.** Each step is a separate owner approval:
1. Approve this design and decide D-M61-1 to D-M61-9.
2. Implementation batches (offline, mock and isolated-domain only; gate `False`).
3. Cloud and local verification of those batches.
4. Read-only graph preflight on the M6.1 launch (observation only, no goal).
5. Final audit, then the enabling commit on a never-merged branch.
6. **One** live run by the owner.
7. Results document, gate re-disabled.

Nothing in this note authorizes any of steps 2–7.

---

## 8. Future work

Every item below needs its own plan and separate approval. None is authorized by this note.

| Milestone | Content | Prerequisite |
|---|---|---|
| M6.2 | The same trot cycle on a **free base** (if M6.1 used the fixed base), or at T = 2.0 s | M6.1 results |
| M6.3 | Multi-cycle replay (N cycles in one goal); wave crawl (3 feet down) as the stability baseline | M6.2; owner-approved N and envelope |
| M6.4 | Contact sensing in the model; foot-slip and support measurement | Model change approval |
| M5.5 | Command-velocity bridge (`/cmd_vel` → gait) | M6.3 |
| M7 | Odometry and state estimation (leg odometry, EKF, drift vs ground truth) | Measured stepping (M6.3/M6.4) |
| M8 | SLAM and localization | M7 |
| M9 | Nav2 | M8 |
| M10 | Real hardware (actuator interface, real masses, servo limits) | Datasheets; all of the above on hardware |

The roadmap's own future item ("walks forward 1 m in simulation without falling, a video is
required") stays unscheduled. M6.1 does not satisfy it.

---

## 9. Timeline (estimate, after approval of §7)

| Batch | Work | Estimate |
|---|---|---|
| A | `m61_envelope.py`, `m61_trot_cycle.py` (generator + preflight), config, offline and mutation tests, pinned IDs | 2 days |
| B | `m61_body_monitor.py` (pure gate functions G1–G7 + rclpy adapter), mock tests with scripted poses | 3 days |
| C | `m6_gait_replay.py` CLI, M6.1 gate and contract, evidence and report extensions, isolated-domain test with fake stack and fake pose | 3 days |
| D | `fortress_m61.launch.py` (+ raised fixed-base variant if D-M61-1 (a)), launch tests; owner-side read-only graph preflight | 2 days + owner run |
| E | Docs, cloud and local full-suite verification | 1–2 days |
| F | Final audit → enabling branch → one owner live run → results doc → gate re-disabled | 1 day (owner) |

**Total: about 2.5 weeks** of implementation plus owner-side runs. The 1-week estimate in the
request is optimistic, mainly because of Batches B and D (new observability), which M6.0-D did not
need.

---

## 10. Implementation status

**Owner decisions (2026-10-03):**

| Decision | Approved choice |
|---|---|
| D-M61-1 | Fixed / clamped base |
| D-M61-2 | Option (ii): keep the 0.1223 rad cap with a 2 cm step and 6 mm lift |
| D-M61-3 | 4.0 s cycle |
| D-M61-4 | Neutral body height |
| D-M61-5 | 0.045 m body-height gate |
| D-M61-8 | `SEND-ONE-TROT-CYCLE` |
| D-M61-9 | Configuration in `spiderx_controller/config`, with a separate `m61_limits.yaml` |

D-M61-6 (contact: proxy only) and D-M61-7 (pose-bridge launch) were not decided; this batch
implements no contact sensor and no launch file.

| Design batch (§9) | State |
|---|---|
| A: envelope, generator, preflight, config, pinned IDs | **Done**. `config/m61_limits.yaml`, `config/m61_trot_cycle.yaml`, `m61_limits.py`, `m61_trot_cycle.py`, `m61_goal.py` |
| B: gate logic G1–G7 + mock tests | **Done**. `m61_gates.py`, `m61_mock.py` |
| C: CLI, M6.1 gate and contract, evidence, isolated-domain test | **Done**. `m6_gait_replay.py`, `m61_live_contract.py`, `m61_evidence.py`, `m61_live_adapter.py` (read-only pose subscription, isolated-domain test) |
| D: M6.1 launch with the pose bridge; fixed-base variant | **Not done**: needs Gazebo to verify, which was prohibited. A live run is blocked until it exists |
| E: docs and cloud verification | **Done (cloud)**. 8 packages built; 1332 tests, 0 failures. Owner-PC verification pending |
| F: audit, enabling branch, one live run | **Not started**. Separate approvals |

**Identity of the approved trajectory:**

| Item | Value |
|---|---|
| Content | `94a492c43fcc046125d6bc71ee7f1d9a8a5d8466f1a1bdd9958a16044dd58e64` |
| `trajectory_id` | `241760e7dfd5ef12` |
| Goal fingerprint | `9dba1a173e212bfc172ffcd7da59124f87e98a321906e914e9d60e55595e5c3a` |

The §2.4 hash `d03e80a0…` identifies the 2.5 cm / 10 mm table, which was not approved.

**Approved trajectory figures** (offline, commanded spline):

| Measure | Value | Limit |
|---|---|---|
| Max displacement | 0.1114 rad | cap 0.1223 rad |
| Peak joint speed | 0.139 rad/s | – |
| Max limit fraction | 0.169 | G3 at 0.8 |

Implementation deviations from this design, with justifications, are listed in
[`M61_IMPLEMENTATION_NOTES.md` §3](M61_IMPLEMENTATION_NOTES.md#3-deviations-from-the-design-or-the-request-with-justification).
The verification checklist that must be satisfied before any live run is in
[§5](M61_IMPLEMENTATION_NOTES.md#5-verification-checklist-before-any-live-m61-run).

---

*The M6.0-D gate stays `False` on `main`, and the enabling branch
`claude/spiderx-m6d-enable-gate` is not merged. The M6.1 gate `M61_LIVE_DISPATCH_ENABLED` is
`False`.*
