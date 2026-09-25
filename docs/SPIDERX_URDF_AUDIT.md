# SpiderX URDF / xacro Engineering Audit

| Item | Value |
|---|---|
| Audited baseline | `main` @ `03d006a` (initial commit, `spiderx_description` only) |
| Also reviewed | PR #1 head `4d13f9e` (bringup/nav scaffolding), PR #2 head `e489e82` (Fortress scaffolding, stacked on PR #1) |
| Primary source | `src/spiderx_description/urdf/spiderx.urdf.xacro` (fusion2urdf export) |
| Included files | `materials.xacro`, `spiderx.trans`, `spiderx.gazebo` |
| Tooling used | xacro 2.0.13, `check_urdf` (urdfdom), sdformat 12.8.0 (`ign sdf`), Ignition Gazebo **6.16.0 (Fortress)**, ros_gz 0.244.20, plus a standalone numpy STL/inertia analyser |

This audit was written **before** any change to the robot model. Every number below was
read from the files or measured from the STL meshes; nothing was estimated unless it is
explicitly marked as an observation or an inference.

---

## 1. Executive summary

1. **The model itself is structurally sound.** It has 31 links and 30 joints (12 revolute, 18 fixed), a single root (`dummy_link`), no disconnected links and no cycles. All 30 mesh files resolve, and every revolute axis is a unit vector. `check_urdf` passes.
2. **All inertial data is CAD-derived and physically valid.** I recomputed mass properties from the STL meshes, and the COM matches to ≤ 0.01 mm and the inertia matches to exporter rounding for every link. All 30 inertia tensors are positive-definite and satisfy the triangle inequality.
3. **The CAD material is almost certainly the Fusion 360 default (steel).** The implied density is 7,850 kg/m³ for every part, which makes the total mass **7.29 kg**. That is geometrically consistent, but probably not the real robot. **Do not "fix" it without weighed parts or real material data.**
4. **8 servo-horn links carry placeholder inertia** (`1e-6` on the diagonal). The true CAD value is 1.2–2.3 × 10⁻⁷ kg·m²; the exporter rounded it to 0 at 6 decimals and it was replaced by hand. The placeholder is valid, conservative, and negligible after fixed-joint lumping.
5. **`main` does not build.** `CMakeLists.txt` installs a `config/` directory that does not exist, and `spiderx.gazebo` references the missing `config/controllers.yaml`. Verified: `colcon build` fails with `ament_cmake_symlink_install_directory() can't find .../config`.
6. **The model is Gazebo-Classic-only.** `spiderx.gazebo` hard-includes `libgazebo_ros2_control.so` and 30 `Gazebo/Silver` material scripts, and `spiderx.trans` uses the `gazebo_ros2_control/GazeboSystem` hardware plugin. Verified: these leak unchanged into the SDF that Fortress receives.
7. **`base_link` does not follow REP-103.** From the leg naming, *forward* is **+y** and *right* is **+x** (+z is up). The `base_link` origin sits near a corner of the body footprint, not at the body centre.
8. **There is no lidar or sensor frame on `main`.** PR #1/#2 add `lidar_link` at `(0.09, 0.0, 0.20)`. That point floats about 7.4 cm above the top plate and sits 4.5 cm / 3.9 cm off the body centre.
9. **PR #2's Fortress launch never spawns the robot.** Verified on Fortress 6.16: it runs `gz sdf -p`, but `gz` does not exist in Fortress, so it fails with exit 127. The empty `/tmp` SDF is still passed to `create`, and only `ground_plane` exists in the world.
10. **Passive physics on Fortress is stable with the existing mesh collisions** (verified, §9). No collision simplification is required.

---

## 2. Kinematic tree (zero configuration)

Legend: `==[name]==>` fixed joint, `--(name ⟲ axis)-->` revolute joint. Positions are the
child-frame origin in the `base_link` frame, in metres.

```
dummy_link                                  (root, no inertial – KDL convention)
└==[dummy_joint]==> base_link               (0, 0, 0)      body shell
    ├==[Rigid 31]==> top_1                  (0.086, -0.045, 0.107)   top plate
    │
    ├==[Rigid 1]==> b_s4_1                  (0.005,  0.0024, 0.091)  hip servo body  ── LF leg
    │   └--(lf_hip ⟲ +y)--> b_s4_c_horn_1
    │       └==[Rigid 12]==> lf_holder_suppor1_1
    │           └==[Rigid 16]==> lf_holder_1
    │               └--(lf_thigh_joint ⟲ -x)--> lf_c_horn_1
    │                   └==[Rigid 25]==> lf_thigh_1
    │                       └--(lf_foot_joint ⟲ +x)--> lf_foot_1     (-0.0367, -0.0389, 0.0110)
    │
    ├==[Rigid 3]==> b_s2_1                  (0.097,  0.0024, 0.091)  ── RF leg
    │   └--(rf_hip ⟲ +y)--> b_s2_c_horn_1
    │       └==[Rigid 11]==> rf_holder_suppor1_1
    │           └==[Rigid 14]==> rf_holder_1
    │               └--(rf_thigh_joint ⟲ +x)--> rf_c_horn_1
    │                   └==[Rigid 23]==> rf_thigh_1
    │                       └--(rf_foot_joint ⟲ -x)--> rf_foot_1     ( 0.1387, -0.0389, 0.0110)
    │
    ├==[Rigid 4]==> b_s3_1                  (0.015, -0.0924, 0.091)  ── LR leg
    │   └--(lr_hip ⟲ -y)--> b_s3_c_horn_1
    │       └==[Rigid 10]==> lr_holder_suppor1_1
    │           └==[Rigid 15]==> lr_holder_1
    │               └--(lr_thigh_joint ⟲ -x)--> lr_c_horn_1
    │                   └==[Rigid 24]==> lr_thigh_1
    │                       └--(lr_foot_joint ⟲ -x)--> lr_foot_1     (-0.0307, -0.2271, 0.0110)
    │
    └==[Rigid 2]==> b_s1_1                  (0.097, -0.0924, 0.091)  ── RR leg
        └--(rr_hip ⟲ -y)--> b_s1_c_horn_1
            └==[Rigid 9]==> rr_holder_suppor1_1
                └==[Rigid 13]==> rr_holder_1
                    └--(rr_thigh_joint ⟲ +x)--> rr_c_horn_1
                        └==[Rigid 18]==> rr_thigh_1
                            └--(rr_foot_joint ⟲ -x)--> rr_foot_1     ( 0.1387, -0.2271, 0.0110)
```

### 2.1 Leg chains (exact joint order)

| Leg | Servo body | Chain (fixed joints in brackets) | Actuated order |
|---|---|---|---|
| LF | `b_s4_1` | [Rigid 1] → **lf_hip** → [Rigid 12] → [Rigid 16] → **lf_thigh_joint** → [Rigid 25] → **lf_foot_joint** | lf_hip, lf_thigh_joint, lf_foot_joint |
| RF | `b_s2_1` | [Rigid 3] → **rf_hip** → [Rigid 11] → [Rigid 14] → **rf_thigh_joint** → [Rigid 23] → **rf_foot_joint** | rf_hip, rf_thigh_joint, rf_foot_joint |
| LR | `b_s3_1` | [Rigid 4] → **lr_hip** → [Rigid 10] → [Rigid 15] → **lr_thigh_joint** → [Rigid 24] → **lr_foot_joint** | lr_hip, lr_thigh_joint, lr_foot_joint |
| RR | `b_s1_1` | [Rigid 2] → **rr_hip** → [Rigid 9] → [Rigid 13] → **rr_thigh_joint** → [Rigid 18] → **rr_foot_joint** | rr_hip, rr_thigh_joint, rr_foot_joint |

Each leg is a standard 3-DOF chain:

- **hip**: abduction/adduction about the body's longitudinal axis (±y).
- **thigh**: flexion about the lateral axis (±x).
- **foot**: the knee, flexing about the lateral axis (±x).

### 2.2 How the SDF sees it (verified with sdformat 12)

sdformat lumps every fixed joint. The Fortress model therefore has **13 links**:

- `dummy_link`: the lumped body, 2.352 kg.
- For each leg: `*_c_horn_1` at the hip, `*_c_horn_1` at the thigh, and `*_foot_1`.

It keeps the **12 revolute joints** under their URDF names. Lumped links and fixed joints
are preserved as SDF `<frame>`s. The canonical link in simulation is **`dummy_link`**.

---

## 3. Frame conventions

| Property | Finding | Evidence |
|---|---|---|
| +z | Up | Top plate at z = 0.107–0.111; feet at z = −0.0545 |
| Forward | **+y** (inferred) | Servo mounts named `*f*` sit at y = +0.0024 and `*r*` (rear) at y = −0.0924 |
| Right | **+x** (inferred) | Servo mounts named `r*` sit at x = 0.097 and `l*` at x ≈ 0.01; with z up and forward +y, right-handedness requires right = +x |
| REP-103 | **Not compliant** | REP-103 expects +x forward and +y left. `base_link` is yawed −90° relative to that |
| `base_link` origin | CAD origin, near a corner of the body footprint (shell bbox x −0.004…0.106, y −0.100…0.010), **not** the body centre | Body centre is at (0.051, −0.045). The shell spans z 0.040–0.107, and the feet are 0.0545 below the origin |
| Robot bounding box (zero pose) | x −0.0427…0.1447, y −0.2594…0.1035, z −0.0545…0.1110 | Size 0.187 × 0.363 × 0.166 m, including the 0.297 m top plate |
| Foot contact centroids | LF (−0.034, 0.042), RF (0.135, 0.042), LR (−0.034, −0.146), RR (0.135, −0.146) | All four feet bottom out at exactly z = −0.0545: a consistent standing pose |
| Whole-robot COM (zero pose) | (0.051, −0.058, 0.070) | Inside the support polygon, 6 mm behind its centre |

---

## 4. Links: visual, collision and inertial status

Notes that apply to every row:

- **COM** is in the link frame, in metres. **Inertia** is in kg·m² about the COM.
- **Collision** is identical to the visual mesh and origin for every link.
- **Mesh** means `file://$(find spiderx_description)/meshes/<name>.stl` at scale 0.001.
- **CAD ✓** means the value was re-derived from the mesh at uniform density and matches to exporter rounding.

| Link | Mesh (triangles) | Mass kg | COM (x, y, z) | ixx / iyy / izz | ixy, ixz, iyz | Status |
|---|---|---|---|---|---|---|
| dummy_link | – | – | – | – | – | Root, no inertial (intentional for KDL; lumped with `base_link` in SDF) |
| base_link | base_link (1912) | 0.7776 | 0.0510, −0.0450, 0.0925 | 1.511e-3 / 1.544e-3 / 2.600e-3 | 0, 0, 0 | CAD ✓ |
| b_s1_1 (RR) | b_s1_1 (652) | 0.2623 | −0.0050, 0.0103, −0.0240 | 7.3e-5 / 4.6e-5 / 4.5e-5 | 0, 0, 0 | CAD ✓ |
| b_s2_1 (RF) | b_s2_1 (652) | 0.2623 | −0.0050, −0.0103, −0.0240 | 7.3e-5 / 4.6e-5 / 4.5e-5 | 0, 0, 0 | CAD ✓ |
| b_s3_1 (LR) | b_s3_1 (652) | 0.2623 | −0.0050, 0.0103, −0.0240 | 7.3e-5 / 4.6e-5 / 4.5e-5 | 0, 0, 0 | CAD ✓ |
| b_s4_1 (LF) | b_s4_1 (652) | 0.2623 | 0.0050, −0.0103, −0.0240 | 7.3e-5 / 4.6e-5 / 4.5e-5 | 0, 0, 0 | CAD ✓ |
| b_s1_c_horn_1 | b_s1_c_horn_1 (1888) | 0.00491 | 0, 0.0003, 0 | 1e-6 / 1e-6 / 1e-6 | 0, 0, 0 | **Placeholder inertia** (CAD: 1.2e-7 / 2.3e-7 / 1.2e-7); mass and COM are CAD ✓ |
| b_s2_c_horn_1 | b_s2_c_horn_1 (1888) | 0.00491 | 0, −0.0003, 0 | 1e-6 ×3 | 0 | **Placeholder inertia** (same as above) |
| b_s3_c_horn_1 | b_s3_c_horn_1 (1888) | 0.00491 | 0, 0.0003, 0 | 1e-6 ×3 | 0 | **Placeholder inertia** |
| b_s4_c_horn_1 | b_s4_c_horn_1 (1888) | 0.00491 | 0, −0.0003, 0 | 1e-6 ×3 | 0 | **Placeholder inertia** |
| rr_holder_suppor1_1 | (1740) | 0.1081 | 0.0089, −0.0043, 0 | 2.7e-5 / 4.0e-5 / 1.5e-5 | 1e-6, 0, 0 | CAD ✓ |
| rf_holder_suppor1_1 | (1740) | 0.1081 | 0.0089, 0.0043, 0 | 2.7e-5 / 4.0e-5 / 1.5e-5 | −1e-6, 0, 0 | CAD ✓ |
| lr_holder_suppor1_1 | (1740) | 0.1081 | −0.0089, −0.0043, 0 | 2.7e-5 / 4.0e-5 / 1.5e-5 | −1e-6, 0, 0 | CAD ✓ |
| lf_holder_suppor1_1 | (1740) | 0.1081 | −0.0089, 0.0043, 0 | 2.7e-5 / 4.0e-5 / 1.5e-5 | 1e-6, 0, 0 | CAD ✓ |
| rr_holder_1 | (1300) | 0.5981 | −0.0097, −0.0240, −0.0165 | 2.12e-4 / 1.86e-4 / 1.74e-4 | 0, 0, −1e-6 | CAD ✓ |
| rf_holder_1 | (1300) | 0.5981 | −0.0097, 0.0240, −0.0165 | 2.12e-4 / 1.86e-4 / 1.74e-4 | 0, 0, −1e-6 | CAD ✓ |
| lr_holder_1 | (1300) | 0.5981 | 0.0097, −0.0240, −0.0165 | 2.12e-4 / 1.86e-4 / 1.74e-4 | 0, 0, 1e-6 | CAD ✓ |
| lf_holder_1 | (1300) | 0.5981 | 0.0097, 0.0240, −0.0165 | 2.12e-4 / 1.86e-4 / 1.74e-4 | 0, 0, 1e-6 | CAD ✓ |
| rr_c_horn_1 | (1888) | 0.00491 | −0.0003, 0, 0 | 1e-6 ×3 | 0 | **Placeholder inertia** (CAD: 2.3e-7 / 1.2e-7 / 1.2e-7) |
| rf_c_horn_1 | (1888) | 0.00491 | −0.0003, 0, 0 | 1e-6 ×3 | 0 | **Placeholder inertia** |
| lr_c_horn_1 | (1888) | 0.00491 | 0.0003, 0, 0 | 1e-6 ×3 | 0 | **Placeholder inertia** |
| lf_c_horn_1 | (1888) | 0.00491 | 0.0003, 0, 0 | 1e-6 ×3 | 0 | **Placeholder inertia** |
| rr_thigh_1 | (2100) | 0.4204 | 0.0090, −0.0323, −0.0323 | 5.32e-4 / 2.78e-4 / 2.78e-4 | −1e-6, −1e-6, −2.43e-4 | CAD ✓ (principal 3.5e-5 / 5.21e-4 / 5.32e-4) |
| rf_thigh_1 | (2100) | 0.4204 | 0.0090, −0.0323, −0.0323 | same | −1e-6, −1e-6, −2.43e-4 | CAD ✓ |
| lr_thigh_1 | (2100) | 0.4204 | −0.0090, −0.0323, −0.0323 | same | 1e-6, 1e-6, −2.43e-4 | CAD ✓ |
| lf_thigh_1 | (2100) | 0.4204 | −0.0090, −0.0323, −0.0323 | same | 1e-6, 1e-6, −2.43e-4 | CAD ✓ |
| rf_foot_1 | (2808) | 0.0976 | −0.0030, 0.0304, −0.0202 | 1.78e-4 / 6.1e-5 / 1.18e-4 | 0, 0, 8.2e-5 | CAD ✓ (slender: principal 2.7e-6 / 1.763e-4 / 1.78e-4) |
| rr_foot_1 | (2808) | 0.0976 | −0.0030, 0.0304, −0.0202 | same | 0, 0, 8.2e-5 | CAD ✓ |
| lr_foot_1 | (2808) | 0.0976 | **−0.0030**, 0.0304, −0.0202 | same | 0, 0, 8.2e-5 | CAD ✓ (x sign matches the *right* feet – see K3) |
| lf_foot_1 | (2808) | 0.0976 | 0.0030, 0.0304, −0.0202 | same | 0, 0, 8.2e-5 | CAD ✓ |
| top_1 | (932) | 0.5253 | −0.0350, 0, 0.0020 | 3.404e-3 / 1.62e-4 / 3.565e-3 | 0, 0, 0 | CAD ✓ (thin plate: izz ≈ ixx + iyy) |

**Totals.** 7.288 kg in 30 massive links. The implied density is 7,847–7,858 kg/m³ for
**every** part, which is steel. The per-link inertia verdicts are in
[`URDF_INERTIA_AUDIT.md`](URDF_INERTIA_AUDIT.md).

**Mesh integrity.** All 30 STLs are binary and consistent in size. Every one is a closed
2-manifold (0 non-manifold edges) with 652–2,808 triangles, and every one resolves.

---

## 5. Joints

Every joint origin has `rpy = 0 0 0`. No joint has a `<dynamics>` element (no damping or
friction), and no joint has `<safety_controller>` or `<mimic>`.

### 5.1 Revolute (actuated) joints

| Joint | Parent → Child | Origin xyz | Axis | Lower / Upper (rad) | Range about +axis | Effort / Vel | Status |
|---|---|---|---|---|---|---|---|
| lf_hip | b_s4_1 → b_s4_c_horn_1 | 0.005, 0.0137, −0.01375 | 0 1 0 | −0.6109 / 0.6981 | [−0.611, 0.698] about +y | 100 / 100 | OK – limits are placeholders (E1) |
| rf_hip | b_s2_1 → b_s2_c_horn_1 | −0.005, 0.0137, −0.01375 | 0 1 0 | −0.6981 / 0.6109 | [−0.698, 0.611] about +y | 100 / 100 | OK |
| lr_hip | b_s3_1 → b_s3_c_horn_1 | −0.005, −0.0137, −0.01375 | 0 −1 0 | −0.6981 / 0.6109 | [−0.611, 0.698] about +y | 100 / 100 | OK |
| rr_hip | b_s1_1 → b_s1_c_horn_1 | −0.005, −0.0137, −0.01375 | 0 −1 0 | −0.6109 / 0.6981 | [−0.698, 0.611] about +y | 100 / 100 | OK |
| lf_thigh_joint | lf_holder_1 → lf_c_horn_1 | −0.0137, 0.01375, −0.005 | −1 0 0 | −0.6109 / 0.7854 | [−0.785, 0.611] about +x | 100 / 100 | OK |
| rf_thigh_joint | rf_holder_1 → rf_c_horn_1 | 0.0137, 0.01375, −0.005 | 1 0 0 | −0.7854 / 0.6109 | [−0.785, 0.611] about +x | 100 / 100 | OK |
| lr_thigh_joint | lr_holder_1 → lr_c_horn_1 | −0.0137, −0.03425, −0.005 | −1 0 0 | −0.6109 / 0.7854 | [−0.785, 0.611] about +x | 100 / 100 | OK |
| rr_thigh_joint | rr_holder_1 → rr_c_horn_1 | 0.0137, −0.03425, −0.005 | 1 0 0 | −0.7854 / 0.6109 | [−0.785, 0.611] about +x | 100 / 100 | OK |
| lf_foot_joint | lf_thigh_1 → lf_foot_1 | −0.012, −0.077782, −0.077782 | 1 0 0 | −0.6981 / 0.4363 | [−0.698, 0.436] about +x | 100 / 100 | OK |
| rf_foot_joint | rf_thigh_1 → rf_foot_1 | 0.012, −0.077782, −0.077782 | −1 0 0 | −0.4363 / 0.6981 | [−0.698, 0.436] about +x | 100 / 100 | OK |
| lr_foot_joint | lr_thigh_1 → lr_foot_1 | **−0.006**, −0.077782, −0.077782 | **−1** 0 0 | −0.4363 / 0.6981 | [−0.698, 0.436] about +x | 100 / 100 | OK, but the sign convention differs from `lf_foot_joint` (K3) |
| rr_foot_joint | rr_thigh_1 → rr_foot_1 | 0.012, −0.077782, −0.077782 | −1 0 0 | −0.4363 / 0.6981 | [−0.698, 0.436] about +x | 100 / 100 | OK |

Once every range is expressed about the positive axis, the model is physically
left/right symmetric:

- **Hips:** the right-side range [−0.698, 0.611] mirrors to the left-side range [−0.611, 0.698].
- **Thighs and feet:** all four legs have the same range.

Every lower limit is below its upper limit, and every limit is present.

### 5.2 Fixed joints

| Joint | Parent → Child | Origin xyz | Status |
|---|---|---|---|
| dummy_joint | dummy_link → base_link | 0, 0, 0 | OK (KDL root pattern) |
| Rigid 1 | base_link → b_s4_1 | 0.005, 0.0024, 0.091 | OK (name contains a space – N1) |
| Rigid 2 | base_link → b_s1_1 | 0.097, −0.0924, 0.091 | OK |
| Rigid 3 | base_link → b_s2_1 | 0.097, 0.0024, 0.091 | OK |
| Rigid 4 | base_link → b_s3_1 | 0.015, −0.0924, 0.091 | OK |
| Rigid 9 | b_s1_c_horn_1 → rr_holder_suppor1_1 | 0, 0.001, 0 | OK |
| Rigid 10 | b_s3_c_horn_1 → lr_holder_suppor1_1 | 0, 0.001, 0 | OK |
| Rigid 11 | b_s2_c_horn_1 → rf_holder_suppor1_1 | 0, −0.001, 0 | OK |
| Rigid 12 | b_s4_c_horn_1 → lf_holder_suppor1_1 | 0, −0.001, 0 | OK |
| Rigid 13 | rr_holder_suppor1_1 → rr_holder_1 | 0.02, −0.01, 0.0165 | OK |
| Rigid 14 | rf_holder_suppor1_1 → rf_holder_1 | 0.02, 0.01, 0.0165 | OK |
| Rigid 15 | lr_holder_suppor1_1 → lr_holder_1 | −0.02, −0.01, 0.0165 | OK |
| Rigid 16 | lf_holder_suppor1_1 → lf_holder_1 | −0.02, 0.01, 0.0165 | OK |
| Rigid 18 | rr_c_horn_1 → rr_thigh_1 | 0.001, 0, 0 | OK |
| Rigid 23 | rf_c_horn_1 → rf_thigh_1 | 0.001, 0, 0 | OK |
| Rigid 24 | lr_c_horn_1 → lr_thigh_1 | −0.001, 0, 0 | OK |
| Rigid 25 | lf_c_horn_1 → lf_thigh_1 | −0.001, 0, 0 | OK |
| Rigid 31 | base_link → top_1 | 0.086, −0.045, 0.107 | OK |

Every visual and collision origin equals the negative of the link's zero-pose position in
`base_link`, which is the fusion2urdf convention. The CAD assembly is therefore reproduced
exactly at q = 0.

---

## 6. Actuation, transmissions and ros2_control

- `spiderx.trans` contains **no `<transmission>` elements**. It holds one `<ros2_control name="GazeboSystem" type="system">` block with the hardware plugin **`gazebo_ros2_control/GazeboSystem`**, which is **Gazebo Classic only**.
- That block covers all 12 revolute joints, each with a `position` command interface and `position` + `velocity` state interfaces.
- `spiderx.gazebo` loads **`libgazebo_ros2_control.so`**, also Classic only, with parameters from `$(find spiderx_description)/config/controllers.yaml`. **That file does not exist on `main`.** PR #1 adds it, with a `joint_state_broadcaster` and a `forward_command_controller` over the 12 joints.
- No Fortress control integration exists. The Fortress equivalent would be `gz_ros2_control` (`ign_ros2_control/IgnitionSystem` on Humble), which is not installed or validated.

## 7. Sensors

- **`main`:** there are no sensor links and no sensor plugins.
- **PR #1:** adds `lidar_link`, a cylinder of r = 0.03 and h = 0.03 with an unlabelled 0.2 kg / 1e-4 inertia. It is attached by `lidar_joint` to `base_link` at `(0.09, 0.0, 0.20)`, rpy 0.
  - The top plate's upper surface is at z = 0.111, so the lidar body floats about 7.4 cm above it.
  - It is also off the plate centre (0.051, −0.045).
  - With yaw 0, the scan's 0° direction points to the robot's **right**.
  - PR #1 also adds a Classic `type="ray"` sensor with `libgazebo_ros_ray_sensor.so`.
- **PR #2:** adds a Fortress `gpu_lidar` block on the gz topic `/scan`. It has no frame override, so Fortress would stamp scans with a scoped gz frame name instead of `lidar_link`.

## 8. Mesh URIs

- There are 30 visual and 30 collision references, all of the form `file://$(find spiderx_description)/meshes/<link>.stl` with `scale="0.001 0.001 0.001"`, converting mm to m. The scale is consistent with the 0.36 m robot length.
- All 30 files exist, and every link references its own mesh. There are no unused mesh files.
- `$(find …)` expands at xacro time to an absolute install path. Fortress reads `file://` absolute URIs directly: verified by spawning. RViz does too.
  - **Portability caveat:** the expanded URDF is not relocatable to another machine. That is acceptable for a local simulation.

## 9. Observed passive physics on Fortress (unmodified geometry)

**Test set-up:**

- The `main` URDF with only the Classic plugin removed.
- A plain ground plane and DART physics at 1 ms.
- Spawned with `ros_gz_sim create` at z = 0.075.

**Result:**

- The root link dropped and settled by about 4 s at z ≈ 0.0122 m, with roll/pitch below 1e-4 rad.
- Over the following 8 s the pose stayed constant to 1e-6.
- There was no jitter, no NaN and no ground penetration.
- The legs fold because the joints are unactuated and undamped, which is expected for a passive model.
- Real-time factor was 0.45 on the cloud CPU without rendering.

**Conclusion:** the mesh collisions are acceptable, and **no collision simplification is required**.

---

## 10. Gazebo-Classic-only content

| Location | Content | Fortress impact |
|---|---|---|
| `spiderx.gazebo` | `<plugin filename="libgazebo_ros2_control.so">` | Copied into the SDF; Fortress logs `[Err] Failed to load system plugin [libgazebo_ros2_control.so]` (verified) |
| `spiderx.gazebo` | 30 × `<material>Gazebo/Silver</material>` | Classic OGRE material scripts; ignored by Fortress (the URDF `rgba` colour is used instead) |
| `spiderx.gazebo` | `mu1/mu2 = 0.2`, `self_collide`, `gravity` | **Portable**: sdformat converts them to SDF `<surface>` and link properties. The `mu = 0.2` value is the fusion2urdf template default, not a measurement |
| `spiderx.trans` | `gazebo_ros2_control/GazeboSystem` | Classic only; ignored by the SDF parser |
| PR #1 `spiderx.gazebo` | `type="ray"` + `libgazebo_ros_ray_sensor.so` | Classic only |
| PR #1 launch files | `gazebo_ros`, `spawn_entity.py`, `GAZEBO_MODEL_PATH`, `gzserver`/`gzclient` (through `gazebo_ros/gazebo.launch.py`) | Classic only |

---

## 11. Problems found

Severity levels: **C** critical, **H** high, **M** medium, **L** low, **I** informational.

### Build and structure
- **B1 (C)** – `main` fails `colcon build`, because `CMakeLists.txt` installs a non-existent `config/` directory. Verified.
- **B2 (H)** – `spiderx.gazebo` points at a missing `config/controllers.yaml`.
- **S1 (I)** – The tree is valid: one root, 31 links, 30 joints, no orphans and no cycles (`check_urdf` passes).
- **S2 (I)** – The massless root `dummy_link` is intentional. In SDF, `base_link` and all fixed children are lumped into `dummy_link`, so the simulator's canonical link is named `dummy_link`.

### Kinematics
- **K1 (I)** – All 12 revolute axes are unit vectors along ±x or ±y, and all ranges are left/right symmetric.
- **K2 (M)** – Sign conventions vary per joint. Rear hips use −y and front hips +y; the thighs mirror left/right. A gait or IK layer needs a per-joint sign table.
- **K3 (M)** – `lr_foot_joint` looks like a **non-mirrored copy of a right foot**:
  - Its axis is −x, like RR and RF, where LF uses +x.
  - Its COM x is −0.003, as on the right feet.
  - Its origin x is −0.006, where the other feet use ±0.012.

  The mesh itself is placed symmetrically (bbox x −0.0367…−0.0307, identical to LF). The origin offset lies **along the rotation axis**, so it has **no kinematic effect**. The axis sign must match the physical servo, so **do not change it without hardware evidence**.
- **K4 (M)** – Effort 100 N·m and velocity 100 rad/s on every joint are placeholders. Hobby servos manage roughly 1–3 N·m and 5–7 rad/s.
- **K5 (L)** – No joint has damping or friction, so the passive model folds under gravity. This is expected, but it must be tuned before control work.

### Collision and physics
- **P1 (I)** – Full-resolution visual meshes are used as collision meshes. They are closed manifolds and passive contact on Fortress is stable (§9), so no change is required.
- **P2 (M)** – Every part has steel density, for a total of 7.29 kg (§4). This is consistent with the CAD but probably not with the real robot.
- **P3 (L)** – The 8 horn links use placeholder inertia `1e-6`, against a CAD value of about 2e-7. It is valid and negligible after lumping.
- **P4 (L)** – `mu = 0.2` on every link is the exporter template default, not a measured friction coefficient.

### Naming
- **N1 (L)** – The fixed joints `Rigid 1` … `Rigid 31` contain spaces and are numbered non-contiguously. This is legal in URDF and accepted by sdformat 12, but awkward on the command line. Nothing references them.
- **N2 (L)** – `holder_suppor1` is a typo. The servo numbering `b_s1..b_s4` does not follow leg order: b_s1 = RR, b_s2 = RF, b_s3 = LR, b_s4 = LF.
- **N3 (I)** – The included files wrap their content in `<robot name="ros_file">`, which xacro ignores. The URDF files use CRLF line endings.

### Transforms
- **T1 (H)** – `base_link` breaks REP-103: forward is +y, not +x. Nav2, `cmd_vel` and odometry assume +x forward.
- **T2 (M)** – The `base_link` origin is near a corner of the body footprint. The body centre is at (0.051, −0.045), and the zero-pose feet are 0.0545 m below the origin. Any Nav2 footprint must be expressed in these coordinates.
- **T3 (I)** – There is no `base_footprint`, `odom` or `lidar_link` on `main`.

### Mesh paths
- **M1 (I)** – All URIs resolve. The expanded URDF embeds absolute paths.

---

## 12. Review of PR #1 / PR #2 (prior scaffolding)

| # | Issue | Evidence |
|---|---|---|
| R1 | Fortress spawn never happens: `gz sdf -p` doesn't exist on Fortress (the CLI is `ign`) | Verified: `bash: gz: command not found`, exit 127; world contains only `ground_plane` |
| R2 | `OnProcessExit` spawns even when SDF generation failed; hard-coded `/tmp/spiderx_fortress_model.sdf` | Verified: `create` loads an empty file (`XML_ERROR_EMPTY_DOCUMENT`) |
| R3 | Scan bridge is bidirectional (`@`), not GZ→ROS | `/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan` |
| R4 | No `/clock` bridge, although every node runs with `use_sim_time: true` | Launch file |
| R5 | `joint_state_publisher` publishes fabricated zero joint states, unrelated to the simulated legs | `spiderx_bringup/launch/fortress.launch.py` |
| R6 | Lidar frame not overridden: the scan `frame_id` will not be `lidar_link` | No `ignition_frame_id` |
| R7 | `lidar_link` floats 7.4 cm above the top plate, off-centre, and yaw 0 points it right | §7 |
| R8 | World uses Garden-style `gz-sim-*` plugin names | Fortress 6.16 accepts them through aliasing (verified); canonical Fortress names are `ignition-gazebo-*-system` |
| R9 | `GZ_SIM_RESOURCE_PATH` is overwritten rather than appended, and `IGN_GAZEBO_RESOURCE_PATH` (Fortress's native variable) is not set | Launch file |
| R10 | The legacy Classic bringup starts the **hardware** RPLidar driver on `/dev/ttyUSB0` during simulation | `spiderx_bringup/launch/gazebo.launch.py` |
| R11 | Nav2 footprint `[[±0.20, ±0.14]]` is centred on the `base_link` origin, but the robot spans x −0.043…0.145 and y −0.259…0.104 in that frame; no `/odom` source exists | `spiderx_navigation/config/*.yaml` |
| R12 | `spiderx_bringup` exec-depends on nav/mapping/localization, `nav2_bringup` and `gazebo_ros`, none of which its Fortress launch uses | `package.xml` |

---

## 13. Do Not Change Without CAD Evidence

1. **Masses, COMs and inertia tensors of all links.** They are CAD-derived and mesh-verified. Changing them needs a real material assignment or weighed parts, not guesses.
2. **Mesh files, mesh scale, and every visual/collision `<origin>`.**
3. **Every joint origin (xyz/rpy)**, including the `lr_foot_joint` x = −0.006 offset.
4. **Joint axis directions and signs**: `lr_foot_joint` −x versus `lf_foot_joint` +x, and the rear −y versus front +y hips. They must match the physical servo mounting.
5. **Joint lower/upper limits.**
6. **The `base_link` origin and orientation.** Moving or re-orienting it moves every child frame. REP-103 alignment should be added as a *new* frame, not by editing CAD frames.
7. **Link names and actuated joint names.** `spiderx.trans`, `controllers.yaml` and TF consumers depend on them.
8. **The horn placeholder inertia values.** They are valid; replacing them means committing to a density model (see P2).
9. **Effort and velocity limits.** They are placeholders, but the replacement needs the servo datasheet, which is hardware evidence.

## 14. Safe Simulation Repairs

These repairs need no CAD evidence and leave the geometry and existing kinematics unchanged:

1. **Fix the build.** Provide `config/controllers.yaml` for the legacy Classic ros2_control path. The model does not change.
2. **Make simulator-specific content selectable with a xacro argument.** The Fortress description must not carry `libgazebo_ros2_control.so`, and the legacy Classic output must stay byte-for-byte equivalent.
3. **Add `lidar_link` and a fixed `lidar_joint` to `base_link`.**
   - Place it at the measured top-plate centre (0.051, −0.045) on the measured top surface (z = 0.111).
   - Label its mass and inertia `SIMULATION_ESTIMATE`.
   - Add a Fortress `gpu_lidar` sensor with `ignition_frame_id=lidar_link`.
4. **Publish real joint states** with the Fortress `JointStatePublisher` system, bridged to `/joint_states`, instead of fabricating them.
5. **Derive the spawn height from the measured geometry.** The lowest foot point is 0.0545 m below the root at q = 0, so add clearance above that.
6. **Annotate the placeholder horn inertia** with a comment only; the values stay the same.

Not required, and therefore **not done**:

- collision simplification (§9)
- inertia replacement (no value is invalid)
- axis or limit changes (none are invalid)

## 15. Prioritized repair plan

| Priority | Work | Type |
|---|---|---|
| **P0** | 1. Fix the build (B1, B2).<br>2. Spawn in Fortress from the xacro via `ros_gz_sim create -topic robot_description`, using one source of truth and no SDF temp file.<br>3. Keep Classic plugins out of the Fortress description. | Safe |
| **P1** | 1. Add `lidar_link` and the `gpu_lidar` sensor.<br>2. Bridge `/scan` and `/clock` one-way from Gazebo to ROS.<br>3. Publish `/joint_states` from Gazebo.<br>4. Set `GZ_SIM_RESOURCE_PATH` and `IGN_GAZEBO_RESOURCE_PATH` by appending, not overwriting.<br>5. Move the hardware lidar driver into its own launch file. | Safe |
| **P2** | 1. Fortress world with canonical system names, ground, light and static obstacles so `/scan` shows returns.<br>2. Headless option.<br>3. Documentation and validation script. | Safe |
| **P3** | 1. REP-103 `base_footprint` frame (T1, T2).<br>2. Real masses (P2).<br>3. Servo effort and velocity limits and joint damping (K4, K5).<br>4. Validated `gz_ros2_control` plus gait/IK and `cmd_vel`.<br>5. Odometry.<br>6. Correct Nav2 footprint (R11). | **Needs evidence or design decisions: future work** |
