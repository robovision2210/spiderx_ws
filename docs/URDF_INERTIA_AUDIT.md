# SpiderX Inertia Audit

This document backs every inertial value in `src/spiderx_description/urdf/spiderx.urdf.xacro`.
It records:

- how each value was checked;
- which values come from CAD;
- which values are placeholders;
- which values are simulation estimates, and the justification for each.

The companion document is [`SPIDERX_URDF_AUDIT.md`](SPIDERX_URDF_AUDIT.md).

## Method

1. I parsed every `<inertial>` block (mass, COM origin, inertia tensor) from the xacro.
2. I loaded every link's STL mesh with the link's visual origin and the 0.001 scale.
3. For each closed mesh I computed the volume, centroid and inertia tensor at uniform density, using the signed-tetrahedron method. I then scaled the result to the declared mass.
4. I compared the declared values with the mesh-derived values.
5. I checked each declared tensor for physical validity:
   - it is symmetric;
   - all eigenvalues are positive (positive-definite);
   - the principal moments satisfy the triangle inequality, I₁ + I₂ ≥ I₃.

All 30 meshes are closed 2-manifolds (0 non-manifold edges), so the mesh-derived mass
properties are well defined.

## Results

| Link group | Count | Declared vs. mesh | Valid? | Classification |
|---|---|---|---|---|
| base_link | 1 | COM Δ 0.01 mm, inertia equal to 1e-6 rounding | Yes | **CAD-derived**, kept |
| b_s1_1 … b_s4_1 (hip servo bodies) | 4 | COM Δ 0.00 mm, inertia equal to rounding | Yes | **CAD-derived**, kept |
| *_holder_suppor1_1 | 4 | COM Δ 0.00 mm, inertia equal to rounding | Yes | **CAD-derived**, kept |
| *_holder_1 | 4 | COM Δ 0.00 mm, inertia equal to rounding | Yes | **CAD-derived**, kept |
| *_thigh_1 | 4 | COM Δ 0.00 mm, inertia equal to rounding | Yes; principal 3.5e-5 / 5.21e-4 / 5.32e-4 | **CAD-derived**, kept |
| *_foot_1 | 4 | COM Δ 0.00 mm, inertia equal to rounding | Yes; principal 2.7e-6 / 1.763e-4 / 1.78e-4 (slender part, near-degenerate by geometry) | **CAD-derived**, kept |
| top_1 | 1 | COM Δ 0.00 mm, inertia equal to rounding | Yes; thin plate, izz ≈ ixx + iyy | **CAD-derived**, kept |
| b_s*_c_horn_1, *_c_horn_1 (servo horns) | 8 | Mass and COM match; **inertia does not** (declared 1e-6, mesh 1.2e-7 / 2.3e-7 / 1.2e-7) | Yes | **Placeholder**, kept and labelled |
| lidar_link | 1 | No CAD | Yes | **SIMULATION_ESTIMATE** (new) |

No existing inertial value was invalid, zero or missing. **No CAD-derived value was
changed.**

## Material observation (not changed)

Every CAD-derived link has an implied density of **7,847–7,858 kg/m³**, which is steel.
That is the Fusion 360 default material. The total robot mass is therefore **7.29 kg**
(7.46 kg including the lidar estimate).

- The values are geometrically correct for steel parts, but printed or aluminium parts and hobby servos would weigh much less.
- **Correcting this needs evidence:** a real material assignment in CAD, or weighed parts.
- I did not rescale anything. Doing so would mean inventing masses.

## Placeholder values (existing, kept)

Eight servo-horn links declare `ixx = iyy = izz = 1e-6 kg·m²` with zero products of inertia:

- b_s1_c_horn_1, b_s2_c_horn_1, b_s3_c_horn_1, b_s4_c_horn_1
- rr_c_horn_1, rf_c_horn_1, lr_c_horn_1, lf_c_horn_1

Findings:

- **What happened:** the CAD tensor from the mesh is about 1.2e-7 to 2.3e-7 kg·m². fusion2urdf writes 6 decimal places, so it would have exported 0.0. Those zeros were later replaced by hand with `1e-6`.
- **Why it is kept:** the placeholder is positive-definite and satisfies the triangle inequality. It is about 4–8× larger than the CAD value, which is the conservative direction for solver stability.
- **Why its effect is negligible:** sdformat lumps every horn into the link across its fixed joint. The lumped hip link has I ≈ 2e-4 and the lumped thigh link I ≈ 5e-4, so the placeholder shifts either by less than 0.5 %.
- **Action taken:** each of the 8 values now carries an XML comment saying it is a placeholder. The numbers themselves are unchanged.

## SIMULATION_ESTIMATE values (new)

### lidar_link

| Field | Value | Justification |
|---|---|---|
| Mass | 0.17 kg | Nominal mass of an RPLidar A1-class 2D lidar, the sensor used by the project's hardware driver. **Not weighed.** |
| Shape model | Solid cylinder, r = 0.035 m, h = 0.040 m | Simplified sensor envelope, matching the visual/collision cylinder. |
| COM | (0, 0, −0.010) m in `lidar_link` | Cylinder centre. `lidar_link` is the scan plane, 0.030 m above the plate, so the centre is 0.010 m below it. |
| ixx = iyy | 7.47e-5 kg·m² | m(3r² + h²)/12 = 0.17 · (3·0.035² + 0.040²)/12 |
| izz | 1.041e-4 kg·m² | m·r²/2 = 0.17 · 0.035²/2 |
| Products | 0 | Axisymmetric about z |

Validity:

- Positive-definite.
- Triangle inequality holds: 7.47e-5 + 7.47e-5 ≥ 1.041e-4.

In the xacro, the block is labelled `SIMULATION_ESTIMATE`.

`lidar_link` is attached by a fixed joint, so Gazebo lumps it into the body link, which
goes from 2.352 kg to 2.522 kg. The estimate only affects the body's total mass and COM.
The COM moves 3 mm upward, from z = 0.0848 m to 0.0879 m, and x/y are unchanged because
the mount is centred.

## How to replace estimates later

1. Assign real materials in CAD, or weigh each part.
2. Re-export from Fusion 360, or update the `<mass>` values and scale the tensors by the mass ratio. Scaling is only valid for uniform-density parts.
3. Re-run the validity checks above: positive-definite, triangle inequality, COM inside the part's bounding box.
4. Remove the `PLACEHOLDER` and `SIMULATION_ESTIMATE` labels only for the values that are now measured.
