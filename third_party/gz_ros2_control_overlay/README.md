# gz_ros2_control overlay (simulation only)

**What it is.** Upstream `ros-controls/gz_ros2_control` tag `0.7.15`, commit `bfc5845aa7f0250cbc384773dba5c9847d609b95` (Apache-2.0, `LICENSE.upstream`), plus one patch:
`patches/0001-plugin-node-parameter-overrides.patch`.

**Why it exists.** Stock 0.7.15 creates its `gz_ros2_control` node before it installs the plugin's `<parameters>` files as global arguments. That node's parameter overrides are therefore always empty, and `position_proportional_gain` always takes its default, 0.1. The full trace and an offline probe are in `docs/evidence/m55/s2_fixed_base_20261029T100210Z/analysis_offline_20261029/REVIEW_ADDENDUM.md`. The design is in `docs/S2_SIM_ACTUATOR_MODEL_DESIGN.md`.

## The patch, in summary

| File | Change |
|---|---|
| `src/gz_ros2_control_plugin.cpp` (+7/−1) | The plugin node is created with `plugin_node_options(files)`: the same `<parameters>` files, as node-local `--params-file` arguments. Remapping rules, the `robot_description` argument and the controller-manager setup are unchanged |
| `src/gz_system.cpp` (+8/−8) | The gain is declared through `declare_position_proportional_gain()` |
| `src/parameter_loading.hpp` (new, private) | The two helpers above, used by the production code and by the test |
| `test/test_parameter_loading.cpp` (new) + `CMakeLists.txt`, `package.xml` | Offline gtest, 9 cases. No Gazebo, no executor |

How `declare_position_proportional_gain()` behaves:
- The default stays 0.1, as upstream.
- A value that is not a finite double greater than 0 is reported, and `initSim()` fails. Values are never clamped or replaced.
- The gain is still read once at start-up.

Each test replays `Configure()`'s order and checks:
- the gain is 0.1 when unspecified;
- a configured 1.0 reaches the plugin node;
- a namespaced node is addressed by its full name or by a wildcard;
- later files take precedence;
- the controller manager, controller parameters and remapping are the same as with stock node creation;
- a second system on the same node reads the declared value;
- NaN, ±inf, 0, negative values and wrong types are rejected.

## Packaging

**Approach.** The repository holds a pin plus a patch plus a script. It vendors no upstream source.

`build_overlay.sh`:
1. Fetches the tag shallowly, sparse-checked-out to the package directory only.
2. Verifies the commit.
3. Applies the patch, after `git apply --check`.
4. Builds with the toolchain's own compilers into a new workspace outside the repository, and never touches `/opt/mm`.
5. Runs the gtest.
6. Writes `OVERLAY_PROVENANCE.txt`: commit, patch hash, compiler, and library hashes.

**Rebuilds.** It refuses a non-empty workspace and deletes nothing. To rebuild, remove the workspace yourself.

**Build flags:**
- `-include cstdint`: the toolchain's GCC 15 cannot otherwise compile `gtest_vendor`'s googletest 1.10.
- `CMAKE_INSTALL_RPATH_USE_LINK_PATH=ON`: the installed libraries find the toolchain's libraries, much as the stock libraries do with `$ORIGIN`.

## Commands

**Build** (network needed once, for the fetch):

```bash
source /opt/mm/activate.sh
third_party/gz_ros2_control_overlay/build_overlay.sh        # -> ~/spiderx_gz_overlay_ws
```

**Use.** The order matters: the overlay goes after the toolchain and before the SpiderX workspace.

```bash
source /opt/mm/activate.sh
source ~/spiderx_gz_overlay_ws/install/local_setup.bash
source <spiderx_ws>/install/setup.bash
ros2 pkg prefix gz_ros2_control      # must print ~/spiderx_gz_overlay_ws/install/gz_ros2_control
```

The Fortress launch files add `get_package_prefix('gz_ros2_control')/lib` to the Gazebo system-plugin path, so they follow this order. **A sourced overlay is necessary but not sufficient.** Before trusting a run, prove at runtime which library was loaded and which gain it read (`docs/S2_SIM_ACTUATOR_MODEL_DESIGN.md`, Phase B).

**Roll back.** Stop sourcing the overlay. Stock 0.7.15 then loads, ignores `gz_ros2_control_plant.yaml`, and runs with gain 0.1: the plant of every run before this change. Revert the configuration commit to remove the plant file as well.

## Limitations

- **Not runtime-verified.** Nothing here has run in Gazebo yet; that is Phase B and Phase C.
- **Pinned to 0.7.15.** The `humble` branch head (0.7.21, checked 2026-10-29) still creates the node first. This patch has not been offered upstream.
- **New parameter scope.** The plugin node now receives every entry addressed to it in the `<parameters>` files, wildcards included. Before, it received none. SpiderX's files address it only through `gz_ros2_control_plant.yaml`.
- **Pre-existing lint divergence.** `ament_uncrustify` reports one divergence in upstream `gz_ros2_control_plugin.cpp` (a lambda's indentation). It is present unpatched at 0.7.15; the patch's own files pass every linter.
