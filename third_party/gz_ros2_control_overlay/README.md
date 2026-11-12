# gz_ros2_control overlay (simulation only)

**What it is.** Upstream `ros-controls/gz_ros2_control` tag `0.7.15`, commit `bfc5845aa7f0250cbc384773dba5c9847d609b95` (Apache-2.0, `LICENSE.upstream`), plus two patches applied in order:
1. `patches/0001-plugin-node-parameter-overrides.patch`: parameter loading and gain validation (below).
2. `patches/0002-destructor-no-second-node-removal.patch`: the shutdown backport (**Patch 0002**).

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

## Patch 0002: no second node removal at shutdown

**Problem.** On SIGINT, controller_manager 2.51.0's pre-shutdown callback removes its own node from the plugin's executor. The 0.7.15 destructor then removes it again. rclcpp 16.0.13 throws "Node needs to be associated with an executor." from the destructor, and the Gazebo server aborts with SIGABRT. This happened with both the stock and the overlay library (`docs/evidence/m55/s2_phase_c_20261030T062140Z_adjudication_r1/shutdown_sigabrt_review.md`).

**Upstream provenance.**
- Humble commit `366137556d1fe3adf327a99b17261dcde549e655`, 2025-09-26: "Don't remove the node at destruction (#683) (#686)".
- It is cherry-picked from `90c951871312862879f66d2ceb8f92b7ecc8dc55` and released in 0.7.17 (`8fa21521b55c73a65d282012e378bbeb27927f33`).
- Only this fix is backported. The dependency stack is not upgraded.

**Change.**
- The destructor no longer removes the controller-manager node. It calls `stop_executor_thread()` (new private `src/shutdown_teardown.hpp`): cancel the executor if it exists, then join the spin thread if it is joinable.
- **Deviation from upstream:** upstream 0.7.17 dereferences the executor and joins the thread unconditionally. Here, a missing executor or a never-started thread is skipped, so a `Configure()` that returned early cannot cause a null dereference or an invalid join.
- No exception is caught and no cleanup is removed. Patch 0001's parameter loading and gain validation are unchanged.

**Offline gtest `test_shutdown_teardown`, 7 cases, no Gazebo:**
- the plugin destroyed before `Configure()`;
- after the controller manager's self-removal, a second removal still throws (the defect, reproduced);
- teardown after that shutdown sequence;
- teardown while spinning;
- teardown with an executor but no spin thread;
- teardown with no executor;
- teardown run twice.

The spinning cases wait until the executor actually spins: in rclcpp, a `cancel()` issued before `spin()` starts is lost. The first build of this patch hung for exactly that reason, in its test set-up; that build is kept, unused, as `~/spiderx_gz_overlay_ws_r2_failed_test_20261031`.

**Not yet runtime-verified.** A clean Gazebo exit still needs one launch-only check.

## Workspaces and provenance

Binary hashes depend on the build workspace path, which is embedded in the libraries. Every build records its own hashes in `OVERLAY_PROVENANCE.txt`; hashes are never carried over between builds.

| Workspace | Patches | `libgz_ros2_control-system.so` | `libgz_hardware_plugins.so` | Use |
|---|---|---|---|---|
| `~/spiderx_gz_overlay_ws` | 0001 | `ef37a04f…` | `c3db97f2…` | S2 Phase B and Phase C evidence. Kept unchanged for reproducibility |
| `~/spiderx_gz_overlay_ws_r2` | 0001 (`14238400…`) + 0002 (`520e73cc…`) | `3a8f2044…` | `06777e14…` | Current overlay for S3 preparation. Gtests: 9 + 7 pass |

## Packaging

**Approach.** The repository holds a pin plus a patch plus a script. It vendors no upstream source.

`build_overlay.sh`:
1. Fetches the tag shallowly, sparse-checked-out to the package directory only.
2. Verifies the commit.
3. Applies each patch in order, after `git apply --check`.
4. Builds with the toolchain's own compilers into a new workspace outside the repository, and never touches `/opt/mm`.
5. Runs the gtests (`test_parameter_loading`, `test_shutdown_teardown`).
6. Writes `OVERLAY_PROVENANCE.txt`: commit, patch hashes, compiler, and library hashes.

**Rebuilds.** It refuses a non-empty workspace and deletes nothing. To rebuild, remove the workspace yourself.

**Build flags:**
- `-include cstdint`: the toolchain's GCC 15 cannot otherwise compile `gtest_vendor`'s googletest 1.10.
- `CMAKE_INSTALL_RPATH_USE_LINK_PATH=ON`: the installed libraries find the toolchain's libraries, much as the stock libraries do with `$ORIGIN`.

## Commands

**Build** (network needed once, for the fetch):

```bash
source /opt/mm/activate.sh
third_party/gz_ros2_control_overlay/build_overlay.sh ~/spiderx_gz_overlay_ws_r2   # a new, empty workspace
```

**Use.** The order matters: the overlay goes after the toolchain and before the SpiderX workspace.

```bash
source /opt/mm/activate.sh
source ~/spiderx_gz_overlay_ws_r2/install/local_setup.bash
source <spiderx_ws>/install/setup.bash
ros2 pkg prefix gz_ros2_control      # must print ~/spiderx_gz_overlay_ws_r2/install/gz_ros2_control
```

The Fortress launch files add `get_package_prefix('gz_ros2_control')/lib` to the Gazebo system-plugin path, so they follow this order. **A sourced overlay is necessary but not sufficient.** Before trusting a run, prove at runtime which library was loaded and which gain it read (`docs/S2_SIM_ACTUATOR_MODEL_DESIGN.md`, Phase B).

**Roll back.** Stop sourcing the overlay. Stock 0.7.15 then loads, ignores `gz_ros2_control_plant.yaml`, and runs with gain 0.1: the plant of every run before this change. Revert the configuration commit to remove the plant file as well.

## Limitations

- **Runtime verification.** Patch 0001 was verified at runtime with the r1 workspace: S2 Phase B showed the loading, and Phase C measured the response law. Patch 0002 has **not** run in Gazebo: its clean shutdown is pending a launch-only check.
- **Pinned to 0.7.15.** The `humble` branch head (0.7.21, checked 2026-10-29) still creates the node first. Patch 0001 has not been offered upstream; patch 0002 is a backport of an upstream fix.
- **New parameter scope.** The plugin node now receives every entry addressed to it in the `<parameters>` files, wildcards included. Before, it received none. SpiderX's files address it only through `gz_ros2_control_plant.yaml`.
- **Pre-existing lint divergence.** `ament_uncrustify` reports one divergence in upstream `gz_ros2_control_plugin.cpp` (a lambda's indentation). It is present unpatched at 0.7.15; the patch's own files pass every linter.
