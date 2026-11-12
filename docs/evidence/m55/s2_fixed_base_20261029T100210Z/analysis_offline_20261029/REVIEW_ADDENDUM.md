# Review addendum to ANALYSIS.md (2026-10-29)

Added at review, before the first commit of this directory.
- `ANALYSIS.md` and every other file under `SHA256SUMS.analysis` are unchanged.
- This addendum and that manifest are covered by `SHA256SUMS.addendum`.
- The S2 verdict (FAIL), the data, the fits and the timeline are unaffected.

## 1. Source verification (upgrades §6 "supported, not source-verified" and §7)

**Upstream code:** `ros-controls/gz_ros2_control`, tag `0.7.15` = commit
`bfc5845aa7f0250cbc384773dba5c9847d609b95` (2025-05-23).

**Default gain and command equation** (`gz_ros2_control/src/gz_system.cpp`):
- L205–213: the default gain is 0.1 (`declare_parameter("position_proportional_gain", 0.1)`), declared on the plugin's own node.
- L684–703: `write()` sets the joint velocity command to −gain × (x_read − x_cmd) × update_rate.
- L565–592: `x_read` changes only in `read()`.

**Scheduling** (`gz_ros2_control/src/gz_ros2_control_plugin.cpp`):
- L531–565: `PreUpdate` calls `write()` on every physics step.
- L568–588: `PostUpdate` calls `read()` and `update()` only once `sim_period >= control_period`.
- L517–521: `control_period` = 1 / the controller manager's `update_rate`.

**Installed provenance:** the evidence supports, but cannot prove, that the installed binary is this code.
- The conda recipe builds `ros2-gbp/ign_ros2_control-release` tag `release/humble/gz_ros2_control/0.7.15-1` and lists no patches.
- That release tag's two source files are byte-identical to the upstream ones.
- The installed headers are identical to upstream.
- The installed libraries contain this version's log strings.

**Conclusion:** this sample-and-hold arrangement yields x[k+1] = x[k] + gain × (c[k] − x[k]), measured here at g = 0.1000000000. The derivation is in the design note under review: `docs/S2_SIM_ACTUATOR_MODEL_DESIGN.md`, uncommitted when this addendum was written.

## 2. Correction to §8: the proposed configuration path is not supported

§8 proposed setting `position_proportional_gain` in `spiderx_fortress_control.xacro`. **Version 0.7.15 does not support that, nor any other configuration route:**

- **No XML child for the gain.** The plugin reads only `parameters`, `robot_param`, `robot_param_node`, `controller_manager_name` and `ros/namespace` / `ros/remapping` (plugin L275–349).
- **The `<parameters>` YAML cannot reach the gain:**
  - The plugin calls `rclcpp::init` with no arguments (L351–356).
  - It then creates its `gz_ros2_control` node (L358–359).
  - Only afterwards does it install the YAML files as the context's global arguments (L391–398).
  - A node's parameter overrides are fixed when the node is created. `declare_parameter` therefore returns the default.
  - The controller manager and the controllers are created later, so the YAML does configure them.
- **Offline probe** with the installed rclcpp (scratch build, not part of the project; no simulator), reproducing the plugin's order:
  - result: plugin node `0.1`; a node created after the arguments were installed `1`;
  - the YAML gave both nodes `1.0`.
- **The `humble` branch head** (0.7.21, `392112557b08…`) has the same order and default.

**Consequence:** in this installation the gain is fixed at 0.1. §8's ranking stands: the cause is in the control path, and no measurement defect exists. Its implementation path is withdrawn, and the remaining options are left to the design note.

Probe, for reproduction (built against the installed `rclcpp` with CMake `find_package(rclcpp)`):

```cpp
// argv[1]: YAML giving position_proportional_gain 1.0 to /gz_ros2_control and to
// /gz_ros2_control_created_after_args
#include <rcl/arguments.h>
#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <string>
#include <vector>
int main(int, char ** argv0)
{
  std::vector<std::string> arguments = {"--ros-args", "--params-file", argv0[1]};
  if (!rclcpp::ok()) {std::vector<const char *> a; rclcpp::init(static_cast<int>(a.size()), a.data());}
  auto plugin_node = rclcpp::Node::make_shared("gz_ros2_control", "/");     // plugin L358-359
  std::vector<const char *> argv;
  for (const auto & s : arguments) {argv.push_back(s.data());}
  rcl_arguments_t rcl_args = rcl_get_zero_initialized_arguments();           // plugin L391-398
  (void)rcl_parse_arguments(static_cast<int>(argv.size()), argv.data(), rcl_get_default_allocator(), &rcl_args);
  plugin_node->get_node_base_interface()->get_context()->get_rcl_context()->global_arguments = rcl_args;
  double g = plugin_node->declare_parameter<double>("position_proportional_gain", 0.1);  // gz_system L208
  auto later = rclcpp::Node::make_shared("gz_ros2_control_created_after_args", "/");
  double g_later = later->declare_parameter<double>("position_proportional_gain", 0.1);
  std::printf("plugin node: %g\nnode created after args: %g\n", g, g_later);       // 0.1 and 1
  rclcpp::shutdown();
}
```
