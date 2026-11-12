# Gazebo server SIGABRT at shutdown: read-only review

**Scope.** The review used:
- the existing launch logs and backtraces;
- the exact plugin source (overlay `bfc5845` + patch `14238400…`);
- the installed rclcpp 16.0.13 headers.

Nothing was launched, fixed or suppressed.

## Proven

1. **After the bag closed, and after the controllers shut down.**
   - The recorder logged `Recording stopped` at 1793342928.578 (realtime). That was 2.2 s before the Gazebo process handled SIGINT (`signal_handler` at 1793342930.753). The M5.5 node had already been stopped (`run/steps.log` 06:48:49).
   - In `run/launch.log.gz` lines 470–519, the controller manager then logged, in order:
     - deactivation and shutdown of `joint_state_broadcaster` and `leg_trajectory_controller`;
     - successful `deactivate` and `shutdown` of the hardware `SpiderXGazeboSimSystem`;
     - "Shutting down the controller manager.";
     - only then, the terminate.
   - No ERROR or FATAL line precedes the shutdown.
2. **The failing operation.** The backtrace shows:
   - frame #10, `gz_ros2_control::GazeboSimROS2ControlPlugin::~GazeboSimROS2ControlPlugin()`;
   - frame #9, `rclcpp::Executor::remove_node(std::shared_ptr<rclcpp::Node>, bool)`;
   - `std::runtime_error` "Node needs to be associated with an executor.", escaping the destructor;
   - `__cxa_call_terminate`, then `abort` (SIGABRT).
   
   In the destructor's code (lines 246–255), the first executor call is `executor_->remove_node(controller_manager_)`, followed by `cancel()` and `join()`. rclcpp 16.0.13 (`executor.hpp`) documents that `remove_node` throws `std::runtime_error` "if the node is not associated with an executor". So when the destructor ran, the controller-manager node was no longer associated with any executor.
3. **Not caused by our patch.**
   - The destructor is byte-identical to upstream `bfc5845`, and so is every `executor_`, `add_node` and `remove_node` line.
   - The patch hunks touch only the include, the `Configure()` parameter handling and the plugin-node creation.
   - The 2026-10-29 S2 run used the **stock** `/opt/mm/.../libgz_ros2_control-system.so`, with no overlay. It aborted the same way: "Shutting down the controller manager.", then the same `what()`, then frames #10 and #9 (`~/spiderx_evidence/s2_run_20261029T100210Z/run/launch.log` lines 305–343).

## Version-matched source review (2026-10-30)

Sources, read only:
- `ros2/rclcpp` tag 16.0.13;
- `ros-controls/ros2_control` tag 2.51.0;
- `ros-controls/gz_ros2_control` tags 0.7.17 and 0.7.21.

These match the installed rclcpp 16.0.13, controller_manager 2.51.0 and gz_ros2_control 0.7.15.

**Proven, by code matched to the log and backtrace:**
1. **The controller manager removes itself first.** On context shutdown (the SIGINT), controller_manager 2.51.0 runs its pre-shutdown callback (`controller_manager.cpp` lines 584–601):
   - it logs "Shutdown request received....";
   - if still associated, it **removes its own node from the executor**;
   - it cancels the executor, shuts down the controllers and the hardware components, and logs "Shutting down the controller manager.".
   
   This matches the logged sequence exactly.
2. **The plugin destructor then removes it again.** Gazebo next destroys the system plugins. The 0.7.15 destructor calls `executor_->remove_node(controller_manager_)` unconditionally. rclcpp 16.0.13 `Executor::remove_node` (`executor.cpp` lines 344–345) throws `std::runtime_error("Node needs to be associated with an executor.")` because the node's association flag is already false. The exception leaves the destructor, so `std::terminate` aborts the server (frames #10 and #9).
3. **It is a teardown double removal, not incomplete shutdown.** The controllers and the hardware were shut down by the callback in step 1, before the throw.
4. **Fixed upstream in gz_ros2_control 0.7.17** (2025-09-29): "Don't remove the node at destruction" (#683, backported in #686).
   - The 0.7.17 destructor deletes the `if (!controller_manager_) return;` guard and the `remove_node` call; it keeps `executor_->cancel()` and `thread_executor_spin_.join()`.
   - Between 0.7.15 and 0.7.17, the plugin source otherwise differs by one cast-style line.

**Smallest candidate correction (not implemented).** Backport the upstream #686 destructor change as one more hunk of the pinned overlay patch, and re-verify:
- the overlay gtest;
- the patch and library provenance;
- one launch-only run (as in Phase B) showing a clean server exit.

A local variant could delete only the `remove_node` line and keep the null guard. That keeps `cancel()` away from a null executor when `Configure()` failed early, but it departs from upstream. Moving the pin to 0.7.17 or later is the larger alternative: the parameter-loading patch would have to be re-applied and re-verified.

## Supported, not proven

- **It is a teardown failure only.** The abort follows a logged, complete shutdown of the controllers and the hardware. The run's data were already closed, and nothing in the run shows a control-time symptom: no controller error, and command ownership was clean (P4).
- **The candidate correction removes the abort.** This is supported by the code, but untested here: after the upstream change, the destructor runs only `cancel()` and `join()`, which do not throw in this path.

## Unresolved

- Why the two launches that sent no goals ended differently. Phase B and the 2026-10-29 disabled check hit a SIGKILL escalation by the ign launcher, not this abort. In all four available logs, the runs that sent goals aborted and the goal-free runs were killed. That is a correlation, not a cause.

## Recommendation (smallest next action)

1. **Decide on the backport.** Approve, or decline, backporting upstream #686 into the overlay patch as a separate, tested change before the next launch.
2. **Until then:**
   - record the abort as a known upstream 0.7.15 teardown defect, present with both the stock and the overlay library;
   - keep each run's leftover check and verified restore, which caught it here.
3. **Any code change is separate.** A destructor guard, for example, would be a separate, approved revision of the overlay patch, with its own test. It is not needed to interpret S2 data, which are complete before the abort.
