# SpiderX Hardware Interface

Everything in this document that is marked **PLACEHOLDER** or **TEMPLATE** is a design interface,
not working hardware support. This repository contains no microcontroller firmware and no servo
driver.

## 1. Hardware entry point

```bash
ros2 launch spiderx_bringup real_robot.launch.py serial_port:=/dev/ttyUSB0
```

| Starts today | Source |
|---|---|
| `robot_state_publisher` (`sim_backend:=none`, `use_sim_time:=false`) | `spiderx_bringup/launch/real_robot.launch.py` |
| RPLidar A1 driver → `/scan` in `lidar_link` | `spiderx_firmware/launch/lidar.launch.py` → `rplidar_ros/rplidar_a1_launch.py` |

**Rule:** hardware launch files live only under `spiderx_firmware/` and `real_robot.launch.py`. The
simulation launch files must never include them. `scripts/validate_fortress.sh` checks this both
statically and at runtime.

## 2. Lidar (configured)

| Item | Value |
|---|---|
| Sensor | Slamtec RPLidar A1 (driver `ros-humble-rplidar-ros`, launch `rplidar_a1_launch.py`) |
| Topic / frame | `/scan` / `lidar_link`, the same as the simulation |
| Serial | `serial_port` argument, default `/dev/ttyUSB0`. Check with `ls /dev/ttyUSB*` |
| Permissions | The user must be in the `dialout` group: `sudo usermod -aG dialout $USER`, then log out and back in |
| Mount pose | `lidar_link` in the URDF (0.051, −0.045, 0.141, yaw +90°) is a **simulation placeholder**. Measure the real mount and update the URDF |

## 3. Actuators (TEMPLATE)

Template: `src/spiderx_firmware/config/actuator_interface_template.yaml`.

Before any servo code is written:

1. **Identify the hardware:** servo model, controller board and bus. Record torque and speed from the datasheet; they replace the URDF placeholders of 100 N·m and 100 rad/s.
2. **Map joints to servos:** for all 12 joints, record `servo_id`, `direction_sign` (measured) and `zero_offset_rad` (the servo reading at the CAD zero pose).
3. **Implement a `ros2_control` `hardware_interface::SystemInterface`:**
   - `position` command interface; `position` and `velocity` state interfaces;
   - joint names from `spiderx_controller/config/spiderx_legs.yaml`;
   - clamp every command to the soft limits in that file.
4. **Load the scaffold:** `spiderx_controller/config/spiderx_ros2_controllers.yaml`, a joint state broadcaster plus a joint trajectory controller.

Safety requirements for the first powered test:

- **Robot lifted:** feet off the ground, on a stand.
- **Supply current limit:** set on the power supply.
- **E-stop:** a physical emergency stop within reach.
- **Speed limit:** move one joint at a time at no more than `max_joint_velocity_rad_s` (0.5 rad/s, placeholder).

## 4. IMU (TEMPLATE)

Template: `src/spiderx_firmware/config/imu_template.yaml`.

- **URDF frame:** the URDF has **no** `imu_link` yet. Add one only from the measured mounting pose.
- **Interface:** the EKF expects `sensor_msgs/Imu` on `/imu/data` (`spiderx_localization/config/ekf.yaml`).

## 5. Embedded computer (PLACEHOLDER)

| Item | Requirement |
|---|---|
| OS | Ubuntu 22.04 + ROS 2 Humble, the same as the development PC |
| Network | The same `ROS_DOMAIN_ID` on the robot and the PC. Use a wired or reliable Wi-Fi link for RViz |
| Devices | Lidar, servo controller and IMU on stable device names (udev rules) |
| Time | NTP/chrony sync between the robot and the PC (TF timestamps) |

The board model and power budget are not defined in this repository.
