# SpiderX Gazebo Fortress Test Guide

By the end of this guide, you will have:

- cloned or switched to the tested branch;
- installed the required packages;
- built SpiderX;
- started Gazebo Fortress;
- confirmed the robot appears;
- verified the simulated lidar data.

| Item | Value |
|---|---|
| Repository | `https://github.com/robovision2210/spiderx_ws` |
| Branch to test | `claude/stoic-shannon-ur2mes` |
| Draft pull request | https://github.com/robovision2210/spiderx_ws/pull/4 (do **not** merge until your local test passes) |
| Workspace folder | `~/spiderx_ws` |
| Launch command | `ros2 launch spiderx_bringup fortress.launch.py` |

---

## Quick Start (for experienced users)

If you already know ROS 2, Git and colcon, this block is the whole test. Everyone else should
skip it and follow the numbered steps from Section 4.

```bash
# Terminal 1
cd ~ && [ -d spiderx_ws ] || git clone https://github.com/robovision2210/spiderx_ws.git
cd ~/spiderx_ws && git fetch origin && git checkout claude/stoic-shannon-ur2mes && git pull
sudo apt update && sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys gazebo_ros2_control
rm -rf build install log && colcon build --symlink-install && source install/setup.bash
ros2 launch spiderx_bringup fortress.launch.py

# Terminal 2
source /opt/ros/humble/setup.bash && source ~/spiderx_ws/install/setup.bash
ros2 topic list && ros2 topic echo /scan --once
cd ~/spiderx_ws && ./scripts/validate_fortress.sh
```

---

## 1. What Claude Changed

### 1.1 First, the existing robot model was inspected

SpiderX's robot description was exported from Fusion 360 CAD. It is a set of files that
describe every part of the robot and how the parts are connected.

- The main file is `src/spiderx_description/urdf/spiderx.urdf.xacro`.
- **URDF** is ROS's robot-description format.
- **xacro** is URDF with macros, so a file can be reused with different options.

Before anything was changed, the whole model was audited. The full report is in
`docs/SPIDERX_URDF_AUDIT.md`. Here is what it found, in plain English:

| Finding | Meaning |
|---|---|
| **31 links** | A *link* is one rigid part: body, servo, thigh, foot, top plate and so on. |
| **30 joints** | A *joint* connects two links. 12 are *revolute* (they can rotate: 3 per leg, namely hip, thigh and foot/knee). 18 are *fixed* (glued together). |
| **One connected root tree** | Every part connects back to a single starting link, `dummy_link`. No part is floating or disconnected. |
| **CAD mesh paths** | Each link's 3D shape is an STL file in `src/spiderx_description/meshes/`. All 30 files exist and load correctly. |
| **Mass and inertia preserved** | *Inertia* describes how hard a part is to rotate. Every existing value was checked against the 3D shapes, matched, and was **left unchanged**. |
| **Fusion steel-density assumption** | Fusion 360 used its default material, steel, for every part, which makes the robot weigh about 7.3 kg. The real robot is probably much lighter. This was documented but **not changed**, because changing it needs real measurements. |
| **Servo horn inertia placeholders** | 8 small servo-horn parts have hand-entered inertia values of `1e-6` instead of true CAD values. They are safe for simulation, so they were kept and labelled as placeholders. |
| **+Y-forward body convention** | In the CAD frame, the robot's front is the **+Y** direction, not +X as the ROS convention (REP-103) expects. This is documented and left unchanged. |

### 1.2 Then these things were repaired or added

| Problem or need | What was done |
|---|---|
| **`main` branch could not build.** `CMakeLists.txt` tried to install a `config/` folder that did not exist. | Added `src/spiderx_description/config/controllers.yaml`, the legacy file that `spiderx.gazebo` already expected. The build now succeeds. |
| No working Gazebo Fortress launch | Added launch files that start Gazebo Fortress, load the robot from the xacro, and spawn it. |
| No simulation world | Added a world with a ground plane, sunlight, physics, four walls and three obstacles. |
| No lidar | Added `lidar_link` on top of the robot and a simulated 2D lidar sensor. |
| No lidar data in ROS 2 | Added a ROS–Gazebo bridge that turns Gazebo's lidar data into the ROS 2 topic `/scan`. |
| Package install rules | The packages now install the `worlds/`, `config/`, `launch/` and `rviz/` folders. |
| No quick way to check everything | Added the validation script `scripts/validate_fortress.sh`. |

### 1.3 Important files and what each one does

| File | What it does |
|---|---|
| `src/spiderx_bringup/launch/fortress.launch.py` | **The one command you run.** Starts the simulation, and RViz if you ask for it. |
| `src/spiderx_description/launch/fortress.launch.py` | The actual simulation launch: Gazebo, robot publisher, robot spawn and bridge. |
| `src/spiderx_description/worlds/spiderx_fortress.sdf` | The Gazebo world: ground, light, walls and obstacles. |
| `src/spiderx_description/urdf/spiderx.urdf.xacro` | The robot description. Changes: a `sim_backend` option, plus `lidar_link`. |
| `src/spiderx_description/urdf/spiderx_fortress.gazebo.xacro` | Fortress-only extras: the lidar sensor and a joint-state publisher. |
| `src/spiderx_description/config/fortress_bridge.yaml` | Tells the bridge which Gazebo topics become ROS topics: `/clock`, `/scan`, `/joint_states`. |
| `src/spiderx_description/config/controllers.yaml` | Legacy Gazebo Classic controller settings. It fixes the build and is not used by Fortress. |
| `src/spiderx_bringup/rviz/spiderx_fortress.rviz` | RViz view settings: robot, TF and laser scan. |
| `src/spiderx_bringup/launch/real_robot.launch.py` | **Real robot only.** Starts the physical RPLidar driver (via `spiderx_firmware`). Never run it for simulation. |
| `scripts/validate_fortress.sh` | Automatic checks of the files and packages. |
| `docs/SPIDERX_URDF_AUDIT.md`, `docs/URDF_INERTIA_AUDIT.md` | Detailed engineering audit reports. |
| `README.md`, `docs/MIGRATION_FORTRESS.md` | Project overview and migration notes. |

---

## 2. What Will Work

Each item below was **actually tested** in the cloud with Gazebo Fortress 6.16 and ROS 2 Humble.

- ✅ **Build the ROS packages** `spiderx_description` and `spiderx_bringup` with `colcon`.
- ✅ **Start Gazebo Fortress** with the SpiderX test world.
- ✅ **Spawn the SpiderX CAD model**, with its real 3D meshes, into the world.
- ✅ **Ground plane, collisions and physics.** The robot drops onto the ground and stays stable.
- ✅ **`robot_state_publisher` and TF.** *TF* is ROS's system for tracking where every part is.
- ✅ **`/clock`.** Simulation time is published to ROS 2.
- ✅ **`/joint_states`.** All 12 joint angles are measured from the simulation.
- ✅ **Simulated lidar.** It gives 360 readings per turn, one per degree, from 0.15 m to 12 m, 10 times per second.
- ✅ **ROS 2 `/scan`.** The lidar data arrives in ROS, in the `lidar_link` frame. Distances to the walls were checked and are correct within about 1 cm.
- ✅ **Optional RViz view.** It shows the robot, TF and laser points.

---

## 3. What Will Not Work Yet

> ### ⛔ The simulated SpiderX is a PASSIVE model. It will NOT stand, walk or navigate.
>
> - ❌ **No standing posture controller.** Nothing holds the legs in a standing pose.
> - ❌ **No gait controller.** Nothing produces a walking leg pattern.
> - ❌ **No inverse kinematics (IK) walking.** Nothing calculates the joint angles needed to put a foot at a chosen spot.
> - ❌ **No `/cmd_vel` → leg-joint conversion.** Sending velocity commands does nothing.
> - ❌ **No `/odom`.** Nothing estimates how far the robot has moved.
> - ❌ **Do not start Nav2 or SLAM expecting motion.** They need `/odom` and a robot that can move.
> - ⚠️ **The legs will fold and the body will settle under gravity.** No motor is holding the joints, so this is **correct behaviour** for a passive physics model, not a bug.

**Why these are separate milestones.** A walking robot needs several layers, each built and
tested on top of the one before:

1. motors that hold commanded angles;
2. a stable standing pose;
3. the leg geometry maths (kinematics);
4. a gait pattern;
5. a way to turn velocity commands into steps;
6. a way to measure motion (odometry).

Navigation sits on top of all of these. This test proves the **foundation**: the model, the
world, the sensors and the ROS connections.

---

## 4. Before You Start

Tick each item before continuing:

- [ ] Your computer runs **Ubuntu 22.04**.
- [ ] **ROS 2 Humble** is installed in `/opt/ros/humble`.
- [ ] You have a working **internet connection**.
- [ ] You have **at least 10 GB of free disk space**. To check, run `df -h ~` and look at the `Avail` column.
- [ ] A **terminal** is open. Press **Ctrl + Alt + T** to open one.
- [ ] **No other Gazebo** window or simulation is running.
- [ ] **No Software Updater, App Center or other `apt` install** is running. Only one program can use the package manager at a time.
- [ ] You know the repository will live in **`~/spiderx_ws`**. `~` means your home folder, for example `/home/yourname`.

### Terminal basics

- **Opening a terminal:** press **Ctrl + Alt + T**.
- **Opening a second terminal:** press **Ctrl + Shift + T** inside a terminal to get a new tab, or press **Ctrl + Alt + T** again for a new window. This guide calls them **Terminal 1** and **Terminal 2**.
- **Pasting a command:** press **Ctrl + Shift + V**. Plain Ctrl + V does not work in the terminal.
- **Running a command:** press **Enter** after pasting it.
- **Stopping a running program:** press **Ctrl + C**. Use it **only when this guide tells you to**, for example to stop the simulation at the end.

> ⚠️ **Safety warning: never delete apt lock files and never kill automatic updater processes.**
>
> If you see a message about a *lock* (`/var/lib/dpkg/lock-frontend`) or `unattended-upgr`:
>
> - do **not** run `sudo rm` on any lock file;
> - do **not** run `kill` on the updater.
>
> Doing so can corrupt your system's package database. Section 5, Step 3, explains the safe fix.

---

## 5. Step-by-Step Setup

All of Section 5 happens in **Terminal 1**.

### Step 1 — Check ROS 2 Humble

**1a. Check the Ubuntu version.**

- **What it does:** prints your Ubuntu release.
- **Why:** this guide and ROS 2 Humble need Ubuntu 22.04.

```bash
lsb_release -rs
```

- **Success looks like:** `22.04`
- **If it fails or shows another number:** this guide is written for 22.04. Other versions are not supported for this test.

**1b. Check that ROS 2 Humble is installed.**

- **What it does:** lists the ROS versions installed on your computer.
- **Why:** everything below needs Humble.

```bash
ls /opt/ros
```

- **Success looks like:** the word `humble` appears.
- **If it fails** (`No such file or directory`, or no `humble`): install ROS 2 Humble first, following the official guide at https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html (the "desktop" install). Then come back here.

**1c. Load ROS into this terminal and confirm it works.**

- **What it does:** "sources" ROS, meaning it loads the ROS commands into this terminal only. It then prints the ROS version name.
- **Why:** a terminal cannot use `ros2` until ROS has been sourced in it.

```bash
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO
```

- **Success looks like:** `humble`
- **If it fails:** re-check Step 1b.

---

### Step 2 — Get the SpiderX branch

A **Git branch** is a separate line of changes in a repository. The tested simulation work
lives on the branch `claude/stoic-shannon-ur2mes`.

**First, find out which situation you are in.**

- **What it does:** checks whether `~/spiderx_ws` already exists and is a Git repository.
- **Why:** situations A and B below need different commands.

```bash
[ -d ~/spiderx_ws/.git ] && echo "Situation A: repo exists" || echo "Situation B: need to clone"
```

- **Success looks like:** one of the two sentences, then follow the matching part below.
- **If it prints Situation B but a `~/spiderx_ws` folder exists anyway:** that folder is not this Git repository. Rename it first with `mv ~/spiderx_ws ~/spiderx_ws_old`, then follow Situation B.

#### Situation A — the repository already exists at `~/spiderx_ws`

**A1. Go into the repository and check for unsaved local changes.**

- **What it does:** moves into the folder and shows the Git state.
- **Why:** Git will refuse to switch branches if you have edited files that are not saved in a commit.

```bash
cd ~/spiderx_ws
git status
```

- **Success looks like:** `nothing to commit, working tree clean`
- **If it lists modified files:** save them safely with the command below, which stores them aside and does not delete them:

  ```bash
  git stash
  ```

  You can bring them back later with `git stash pop`.

**A2. Download the latest branches from GitHub.**

- **What it does:** fetches new branches and commits. It does not change your files yet.
- **Why:** your copy may not know about the Claude branch yet.

```bash
git fetch origin
```

- **Success looks like:** either a few lines such as `* [new branch] claude/stoic-shannon-ur2mes -> origin/claude/stoic-shannon-ur2mes`, or no output at all (already up to date).
- **If it fails** (`Could not resolve host`): check your internet connection and retry.

**A3. Switch to the tested branch.**

- **What it does:** changes your files to match the tested branch.
- **Why:** the Fortress simulation only exists on this branch until the PR is merged.

```bash
git checkout claude/stoic-shannon-ur2mes
git pull
```

- **Success looks like:** `Switched to branch 'claude/stoic-shannon-ur2mes'` (or `Already on ...`), and then `Already up to date.` or a list of updated files.
- **If it fails** with `pathspec ... did not match`: run `git fetch origin` again, then retry. If it fails with `Your local changes would be overwritten`: go back to A1 and run `git stash`.

Now skip to **A/B4** below.

#### Situation B — the repository does not exist yet

**B1. Clone (download) the repository into your home folder.**

- **What it does:** copies the whole project from GitHub into `~/spiderx_ws`.
- **Why:** you need the code on your computer.

```bash
cd ~
git clone https://github.com/robovision2210/spiderx_ws.git
```

- **Success looks like:** `Cloning into 'spiderx_ws'...`, then some progress lines ending in `done.`
- **If it fails:**
  - `git: command not found`: run `sudo apt install git` and retry.
  - `already exists and is not an empty directory`: see the note at the start of Step 2.

**B2. Enter the folder and switch to the tested branch.**

- **What it does:** moves into the repository and checks out the Claude branch.
- **Why:** a fresh clone starts on `main`, which does not contain the Fortress work.

```bash
cd ~/spiderx_ws
git checkout claude/stoic-shannon-ur2mes
```

- **Success looks like:** `branch 'claude/stoic-shannon-ur2mes' set up to track 'origin/claude/stoic-shannon-ur2mes'.` and `Switched to a new branch 'claude/stoic-shannon-ur2mes'`.
- **If it fails:** run `git fetch origin` and retry.

#### A/B4. Confirm you are on the correct branch (both situations)

- **What it does:** shows the current branch and whether your files are clean.
- **Why:** testing the wrong branch is the most common mistake.

```bash
cd ~/spiderx_ws
git status
```

**Success looks exactly like this:**

```
On branch claude/stoic-shannon-ur2mes
Your branch is up to date with 'origin/claude/stoic-shannon-ur2mes'.

nothing to commit, working tree clean
```

**Extra check:** confirm that the key files exist.

```bash
ls src
ls scripts
```

- **Success looks like:** `src` shows `spiderx_bringup  spiderx_description`, and `scripts` shows `validate_fortress.sh`.
- **If you also see other packages in `src`** (for example `rplidar_ros-...` or `spiderx_navigation`): you are on a different branch. Repeat A3.

---

### Step 3 — Install the Gazebo Fortress dependencies

**3a. Refresh the package list.**

- **What it does:** downloads the latest list of available Ubuntu and ROS packages.
- **Why:** `apt` must know the newest versions before installing.

```bash
sudo apt update
```

**About the password prompt.** `sudo` asks for **your login password**. While you type it,
**nothing appears on screen**: no dots and no stars. This is normal. Type it and press
**Enter**.

- **Success looks like:** many lines ending in something like `Reading package lists... Done`. A final line such as `N packages can be upgraded` is fine.
- **If it fails:** see the lock message box below, or check your internet connection.

**3b. Install the ROS ↔ Gazebo Fortress packages.**

- **What it does:** installs the three packages this simulation uses. Gazebo Fortress itself is installed automatically as a dependency.
  - `ros_gz_sim` starts Gazebo and spawns robots.
  - `ros_gz_bridge` copies Gazebo topics into ROS 2.
  - `ros_gz_interfaces` provides message definitions.
- **Why:** without them, the launch file cannot start Gazebo or produce `/scan`.

```bash
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces
```

- When asked `Do you want to continue? [Y/n]`, type `Y` and press **Enter**.
- **Success looks like:** the download and install finish with no line starting with `E:`. Running it a second time says `ros-humble-ros-gz-sim is already the newest version`.
- **If it fails** with `Unable to locate package ros-humble-ros-gz-sim`: the ROS 2 apt source is missing. Re-check your ROS installation (Step 1b) and run `sudo apt update` again.

> ### 🔒 If you see a "lock" message
>
> Examples:
>
> - `Could not get lock /var/lib/dpkg/lock-frontend. It is held by process 1234 (unattended-upgr)`
> - `Waiting for cache lock`
>
> This means Ubuntu's automatic updater, or another installer, is busy right now.
>
> 1. **Wait.** It usually finishes within 5–10 minutes. Leave the terminal alone.
> 2. **Do NOT delete lock files** (no `sudo rm /var/lib/dpkg/lock...`).
> 3. **Do NOT kill the process** (no `kill`, `pkill` or `killall` on `unattended-upgr`, `apt` or `dpkg`).
> 4. If the message is **still there after 15 minutes**: close all terminals and **restart the computer normally** (menu → Power Off / Log Out → Restart). After logging in, **wait 2 minutes**, open a terminal, and run Step 3a again.

**3c. Make sure the build and dependency tools are installed.**

- **What it does:** installs `colcon` (the ROS build tool) and `rosdep` (the dependency installer). If you already have them, nothing changes.
- **Why:** Steps 4 and 5 use both.

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep
```

- **Success looks like:** it finishes without `E:` lines, or says `already the newest version`.

---

### Step 4 — Install missing ROS dependencies with rosdep

**rosdep** reads the `package.xml` file of every package in `src/`. It finds the ROS and
Ubuntu packages they need and installs whatever is missing, so you do not have to find them
one by one.

**4a. Initialise rosdep (only needed once per computer).**

- **What it does:** creates rosdep's configuration, then downloads its database.
- **Why:** rosdep cannot work without them.

```bash
sudo rosdep init
rosdep update
```

- **Success looks like:** `rosdep update` ends with `updated cache in /home/.../.ros/rosdep/sources.cache`.
- **If `sudo rosdep init` says `default sources list file already exists`:** that is fine. It was already initialised. Just run `rosdep update`.
- **Do not** run `rosdep update` with `sudo`.

**4b. Install this workspace's dependencies.**

- **What it does:** installs everything SpiderX's two packages need, for example `xacro`, `robot_state_publisher`, `rviz2` and `rplidar_ros`.
- **Why:** the build and launch need them.

```bash
cd ~/spiderx_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys gazebo_ros2_control
```

**Why `--skip-keys gazebo_ros2_control`?** That package belongs to the old **Gazebo Classic**
setup, which is kept only as legacy reference. Installing it would pull in the whole old
Gazebo Classic simulator, which you do not need for this Fortress test. Skipping it saves
time and disk space.

- **Success looks like:** `#All required rosdeps installed successfully`
- **If it fails** with `Cannot locate rosdep definition for [...]`: copy that line and send it for help. The `-r` option already continues past individual failures, so also check whether the build (Step 5) works anyway.

---

### Step 5 — Clean and build the workspace

**5a. Source ROS 2 Humble.**

- **What it does:** loads ROS into this terminal.
- **Why:** `colcon` needs to find ROS's build tools and packages. **Sourcing** runs a setup script that tells this terminal where ROS is installed. It only affects the terminal you run it in.

```bash
source /opt/ros/humble/setup.bash
```

- **Success looks like:** no output (silence means success).

**5b. Remove old build results.**

- **What it does:** deletes the `build`, `install` and `log` folders, which are generated output and never your source code.
- **Why:** after switching branches, old build output can refer to files or packages that no longer exist and cause confusing errors. Rebuilding from clean avoids that.

```bash
cd ~/spiderx_ws
rm -rf build install log
```

> ⚠️ Run this **only inside `~/spiderx_ws`**. The `cd` line makes sure of that. Type the
> folder names exactly as shown.

- **Success looks like:** no output.

**5c. Build.**

- **What it does:** builds the two SpiderX packages. `--symlink-install` links files instead of copying them, so later edits to launch or world files take effect without rebuilding.
- **Why:** ROS can only launch packages that have been built and installed.

```bash
colcon build --symlink-install
```

**Success looks like** (the times will differ):

```
Starting >>> spiderx_description
Finished <<< spiderx_description [1.1s]
Starting >>> spiderx_bringup
Finished <<< spiderx_bringup [0.8s]

Summary: 2 packages finished [2.0s]
```

- A line like `1 package had stderr output: spiderx_description`, caused by a CMake *deprecation warning*, is harmless **as long as** the summary says `2 packages finished`.
- **Failure looks like:** `Failed <<< ...` or `Summary: 1 package failed`.
- **If it fails, copy for help:**
  1. everything from the first `---` line to the end of the output;
  2. the output of this command:

     ```bash
     cat ~/spiderx_ws/log/latest_build/*/stderr.log
     ```

---

### Step 6 — Source the built workspace

- **What it does:** loads **your** newly built SpiderX packages into the terminal, on top of ROS.
- **Why:** without this, ROS does not know `spiderx_bringup` exists.

> 📌 **Rule:** every time you open a **new** terminal, run **both** of these lines before any
> `ros2` command. Each terminal starts with no knowledge of ROS or of your workspace.

```bash
source /opt/ros/humble/setup.bash
source ~/spiderx_ws/install/setup.bash
```

- **Success looks like:** no output.

**Verify that ROS can find the packages.**

- **What it does:** prints where each package is installed.
- **Why:** it proves both the workspace and Gazebo's ROS packages are visible.

```bash
ros2 pkg prefix spiderx_bringup
ros2 pkg prefix ros_gz_sim
```

**Success looks like** (with your user name):

```
/home/yourname/spiderx_ws/install/spiderx_bringup
/opt/ros/humble
```

- **If `Package not found`:** for `spiderx_bringup`, redo Step 5 and then Step 6. For `ros_gz_sim`, redo Step 3b.

---

## 6. Launch SpiderX Fortress

**Gazebo Fortress** is a 3D robot simulator, also called Ignition Gazebo version 6. It
simulates physics (gravity, collisions) and sensors (such as the lidar) and shows everything
in a 3D window. It is the newer replacement for the old "Gazebo Classic".

### Terminal 1 — start the simulation

Make sure Terminal 1 has been sourced (Step 6). Then:

```bash
ros2 launch spiderx_bringup fortress.launch.py
```

**What this starts:**

| Process | Job |
|---|---|
| `ign gazebo` | The Gazebo Fortress simulator and its window |
| `robot_state_publisher` | Reads the robot description and publishes TF (the positions of all parts) |
| `create` (`spawn_spiderx`) | Puts the SpiderX model into the world, then exits. Exiting is normal |
| `parameter_bridge` (`spiderx_gz_bridge`) | Copies `/clock`, `/scan` and `/joint_states` from Gazebo into ROS 2 |

**Expected terminal messages** (among many others):

```
[robot_state_publisher-2] ... got segment lidar_link
[create-3] ... [spawn_spiderx]: OK creation of entity.
[INFO] [create-3]: process has finished cleanly
[parameter_bridge-4] ... Creating GZ->ROS Bridge: [/clock (gz.msgs.Clock) -> /clock (rosgraph_msgs/msg/Clock)]
[parameter_bridge-4] ... Creating GZ->ROS Bridge: [/spiderx/scan (gz.msgs.LaserScan) -> /scan (sensor_msgs/msg/LaserScan)]
[parameter_bridge-4] ... Creating GZ->ROS Bridge: [/spiderx/joint_states (gz.msgs.Model) -> /joint_states (sensor_msgs/msg/JointState)]
```

Yellow `[Wrn]` or `QML` lines from `ign gazebo` are common and usually harmless.

**Expected Gazebo window, after 10–60 seconds (the first start can be slower):**

- a light grey floor with a grid;
- four grey walls forming a square room;
- an orange box, a blue box and a green pillar (cylinder);
- the **SpiderX quadruped** in the centre, with a small **dark cylinder** on top (the lidar);
- the camera starts close to the robot;
- the legs **fold slightly as the body settles**. This is expected (Section 3);
- the play button at the bottom left shows the simulation is running.

**If no window appears after 2 minutes:**

1. Look in Terminal 1 for red `[ERROR]` lines or `process has died`.
2. Go to Section 8 (Troubleshooting), rows "Gazebo window does not open" and "black screen".

**How to save the terminal output for help.** Stop the simulation with **Ctrl + C** in
Terminal 1, then run the launch again and save everything to a file:

```bash
ros2 launch spiderx_bringup fortress.launch.py 2>&1 | tee ~/spiderx_launch.log
```

The file `~/spiderx_launch.log` can then be shared. `tee` shows the output **and** saves it.

### Optional — with RViz

**RViz** is ROS's visualisation tool. It shows what ROS "sees": the robot model, TF frames and
the laser points. To use it, stop the first launch with **Ctrl + C**, then run:

```bash
ros2 launch spiderx_bringup fortress.launch.py rviz:=true
```

Expected result: a second window (RViz) with **Global Status: Ok**, the robot model, and red
laser points outlining the walls, boxes and pillar.

### Stopping the simulation

When you are finished (**and only then**), click in Terminal 1 and press **Ctrl + C**. Wait
until the terminal prompt returns. Closing the Gazebo window also stops everything.

---

## 7. Verify the Simulation

Leave the simulation running in Terminal 1. Open **Terminal 2** with **Ctrl + Shift + T**.

> 📌 **Terminal 2 must be sourced first.** Paste this before anything else:
>
> ```bash
> source /opt/ros/humble/setup.bash
> source ~/spiderx_ws/install/setup.bash
> ```

### Check 1 — list all ROS topics (Terminal 2)

A **topic** is a named channel that ROS programs publish data on.

```bash
ros2 topic list
```

**Success looks like** (the order can differ, and extra lines are fine):

```
/clock
/joint_states
/parameter_events
/robot_description
/rosout
/scan
/tf
/tf_static
```

### Check 2 — confirm the five important topics (Terminal 2)

| Topic | What it proves |
|---|---|
| `/clock` | Simulation time is flowing into ROS |
| `/joint_states` | The 12 joint angles come from Gazebo physics |
| `/scan` | The simulated lidar reaches ROS |
| `/tf` | Moving-part positions are published |
| `/tf_static` | Fixed-part positions, including `lidar_link`, are published |

Quick live tests. Each one prints one message and then stops by itself.

```bash
ros2 topic echo /clock --once
ros2 topic echo /joint_states --once --field name
ros2 topic hz /scan
```

- **`/clock`:** `sec:` should be a growing number. Run it twice to see it increase.
- **`/joint_states`:** a list of 12 names, from `lf_hip` to `lr_foot_joint`.
- **`/scan` rate:** shows about `average rate: 10.0` on a normal computer. It can be lower on slow or virtual machines. **This command keeps running.** Stop it with **Ctrl + C** in Terminal 2 once you have seen a few lines.

### Check 3 — read one lidar scan (Terminal 2)

```bash
ros2 topic echo /scan --once
```

### Check 4 — what the LaserScan values mean

| Field | Expected value | Meaning |
|---|---|---|
| `header.frame_id` | `lidar_link` | The scan is measured from the lidar on top of the robot |
| `angle_min` | `-3.1415...` | The first reading points backwards (−180°) |
| `angle_max` | `3.1241...` | The last reading is at +179° |
| `angle_increment` | `0.01745...` | One reading every 1° (0.01745 rad) |
| `range_min` | `0.15` | Anything closer than 15 cm is ignored |
| `range_max` | `12.0` | The maximum distance is 12 m |
| `ranges` | 360 numbers, between about `1.5` and `4.2` | The distance in metres to the nearest object at each angle. The walls are about 3 m away (up to 4.2 m into the corners); the boxes and pillar are about 1.5–1.8 m away. `.inf` would mean nothing was hit |

The output is long and the `ranges` list may be cut short with `...`. That is normal.

Optional check of where the lidar sits on the robot:

```bash
ros2 run tf2_ros tf2_echo base_link lidar_link
```

Expected: `Translation: [0.051, -0.045, 0.141]`. Stop it with **Ctrl + C**.

### Check 5 — run the validation script (Terminal 2)

```bash
cd ~/spiderx_ws && ./scripts/validate_fortress.sh
```

**Success looks like:** many `[PASS]` lines, ending with:

```
All checks passed.
```

### Check 6 — what the validation script checks

| Section | What it checks |
|---|---|
| Package discovery | `spiderx_description`, `spiderx_bringup`, `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_interfaces`, `robot_state_publisher`, `xacro` and the `ign` command are all found |
| xacro and URDF | The robot description builds in all three modes (`classic`, `fortress`, `none`) and passes `check_urdf`; wrong options are rejected |
| Fortress SDF | Gazebo's converter accepts the robot; the lidar sensor, the `lidar_link` frame and the joint-state publisher are present; the world file is valid |
| Mesh URIs | All 30 STL mesh files can be found |
| Launch files | Every launch file has valid Python and its arguments load |
| Classic tokens | No old Gazebo Classic plugins or commands appear in the Fortress files |

The script does **not** start Gazebo. That is what Sections 6 and 7 do.

**If any line says `[FAIL]`:** copy the full script output and send it for help.

### Check 7 — take a screenshot or GIF for GitHub

1. **Gazebo screenshot:** in the Gazebo window, click the **camera icon** in the top-left toolbar. The picture is saved into `~/.ignition/gui/pictures/`.
2. **Whole-screen screenshot:** press **Print Screen** (PrtSc). Ubuntu saves it into `~/Pictures/Screenshots/`.
3. **Short screen recording:** press **Ctrl + Shift + Alt + R** to start and stop a recording, which Ubuntu saves as a video in `~/Videos/`. For a GIF you can install Peek with `sudo apt install peek`.
4. **Suggested shots:** see `media/README.md` for the full list. They include an overview of the world with the robot, a robot close-up, and RViz with laser points.

---

## 8. Troubleshooting

"Output for help" means what to copy and send when asking for help.

| Symptom | Likely cause | Exact safe fix | Output for help |
|---|---|---|---|
| `ros2: command not found` | ROS is not sourced in this terminal | `source /opt/ros/humble/setup.bash` (and then the workspace, see Step 6) | Output of `ls /opt/ros` |
| `Package 'ros_gz_sim' not found` | The Fortress packages are not installed | Redo Step 3b, then `source /opt/ros/humble/setup.bash` | Output of `apt list --installed 2>/dev/null \| grep ros-gz` |
| `Package 'spiderx_bringup' not found` | The workspace is not built or not sourced, or you are on the wrong branch | `cd ~/spiderx_ws && git status` (must show `claude/stoic-shannon-ur2mes`), then redo Steps 5 and 6 | `git status` output and the `colcon build` summary |
| `colcon build` fails | Missing dependency or old build files | Redo Step 4b, then Step 5 (including `rm -rf build install log`) | Build output from `---` to the end, plus `cat ~/spiderx_ws/log/latest_build/*/stderr.log` |
| `Could not get lock ...` / `unattended-upgr` | The automatic updater is busy | **Wait.** Never delete locks and never kill the process. After 15 minutes, restart normally, wait 2 minutes after login, and retry (Step 3 box) | The full lock message |
| Gazebo window does not open | Crash at start, usually graphics | Read Terminal 1 for `[ERROR]` lines. Try `ign gazebo --versions` (should print `6.x.x`). Then try the "black screen" fix below | `~/spiderx_launch.log` (see Section 6) |
| Black Gazebo screen / OpenGL or `ogre2` error | The graphics driver cannot run Gazebo's renderer (common in virtual machines) | Stop with Ctrl + C, then in the same terminal run `export LIBGL_ALWAYS_SOFTWARE=1` and launch again (slower, but works). On a real PC, install the recommended driver: `sudo ubuntu-drivers autoinstall`, then reboot | Launch log, plus the output of `glxinfo -B` (needs `sudo apt install mesa-utils`) |
| Robot is missing | The spawn failed, or you are looking at the wrong spot | In Terminal 1 look for `OK creation of entity`. In Terminal 2 run `ign model --list`: `spiderx` must be listed. In Gazebo, right-click `spiderx` in the entity list and choose *Move to* | `ign model --list` output and the launch log |
| Robot falls or legs fold | **Expected.** A passive model has no motors holding the joints (Section 3) | No fix needed. It should settle within a few seconds and then stay still | Only if it shakes endlessly or flies away: the launch log and a screenshot |
| `/scan` missing from `ros2 topic list` | The bridge is not running, or Terminal 2 is not sourced | Source Terminal 2 (Step 6). Check Terminal 1 for the `Creating GZ->ROS Bridge ... /scan` line. Run `ign topic -l \| grep scan`: `/spiderx/scan` must appear | Outputs of `ros2 topic list` and `ign topic -l` |
| `/scan` exists but has no useful ranges (all `.inf` or no messages) | The lidar needs Gazebo's renderer; graphics problem, or simulation paused | Make sure Gazebo is playing (▶ at the bottom left). Look for `ogre2` errors in Terminal 1. Try the black-screen fix | `ros2 topic echo /scan --once` output and the launch log |
| RViz: `No transform from [...] to [dummy_link]` | TF has not arrived yet, or the simulation is paused | Wait 10 seconds. Press ▶ in Gazebo. Check that `ros2 run tf2_ros tf2_echo dummy_link lidar_link` prints a translation | That `tf2_echo` output and an RViz screenshot |
| Old Gazebo (Classic or Fortress) still open, or warnings `Found additional publishers on /clock` or `Moved backwards in time` | A previous simulation is still running in the background | Close all Gazebo windows. Stop every launch with Ctrl + C. If needed, run `pkill -f "ign gazebo"` (Fortress) and `pkill gzserver; pkill gzclient` (Classic). These only stop simulators, never apt. Then launch again | Output of `ps aux \| grep -E "gazebo\|gzserver"` |

---

## 9. Acceptance Checklist

Copy this list into a note or into the PR comment and tick each item:

```
- [ ] Correct Git branch checked out (claude/stoic-shannon-ur2mes)
- [ ] Dependencies installed
- [ ] Build completed (Summary: 2 packages finished)
- [ ] Gazebo Fortress opened
- [ ] SpiderX visible
- [ ] /clock available
- [ ] /joint_states available
- [ ] /scan available (frame_id: lidar_link)
- [ ] validate_fortress.sh passes (All checks passed.)
- [ ] Screenshot recorded
- [ ] PR remains unmerged until local test is confirmed
```

---

## 10. Next Engineering Roadmap

The correct order for making SpiderX move. Each step depends on the ones before it.

1. **Stable joint controller.** Motors in simulation (via `gz_ros2_control`) that hold commanded joint angles. This needs realistic servo torque and speed limits and joint damping.
2. **Standing-pose controller.** Command all 12 joints into a pose where the robot stands up and stays up.
3. **Leg forward/inverse kinematics.** The maths that converts between joint angles and foot positions, using the per-joint sign conventions in `docs/SPIDERX_URDF_AUDIT.md`.
4. **Gait generator.** A repeating pattern of foot movements (for example a trot or a walk).
5. **`/cmd_vel` → gait interface.** Turn "move forward at 0.1 m/s" commands into gait parameters.
6. **Odometry.** Estimate the robot's movement from the legs and/or an IMU, and publish `/odom` and the `odom → base_link` transform.
7. **SLAM.** Build a map with `/scan` and `/odom`.
8. **Nav2.** Plan paths on the map and send `/cmd_vel`.
9. **Real-hardware validation.** Repeat each step on the physical robot, starting with the joint controller.

**Why Nav2 must not be attempted first.** Nav2 only *decides where to go*. It sends
`/cmd_vel` commands and expects the robot to:

- move;
- report its motion on `/odom`.

Today, nothing turns `/cmd_vel` into leg motion (steps 1–5), and nothing produces `/odom`
(step 6). Nav2 would therefore start, fail to localise, and send commands that nothing
executes. Building the foundation first means each layer can be tested on its own, and
problems are found where they happen.
