/* All project content shown by the console. Every project-specific statement carries an evidence label:
   C = VERIFIED IN CODE (read in source at the snapshot commit)
   T = VERIFIED BY TEST EVIDENCE (raw evidence files committed in the repository, inspected for this console)
   R = REPORTED / NOT RECHECKED (stated in project documents; no raw log in the repository, nothing rerun)
   P = PLANNED (roadmap item, not implemented) */
const D = 'docs/', SRC = 'src/', CTL = 'src/spiderx_controller/', DSC = 'src/spiderx_description/';
const RUN3 = 'docs/evidence/m61_cloud_one_cycle/20261012T103615Z/';
const LABELS = {
  C: { name: 'VERIFIED IN CODE', short: 'IN CODE', help: 'Read directly in the source files at the snapshot commit. Code existing does not prove it runs correctly.' },
  T: { name: 'VERIFIED BY TEST EVIDENCE', short: 'TEST EVIDENCE', help: 'Raw evidence files are committed in the repository and were inspected for this console. The evidence belongs to the run and commit it names, not to a rerun today.' },
  R: { name: 'REPORTED / NOT RECHECKED', short: 'REPORTED', help: 'Stated in a project document. The raw logs are not in the repository and nothing was rerun for this console.' },
  P: { name: 'PLANNED', short: 'PLANNED', help: 'A roadmap item. Not implemented.' },
};
const GLOSSARY = {
  'DOF': 'Degree of freedom. One independent way a mechanism can move. SpiderX has 12: one per motorised joint.',
  'URDF': 'Unified Robot Description Format. An XML file listing every link, joint, mesh, mass and limit of the robot.',
  'xacro': 'A macro layer on top of URDF. It lets one file produce the simulation and hardware variants.',
  'link': 'A rigid part of the robot. It never bends.',
  'joint': 'The connection between two links. A revolute joint rotates about one axis; a fixed joint does not move.',
  'TF': 'The ROS 2 transform system. It publishes where every coordinate frame is relative to its parent.',
  'topic': 'A named stream of messages in ROS 2. One node publishes, any number subscribe.',
  'action': 'A ROS 2 request that takes time. The client sends a goal, receives feedback, then a result.',
  'node': 'One running ROS 2 program.',
  'FK': 'Forward kinematics. Joint angles in, foot position out.',
  'IK': 'Inverse kinematics. Wanted foot position in, joint angles out.',
  'gait': 'The repeating pattern that decides which feet are on the ground and when.',
  'stance': 'The part of a step when the foot is on the ground and pushes the body.',
  'swing': 'The part of a step when the foot is in the air, moving to its next foothold.',
  'duty factor': 'The fraction of one gait cycle that a foot spends in stance.',
  'base_link': 'The body coordinate frame of SpiderX. Here +y is the front, +x is the right and +z is up, which is not the usual ROS convention.',
  'ros2_control': 'The ROS 2 framework that connects controllers to hardware, or to a simulator standing in for hardware.',
  'Gazebo Fortress': 'The physics simulator used by the project (Ignition Gazebo 6).',
  'fixed base': 'The body is welded in the air so the legs can move without the robot touching the ground.',
};
const MISSION = {
  what: 'SpiderX is a 12 DOF quadruped robot: four legs, each with a hip, a thigh and a knee joint. It exists today as an audited CAD model in a ROS 2 Humble workspace that runs in the Gazebo Fortress simulator.',
  why: 'The repository states an engineering goal, not a personal motive: build a legged robot from the bottom up, where every layer has a test that proves it before the next layer starts, ending with autonomous navigation and then real hardware. Your brief adds a second purpose that the repository does not mention: Torq, an animated character based on the robot, for future educational content.',
  objectives: [
    { t: 'Gait playback and walking in simulation', l: 'P', src: [D + 'SPIDERX_DEVELOPMENT_ROADMAP.md:154'] },
    { t: 'Command velocity bridge: /cmd_vel drives stepping (M5.5)', l: 'P', src: [D + 'SPIDERX_DEVELOPMENT_ROADMAP.md:161'] },
    { t: 'Odometry and state estimation (M7)', l: 'P', src: [D + 'SPIDERX_DEVELOPMENT_ROADMAP.md:169'] },
    { t: 'SLAM and localization (M8), then Nav2 navigation (M9)', l: 'P', src: [D + 'SPIDERX_DEVELOPMENT_ROADMAP.md:179'] },
    { t: 'Real hardware: actuator interface, weighed masses (M10)', l: 'P', src: [D + 'SPIDERX_DEVELOPMENT_ROADMAP.md:189'] },
  ],
  achieved: [
    { t: 'The robot description holds 32 links and 31 joints: 12 revolute, 19 fixed, 30 CAD meshes', l: 'C', src: [DSC + 'urdf/spiderx.urdf.xacro'], note: 'Counted from the file for this console. The audit counted 31 and 30 before lidar_link was added.' },
    { t: 'It spawns in Gazebo Fortress with a simulated 360 degree lidar on /scan', l: 'T', src: ['media/local_validation/gazebo_lidar_rays_collisions.png', DSC + 'urdf/spiderx_fortress.gazebo.xacro:25'], note: 'The screenshots show the spawned model and lidar rays. They do not show topic rates.' },
    { t: 'All 12 joints accept position commands through gz_ros2_control (M1)', l: 'R', src: [D + 'M1_TEST_RESULTS.md', 'STATUS.md:174'] },
    { t: 'The CAD neutral posture was held for 10 s of simulation time (M2)', l: 'R', src: [D + 'M2_TEST_RESULTS.md', 'STATUS.md:171'] },
    { t: 'Forward and inverse kinematics for all four legs, with three static poses held in Gazebo (M3, M4)', l: 'R', src: [D + 'M3_TEST_RESULTS.md', D + 'M4_TEST_RESULTS.md', CTL + 'spiderx_controller/leg_kinematics.py:465'] },
    { t: 'Six gaits analysed offline. Four pass their kinematic checks, two fail static stability (M4.5)', l: 'R', src: [D + 'M4_5_TEST_RESULTS.md:118'] },
    { t: 'One approved trot cycle was sent once to the simulated controllers on a welded body and tracked within 0.0085 rad (M6.1)', l: 'T', src: [RUN3 + 'requirements.json', RUN3 + 'run/m61_run/20261012T104415Z/commanded_vs_observed.csv'], note: 'Recomputed for this console from the committed CSV: maximum error 0.00851 rad on lr_thigh_joint. 223 evidence files match their recorded SHA-256.' },
  ],
  unproven: [
    'Walking. No gait has been played with the feet on the ground, in simulation or on hardware.',
    'Balance. Nothing reads body tilt and corrects it. There is no IMU in the model.',
    'Foot contact. The model has no contact sensor, so contact has never been measured.',
    'Real actuators. The 100 N·m effort and 100 rad/s velocity limits are exporter placeholders.',
    'Real mass. The 7.29 kg CAD mass assumes solid steel for every part.',
    'Odometry, mapping and navigation. They are configured but cannot work until the robot moves.',
    'Any hardware. The repository has no servo driver and no microcontroller firmware.',
  ],
};
/* Components for Explore the Robot. `links` filled at runtime for legs. */
const LEG_INFO = {
  lf: { name: 'Front left leg', servo: 'b_s4_1', note: 'The odd one out: its knee axis is +x while the other three knees are -x, so its knee limits are mirrored.' },
  rf: { name: 'Front right leg', servo: 'b_s2_1', note: '' },
  lr: { name: 'Rear left leg', servo: 'b_s3_1', note: '' },
  rr: { name: 'Rear right leg', servo: 'b_s1_1', note: '' },
};
const JOINT_ROLE = {
  hip: { what: 'Hip joint (abduction)', does: 'Swings the whole leg sideways, toward or away from the body, by rotating about the body’s front to back axis.', why: 'Without it the foot could only move in one flat plane. The hip lets the foot be placed left or right, which the robot needs to turn and to shift weight.' },
  thigh: { what: 'Thigh joint (flexion)', does: 'Swings the thigh forward and backward by rotating about the sideways axis.', why: 'Together with the knee it sets how far forward and how low the foot is.' },
  foot: { what: 'Knee joint (named foot joint in the URDF)', does: 'Bends the lower leg relative to the thigh. Its axis is parallel to the thigh axis.', why: 'Thigh and knee together form a two link arm in one plane. That pair is what lifts the foot and extends the leg.' },
};
const COMPONENTS = {
  body: { title: 'Body', kind: 'Structure', what: 'The central shell (base_link), the top plate (top_1) and the four hip servo bodies (b_s1_1 to b_s4_1), all joined by fixed joints.',
    does: 'Carries the four legs and the lidar. In the simulator all of these parts merge into one rigid body of 2.352 kg (CAD steel density).', why: 'Every leg position is measured relative to this body. It is the reference for all kinematics.',
    rel: 'Parent of all four legs and of lidar_link. Root of the leg chains used by forward and inverse kinematics.', l: 'C',
    src: [DSC + 'urdf/spiderx.urdf.xacro:50', DSC + 'urdf/spiderx.urdf.xacro:696', D + 'SPIDERX_URDF_AUDIT.md:92'], links: ['base_link', 'top_1', 'b_s1_1', 'b_s2_1', 'b_s3_1', 'b_s4_1'] },
  lidar: { title: 'Lidar sensor', kind: 'Sensor', what: 'A 2D laser scanner frame (lidar_link) drawn as a cylinder. It is not part of the CAD export; the mount pose is a simulation placeholder.',
    does: 'In simulation a gpu_lidar sensor produces 360 samples at 1 degree steps, range 0.15 to 12 m, at 10 Hz, bridged to the ROS topic /scan.', why: 'Mapping and navigation need range data. Those layers are configured but blocked until the robot can move.',
    rel: 'Fixed to base_link at (0.051, -0.045, 0.141) with a +90 degree yaw, so the lidar’s +x points to the robot’s front.', l: 'C',
    src: [DSC + 'urdf/spiderx.urdf.xacro:903', DSC + 'urdf/spiderx_fortress.gazebo.xacro:25', DSC + 'config/fortress_bridge.yaml:13'], links: ['lidar_link'] },
  f_dummy: { title: 'Frame: dummy_link', kind: 'Coordinate frame', what: 'The root frame of the robot description. It has no shape and no mass.', does: 'Gives the model a single root. In Gazebo it is the model’s canonical link.', why: 'Future odometry must publish odom to dummy_link. Targeting base_link would give that frame two parents.',
    rel: 'Joined to base_link by dummy_joint with zero offset, so the two frames coincide.', l: 'C', src: [DSC + 'urdf/spiderx.urdf.xacro:42', D + 'SPIDERX_SYSTEM_ARCHITECTURE.md:44'], frame: 'dummy_link', links: [] },
  f_base: { title: 'Frame: base_link', kind: 'Coordinate frame', what: 'The body frame. +y is the robot’s front, +x its right, +z up. Its origin is the CAD origin near a corner of the body, not the body centre.', does: 'All forward and inverse kinematics inputs and outputs are expressed in this frame.', why: 'One shared frame lets four legs agree on where their feet are.',
    rel: 'Does not follow REP-103, the ROS convention of +x forward. A REP-103 frame is planned before Nav2.', l: 'C', src: [D + 'M3_FRAME_CONVENTIONS.md:10', D + 'SPIDERX_URDF_AUDIT.md:102'], frame: 'base_link', links: [] },
  f_lidar: { title: 'Frame: lidar_link', kind: 'Coordinate frame', what: 'The frame that every laser scan is expressed in.', does: 'Laser ranges on /scan carry this frame id, so TF can place them around the robot.', why: 'Without a known sensor frame a scan cannot be related to the body.',
    rel: 'Child of base_link, yawed +90 degrees.', l: 'C', src: [DSC + 'urdf/spiderx.urdf.xacro:928'], frame: 'lidar_link', links: ['lidar_link'] },
  f_tip: { title: 'Derived foot tips', kind: 'Reference point (not a frame)', what: 'The lowest point of each foot’s collision mesh at the neutral pose. The URDF has no foot tip frame, so the project derives this point from the mesh.', does: 'It is the point that inverse kinematics places. Recomputed for this console from the STL files: all four tips sit at z = -0.054543 m, each from 13 mesh vertices.', why: 'A leg needs one agreed point to aim. The real contact point on the rounded pad moves slightly as the foot rotates.',
    rel: 'Fixed in each foot link. Never published to TF.', l: 'C', src: [CTL + 'spiderx_controller/leg_kinematics.py:330', D + 'M3_FRAME_CONVENTIONS.md:63'], frame: 'tips', links: ['lf_foot_1', 'rf_foot_1', 'lr_foot_1', 'rr_foot_1'] },
};
const FLOW = [
  { id: 'cmdvel', t: '/cmd_vel velocity command', sub: 'teleop or Nav2', l: 'P', planned: true,
    plain: 'A walking robot is normally driven by a velocity: go forward at this speed, turn at that rate. SpiderX has the topic name reserved but nothing listens to it.',
    io: ['In: operator or planner intent', 'Out: geometry_msgs/Twist on /cmd_vel'], where: ['README.md:321', D + 'SPIDERX_SYSTEM_ARCHITECTURE.md:66'] },
  { id: 'gait', t: 'Gait generator', sub: 'which foot moves when', l: 'P', planned: true,
    plain: 'A gait turns a velocity into a timed pattern of foot targets. The project has an offline gait analyser, but no node that streams a gait to the controllers.',
    io: ['In: velocity command', 'Out: a foot target per leg in base_link'], where: [CTL + 'spiderx_controller/gait_trajectory.py:43', 'STATUS.md:185'] },
  { id: 'client', t: 'Command source', sub: 'a test or replay tool', l: 'C',
    plain: 'Today every motion starts in a small tool that you run by hand. It decides the wanted angle of all 12 joints: one joint step (M1), a named pose (M2), foot targets solved by inverse kinematics (M3, M4), or the one approved trot cycle (M6.1).',
    io: ['In: a pose file, foot targets or a stored trajectory', 'Out: 12 joint angles with a time to reach them'], where: [CTL + 'scripts/test_one_joint.py', CTL + 'spiderx_controller/m4_pose_validation.py', CTL + 'spiderx_controller/m6_gait_replay.py'] },
  { id: 'safety', t: 'Checks before sending', sub: 'refuse, never clamp', l: 'C',
    plain: 'Before anything is sent the tool checks the request. Joint angles must stay inside the URDF limits minus a 0.05 rad margin. Unreachable foot targets are refused, not adjusted. The gait replay adds gates G1 to G8 and a typed confirmation, and its live switch is False in this snapshot.',
    io: ['In: the candidate joint targets', 'Out: the same targets, or a refusal with a reason'], where: [CTL + 'spiderx_controller/leg_kinematics.py:449', CTL + 'spiderx_controller/m61_gates.py:3', CTL + 'spiderx_controller/m61_live_contract.py:19'] },
  { id: 'action', t: 'Action goal', sub: 'FollowJointTrajectory', l: 'C',
    plain: 'The tool sends one goal over a ROS 2 action. An action is a request that takes time: the server accepts it, reports progress, then returns a result. Every goal must name all 12 joints.',
    io: ['In: joint names, positions, time from start', 'Out: accepted or rejected, feedback, final result'], where: [CTL + 'spiderx_controller/trajectory_client.py:16', CTL + 'config/spiderx_ros2_controllers.yaml:48'], topic: '/leg_trajectory_controller/follow_joint_trajectory' },
  { id: 'jtc', t: 'leg_trajectory_controller', sub: 'JointTrajectoryController, 100 Hz', l: 'C',
    plain: 'This controller turns the goal into a smooth stream. On each control cycle it interpolates between the trajectory points and writes one position command per joint.',
    io: ['In: the trajectory goal and the measured joint positions', 'Out: 12 position commands per cycle'], where: [CTL + 'config/spiderx_ros2_controllers.yaml:15', CTL + 'launch/controller.launch.py:77'] },
  { id: 'hw', t: 'gz_ros2_control', sub: 'GazeboSimSystem plugin', l: 'C',
    plain: 'ros2_control talks to "hardware" through a plugin. Here the plugin is a stand in that passes each position command to the matching joint inside Gazebo. On a real robot a servo driver would take this place. That driver does not exist yet.',
    io: ['In: position command interface, 12 joints', 'Out: joint targets inside the simulator; position and velocity state back'], where: [DSC + 'urdf/spiderx_fortress_control.xacro:29', DSC + 'urdf/spiderx_fortress_control.xacro:47'] },
  { id: 'sim', t: 'Gazebo Fortress physics', sub: '1 ms steps, gravity 9.8', l: 'C',
    plain: 'The simulator moves the links, applying gravity, collisions and joint limits. Tracking looks nearly perfect because the joints may use up to 100 N·m, a placeholder far beyond a small servo.',
    io: ['In: joint targets, the world file', 'Out: new link poses every millisecond of simulated time'], where: [DSC + 'worlds/spiderx_fortress.sdf:246', DSC + 'urdf/spiderx.urdf.xacro:725'] },
  { id: 'jsb', t: 'joint_state_broadcaster', sub: 'publishes /joint_states', l: 'C',
    plain: 'The measured joint angles come back out. This broadcaster reads the state interfaces and publishes them on /joint_states. In passive mode, without controllers, a Gazebo plugin and the bridge publish that topic instead, so there is always exactly one publisher.',
    io: ['In: position and velocity state of 12 joints', 'Out: sensor_msgs/JointState on /joint_states'], where: [CTL + 'config/spiderx_ros2_controllers.yaml:18', DSC + 'config/fortress_bridge_control.yaml:1'], topic: '/joint_states' },
  { id: 'rsp', t: 'robot_state_publisher', sub: 'publishes /tf', l: 'C',
    plain: 'It combines the robot description with the joint angles and publishes where every link frame is. RViz uses that to draw the robot, and the kinematics tests compare it with their own calculation.',
    io: ['In: /joint_states and the robot description', 'Out: /tf, /tf_static, /robot_description'], where: [DSC + 'launch/fortress.launch.py:94'], topic: '/tf' },
  { id: 'verify', t: 'Verification feedback', sub: 'result, ground truth, report', l: 'T',
    plain: 'The tool waits for the action result and compares commanded with observed angles. Test launches also bridge the simulator’s ground truth body pose on a separate topic that never enters TF. In the one passed trot cycle the largest error was 0.0085 rad against a 0.05 rad limit.',
    io: ['In: action result, /joint_states, /spiderx/sim/world_poses', 'Out: a JSON report with PASS or FAIL per check'], where: [SRC + 'spiderx_bringup/launch/fortress_posture_hold.launch.py:30', RUN3 + 'requirements.json'], topic: '/spiderx/sim/world_poses' },
];
const SYSMAP = {
  packages: {
    intro: 'Eight packages. Arrows show which package a launch file or tool reaches into. Mapping, localization and navigation are never started by bringup.',
    items: [
      { id: 'spiderx_description', role: 'The robot itself: URDF and xacro, 30 meshes, the Fortress world, the Gazebo to ROS bridge configuration.', l: 'C', src: [DSC + 'package.xml', DSC + 'urdf/spiderx.urdf.xacro'], x: 0, y: 1, st: 'active' },
      { id: 'spiderx_bringup', role: 'Entry points only. Its launch files compose the other packages: passive simulation, controlled simulation, posture hold bench, fixed base, real robot.', l: 'C', src: [SRC + 'spiderx_bringup/launch/fortress_control.launch.py'], x: 1, y: 0, st: 'active' },
      { id: 'spiderx_controller', role: 'Controller configuration, leg kinematics, pose and gait tools, safety layers, and the test suite (836 test functions counted in 43 files).', l: 'C', src: [CTL + 'config/spiderx_ros2_controllers.yaml', CTL + 'spiderx_controller/leg_kinematics.py'], x: 1, y: 1, st: 'active' },
      { id: 'spiderx_scripts', role: 'Diagnostics: read_lidar and read_joint_states.', l: 'C', src: [SRC + 'spiderx_scripts/spiderx_scripts/read_lidar.py'], x: 2, y: 1, st: 'active' },
      { id: 'spiderx_firmware', role: 'Hardware layer: an RPLidar launch file plus actuator and IMU templates. No servo driver, no microcontroller code.', l: 'C', src: [SRC + 'spiderx_firmware/config/actuator_interface_template.yaml', D + 'SPIDERX_HARDWARE_INTERFACE.md:3'], x: 2, y: 0, st: 'template' },
      { id: 'spiderx_mapping', role: 'SLAM Toolbox configuration and launch. Blocked: it needs odometry.', l: 'C', src: [SRC + 'spiderx_mapping/config/slam_toolbox.yaml', 'STATUS.md:194'], x: 0, y: 2, st: 'blocked' },
      { id: 'spiderx_localization', role: 'EKF and AMCL configuration. Blocked: its inputs /spiderx/leg_odometry and /imu/data do not exist.', l: 'C', src: [SRC + 'spiderx_localization/config/ekf.yaml', 'STATUS.md:195'], x: 1, y: 2, st: 'blocked' },
      { id: 'spiderx_navigation', role: 'Nav2 server configurations and a behaviour tree. Blocked: no odom frame and nothing consumes /cmd_vel.', l: 'C', src: [SRC + 'spiderx_navigation/config/controller_server.yaml', 'STATUS.md:197'], x: 2, y: 2, st: 'blocked' },
    ],
    edges: [['spiderx_bringup', 'spiderx_description'], ['spiderx_bringup', 'spiderx_controller'], ['spiderx_bringup', 'spiderx_firmware'], ['spiderx_scripts', 'spiderx_controller']],
  },
  nodes: {
    intro: 'The ROS 2 graph started by fortress_posture_hold.launch.py, read from the launch files. Rounded boxes are nodes, plain labels are topics. This is a drawing of the configuration, not a capture of a running system.',
    items: [
      { id: 'gz', kind: 'node', t: 'Gazebo Fortress', role: 'The simulator process (ros_gz_sim). With control enabled, the gz_ros2_control plugin inside it creates /controller_manager.', l: 'C', src: [DSC + 'launch/fortress.launch.py:106', DSC + 'urdf/spiderx_fortress_control.xacro:47'] },
      { id: 'bridge', kind: 'node', t: 'spiderx_gz_bridge', role: 'ros_gz_bridge parameter_bridge. One way, Gazebo to ROS: /clock and /scan, plus /joint_states only in passive mode.', l: 'C', src: [DSC + 'launch/fortress.launch.py:140', DSC + 'config/fortress_bridge_control.yaml'] },
      { id: 'gtb', kind: 'node', t: 'spiderx_sim_ground_truth_bridge', role: 'Test only. Bridges the simulator’s true link poses to /spiderx/sim/world_poses. Deliberately kept out of /tf: it is not odometry.', l: 'C', src: [SRC + 'spiderx_bringup/launch/fortress_posture_hold.launch.py:46'] },
      { id: 'cm', kind: 'node', t: 'controller_manager', role: 'Runs the control loop at 100 Hz and owns both controllers.', l: 'C', src: [CTL + 'config/spiderx_ros2_controllers.yaml:13'] },
      { id: 'jsb', kind: 'node', t: 'joint_state_broadcaster', role: 'Publishes the measured joint positions and velocities.', l: 'C', src: [CTL + 'config/spiderx_ros2_controllers.yaml:18'] },
      { id: 'jtc', kind: 'node', t: 'leg_trajectory_controller', role: 'Accepts FollowJointTrajectory goals for all 12 joints. Partial goals are refused. Started only after the broadcaster’s spawner exits with code 0.', l: 'C', src: [CTL + 'config/spiderx_ros2_controllers.yaml:24', CTL + 'launch/controller.launch.py:77'] },
      { id: 'rsp', kind: 'node', t: 'robot_state_publisher', role: 'Publishes /robot_description, /tf and /tf_static from the URDF and /joint_states.', l: 'C', src: [DSC + 'launch/fortress.launch.py:94'] },
      { id: 'client', kind: 'node', t: 'test or replay tool', role: 'Run by hand in a second terminal. The only source of joint goals today.', l: 'C', src: [CTL + 'spiderx_controller/trajectory_client.py:19'] },
      { id: 'rviz', kind: 'node', t: 'rviz2 (optional)', role: 'Started with rviz:=true. Shows the robot model, TF and /scan.', l: 'C', src: [SRC + 'spiderx_bringup/launch/fortress_control.launch.py:60'] },
      { id: 't_clock', kind: 'topic', t: '/clock', role: 'Simulation time. Every node uses use_sim_time, so durations do not depend on how fast the PC runs the simulator.', l: 'C', src: [DSC + 'config/fortress_bridge_control.yaml:5'] },
      { id: 't_scan', kind: 'topic', t: '/scan', role: 'sensor_msgs/LaserScan in lidar_link. Gazebo topic /spiderx/scan bridged to ROS.', l: 'C', src: [DSC + 'config/fortress_bridge.yaml:13'] },
      { id: 't_js', kind: 'topic', t: '/joint_states', role: 'sensor_msgs/JointState for the 12 joints. Exactly one publisher in either mode.', l: 'C', src: [DSC + 'urdf/spiderx_fortress.gazebo.xacro:63'] },
      { id: 't_tf', kind: 'topic', t: '/tf, /tf_static', role: 'Frame transforms from robot_state_publisher.', l: 'C', src: [DSC + 'launch/fortress.launch.py:94'] },
      { id: 't_gt', kind: 'topic', t: '/spiderx/sim/world_poses', role: 'tf2_msgs/TFMessage with ground truth poses, for measuring body height and tilt in tests.', l: 'C', src: [SRC + 'spiderx_bringup/launch/fortress_posture_hold.launch.py:30'] },
      { id: 't_act', kind: 'topic', t: 'action: /leg_trajectory_controller/follow_joint_trajectory', role: 'control_msgs/FollowJointTrajectory. A committed capture from the passed run lists 1 server and, after the run, 0 clients.', l: 'T', src: [CTL + 'spiderx_controller/trajectory_client.py:16', RUN3 + 'a2/captures/action_info.txt'] },
      { id: 't_cmdvel', kind: 'topic', t: '/cmd_vel', role: 'Reserved name. No node in the repository subscribes to it.', l: 'P', src: ['README.md:321'] },
    ],
    edges: [['gz', 't_clock', 'bridge'], ['gz', 't_scan', 'bridge'], ['gz', 't_gt', 'gtb'], ['cm', 'jsb'], ['cm', 'jtc'], ['gz', 'cm', 'hosts'], ['jsb', 't_js'], ['t_js', 'rsp'], ['rsp', 't_tf'], ['client', 't_act'], ['t_act', 'jtc'], ['t_js', 'client'], ['t_gt', 'client'], ['t_tf', 'rviz'], ['t_scan', 'rviz']],
  },
  tf: {
    intro: 'The TF tree is about geometry, not programs: which coordinate frame hangs off which. Read from the URDF. Fixed joints are solid lines, the 12 revolute joints are orange.',
  },
  sim: {
    intro: 'How the simulator and ROS 2 are joined. Gazebo has its own topics. Only what the bridge or the control plugin carries across reaches ROS.',
    rows: [
      { gz: 'gpu_lidar on lidar_link → /spiderx/scan', via: 'ros_gz_bridge', ros: '/scan', l: 'C', src: [DSC + 'config/fortress_bridge.yaml:13'] },
      { gz: 'simulation clock → /clock', via: 'ros_gz_bridge', ros: '/clock', l: 'C', src: [DSC + 'config/fortress_bridge.yaml:7'] },
      { gz: 'JointStatePublisher → /spiderx/joint_states (passive mode only)', via: 'ros_gz_bridge', ros: '/joint_states', l: 'C', src: [DSC + 'config/fortress_bridge.yaml:19', DSC + 'urdf/spiderx_fortress.gazebo.xacro:63'] },
      { gz: 'joint position targets and joint state (control mode)', via: 'gz_ros2_control GazeboSimSystem', ros: 'position command, position and velocity state interfaces', l: 'C', src: [DSC + 'urdf/spiderx_fortress_control.xacro:21'] },
      { gz: 'SceneBroadcaster → /world/spiderx_fortress/pose/info', via: 'extra bridge in test launches', ros: '/spiderx/sim/world_poses', l: 'C', src: [SRC + 'spiderx_bringup/launch/fortress_posture_hold.launch.py:29'] },
      { gz: 'robot_description topic', via: 'ros_gz_sim create', ros: 'spawns the model named spiderx at z = 0.075 m', l: 'C', src: [DSC + 'launch/fortress.launch.py:125'] },
    ],
  },
};
/* Gait data copied from src/spiderx_controller/config/m4_5_gaits.yaml; verdicts from docs/M4_5_TEST_RESULTS.md:118 */
const GAIT_PARAMS = { step_length_m: 0.04, step_height_m: 0.015, body_speed_m_s: 0.005 };
const GAITS = {
  wave: { duty_factor: 0.85, phase_offsets: { front_left: 0.25, front_right: 0.75, rear_left: 0.0, rear_right: 0.5 }, verdict: 'FAIL', failed: 'static stability, joint speed', desc: 'One leg swings at a time with a long four foot overlap.', minFeet: 3 },
  tripod_crawl: { duty_factor: 0.75, phase_offsets: { front_left: 0.25, front_right: 0.75, rear_left: 0.0, rear_right: 0.5 }, verdict: 'FAIL', failed: 'static stability', desc: 'Always at least three feet down, one swinging.', minFeet: 3 },
  ripple: { duty_factor: 0.65, phase_offsets: { front_left: 0.25, front_right: 0.75, rear_left: 0.0, rear_right: 0.5 }, verdict: 'PASS', failed: '', desc: 'Overlapping swings. Alternates three and two foot support.', minFeet: 2 },
  amble: { duty_factor: 0.55, phase_offsets: { front_left: 0.25, front_right: 0.75, rear_left: 0.0, rear_right: 0.5 }, verdict: 'PASS', failed: '', desc: 'Four beat gait, mostly two foot support.', minFeet: 2 },
  pace: { duty_factor: 0.5, phase_offsets: { front_left: 0.0, front_right: 0.5, rear_left: 0.0, rear_right: 0.5 }, verdict: 'PASS', failed: '', desc: 'The two legs on one side swing together.', minFeet: 2 },
  trot: { duty_factor: 0.5, phase_offsets: { front_left: 0.0, front_right: 0.5, rear_left: 0.5, rear_right: 0.0 }, verdict: 'PASS', failed: '', desc: 'Diagonal pairs swing together. The project’s reference gait.', minFeet: 2 },
};
const GALLERY = [
  { img: 'local_gazebo', title: 'SpiderX spawned in Gazebo Fortress, owner’s PC', shows: 'The model spawned from the URDF in the test world, with the entity tree, on Ubuntu 22.04.', matters: 'First proof that the audited CAD model loads in the target simulator on your own machine.', prov: 'Captured on the owner’s Ubuntu PC during the first local test.', not: 'It does not show any joint being controlled. The model is passive here.', l: 'T' },
  { img: 'local_lidar', title: 'Lidar rays and collision shapes', shows: 'Gazebo’s collision view in magenta and the simulated lidar rays in green. The real time factor reads 25 percent.', matters: 'Shows that the collision meshes and the lidar sensor are active in the simulation.', prov: 'Owner’s Ubuntu PC, first local test.', not: 'It does not show that /scan values are correct. The legs are folded under gravity because nothing actuates them.', l: 'T' },
  { img: 'cloud_rviz', title: 'RViz: robot model, TF and /scan', shows: 'The robot model, the TF frames and red laser points outlining walls and obstacles. Global Status reads Ok and the fixed frame is dummy_link.', matters: 'Evidence that the description, joint states, TF and the scan agree well enough to draw one consistent picture.', prov: 'Automated cloud validation on a VM without a GPU.', not: 'It does not show odometry or a map. Nothing here localizes the robot.', l: 'T' },
  { img: 'cloud_gui', title: 'Passive model in the Fortress GUI', shows: 'SpiderX in the test world with the lidar cylinder on the top plate and the legs passively folded.', matters: 'The baseline: what the robot does with no control at all.', prov: 'Automated cloud validation, software rendering.', not: 'No standing. No control.', l: 'T' },
  { img: 'cloud_m1', title: 'M1: twelve joints held at the neutral pose', shows: 'The robot after test_neutral_pose.py, with the legs held at the CAD neutral angles by position control.', matters: 'The first picture where the controllers, not gravity, decide the leg shape.', prov: 'Cloud validation after the M1 neutral pose test.', not: 'It is not a standing controller. A still image cannot show tracking error, and the joints use a 100 N·m placeholder limit.', l: 'T' },
  { img: 'cloud_m2', title: 'M2: neutral posture hold', shows: 'The robot resting on four feet in the neutral pose inside the walled test world.', matters: 'Captured after the posture hold test reported that the 10 s hold stayed within its thresholds.', prov: 'Cloud validation after run_posture_hold_test.py.', not: 'Not balance, not walking. Foot contact was not measured.', l: 'T' },
  { img: 'm61_side', title: 'M6.1: body welded at 0.125 m, side view', shows: 'The robot fixed in the air above the ground after the one passed trot cycle. The feet hang clear of the floor.', matters: 'This is the setup of the only gait trajectory ever executed. Seeing the gap under the feet explains why that run proves trajectory tracking and nothing about walking.', prov: 'Cloud run 20261012T103615Z, taken after the run. Part of an evidence bundle of 223 files whose SHA-256 values were rechecked for this console.', not: 'It does not show walking, contact or balance. The feet never touched the ground.', l: 'T' },
  { img: 'm61_foot', title: 'M6.1: foot level view', shows: 'The same welded robot seen from foot height.', matters: 'A second view of the clearance between feet and floor.', prov: 'Cloud run 20261012T103615Z.', not: 'Same limits as the side view.', l: 'T' },
];
const CODE_EXCERPTS = [
  { title: 'Inverse kinematics refuses, it never clamps', path: CTL + 'spiderx_controller/leg_kinematics.py', line: 465, l: 'C',
    code: 'def inverse(geom, target, frame=FRAME, reference=None, margin=DEFAULT_MARGIN_RAD):\n    """Analytic position IK for the foot tip.\n    ...\n    Never clamps: a candidate outside the limits (minus margin) is rejected, not adjusted.\n    """',
    matters: 'A leg that cannot reach a target says so. Silent clamping would hide a wrong command.', not: 'The docstring states intent. Whether the code honours it is shown by tests, which were not rerun for this console.' },
  { title: 'Live gait dispatch is switched off', path: CTL + 'spiderx_controller/m61_live_contract.py', line: 19, l: 'C',
    code: 'M61_LIVE_DISPATCH_ENABLED = False', matters: 'In this snapshot the replay tool cannot send a goal to a live controller. The passed run used a temporary two line local change that was then restored and verified.', not: 'It does not prevent the older M1 to M4 test tools from sending their own goals.' },
  { title: 'Every goal must command all 12 joints', path: CTL + 'config/spiderx_ros2_controllers.yaml', line: 48, l: 'C',
    code: 'leg_trajectory_controller:\n  ros__parameters:\n    command_interfaces:\n      - position\n    state_interfaces:\n      - position\n      - velocity\n    allow_partial_joints_goal: false',
    matters: 'A goal that forgets a joint is rejected instead of leaving that joint undefined.', not: 'Configuration only.' },
];
const ASSETS_NEEDED = [
  { t: 'Walking or standing clip', why: 'The media guide says these may be added only after the capability exists and passes its roadmap test. It does not exist yet.', src: 'media/README.md:39' },
  { t: 'TF tree image from view_frames', why: 'Listed in the media guide as still to capture. The tree in this console is drawn from the URDF instead.', src: 'media/README.md:38' },
  { t: 'CAD renders from Fusion 360', why: 'No CAD screenshots are in the repository. The 3D view here uses the exported STL meshes.', src: 'not in repository' },
  { t: 'Photos of the physical robot', why: 'No hardware photos are in the repository, and no hardware test is recorded.', src: 'not in repository' },
  { t: 'Torq character art', why: 'Torq is not mentioned anywhere in the repository. Supply the artwork if you want it shown here.', src: 'not in repository' },
];
/* Milestones. cat: done | unverified | progress | planned */
const MILESTONES = [
  { id: 'M0', t: 'Model and simulation', cat: 'done', date: '2026-09-25', l: 'R', obj: 'Get the CAD model into Gazebo Fortress with a lidar, joint states and TF.', impl: 'URDF audit, sim_backend switch, Fortress world, bridge configuration, leg and joint groups checked against the URDF.', ev: 'Reported verified locally and in the cloud. Four screenshots are committed.', lim: 'Passive model. Mass is CAD steel density. The real time factor on the first local PC was 21 to 25 percent.', src: [D + 'SPIDERX_DEVELOPMENT_ROADMAP.md:20', D + 'SPIDERX_URDF_AUDIT.md'] },
  { id: 'M1', t: 'Joint position control', cat: 'done', date: '2026-09-25', l: 'R', obj: 'Command the 12 joints in simulation.', impl: 'gz_ros2_control with a joint state broadcaster and a joint trajectory controller.', ev: 'Reported: both controllers active, a 0.2 rad step on lf_hip tracked, all joints driven to the neutral pose, 26 local test steps passed.', lim: 'Tracking is idealised by the 100 N·m placeholder. Two roadmap boxes remain open: reaching both soft limits on a lifted robot, and real servo limits.', src: [D + 'M1_TEST_RESULTS.md', D + 'SPIDERX_DEVELOPMENT_ROADMAP.md:27'] },
  { id: 'M2', t: 'Simulation posture hold', cat: 'done', date: '2026-09-25', l: 'R', obj: 'Hold the CAD neutral pose for 10 s and measure it.', impl: 'run_posture_hold_test.py with a ground truth pose bridge and a JSON report.', ev: 'Reported over 3 runs: body height min 0.05448 m, roll and pitch within 0.00022 rad, joint error within 0.00028 rad.', lim: 'Not balance. Foot contact not measured. No disturbance test.', src: [D + 'M2_TEST_RESULTS.md', 'STATUS.md:171'] },
  { id: 'M3', t: 'Single leg kinematics', cat: 'done', date: '2026-09-26', l: 'R', obj: 'Forward and inverse kinematics for the front left leg, taken from the URDF.', impl: 'leg_kinematics.py: URDF chain, derived foot tip, analytic inverse kinematics that refuses bad targets.', ev: 'Reported: FK matched TF within 5.4e-11 m, 5 lifted targets reached, 2 negative targets refused with 0 goals sent.', lim: 'Simulation only. One leg at this stage.', src: [D + 'M3_TEST_RESULTS.md', CTL + 'spiderx_controller/leg_kinematics.py'] },
  { id: 'M4', t: 'All leg kinematics and static poses', cat: 'done', date: '2026-09-29', l: 'R', obj: 'Extend kinematics to four legs and hold three static poses.', impl: 'Opt in all leg FK and IK, pose targets neutral_stance, crouch_10mm, lift_lf_15mm.', ev: 'Reported: foot tips within 1.2e-10 m of target, 5 s holds, tilt within 2.8e-5 rad.', lim: 'Static poses only. Contact not measured.', src: [D + 'M4_TEST_RESULTS.md', CTL + 'config/m4_pose_targets.yaml'] },
  { id: 'M4.1', t: 'Controller start up robustness', cat: 'done', date: '2026-10-01', l: 'R', obj: 'Remove an intermittent start up race between the two controllers.', impl: 'The trajectory controller’s spawner starts only after the broadcaster’s spawner exits with code 0.', ev: 'Reported: 385 tests, forced failure never loaded the trajectory controller, clean Ctrl+C.', lim: 'Start up ordering only.', src: [D + 'M4_1_TEST_RESULTS.md', CTL + 'launch/controller.launch.py:56'] },
  { id: 'M4.5', t: 'Offline gait analysis', cat: 'done', date: '2026-10-01', l: 'R', obj: 'Check six gait patterns against the leg geometry without a simulator.', impl: 'YAML gaits, phase engine, cycloid swing trajectories, sampled IK, a static stability approximation.', ev: 'Reported: all six kinematically feasible, wave and tripod_crawl fail static stability, 556 tests, byte identical outputs on two machines.', lim: 'Offline model only. Outputs are not committed. One roadmap box open: paper evaluation with better mass data.', src: [D + 'M4_5_TEST_RESULTS.md', CTL + 'config/m4_5_gaits.yaml'] },
  { id: 'M5', t: 'Offline evaluation study', cat: 'done', date: '2026-10-02', l: 'R', obj: 'A staged, reproducible study of gait configuration classes.', impl: 'Study specification, runner with a gate, provenance records.', ev: 'Reported: 700 tests, two byte identical runs, hypotheses H1 to H3 supported as model predictions.', lim: 'Model predictions, not measurements. The citations review step has not started.', src: [D + 'M5_TEST_RESULTS.md', D + 'SPIDERX_DEVELOPMENT_ROADMAP.md:113'] },
  { id: 'M6.0', t: 'Gait playback safety layers', cat: 'progress', date: '2026-10-02', l: 'R', obj: 'Make it safe to replay one bounded crouch trajectory through the controllers.', impl: 'Offline preflight with 32 refusal codes, a single goal action client, a read only live preflight, goal fingerprint, confirmation word.', ev: 'Reported: 1116 tests in offline, mock and isolated domain modes. Read only preflight READY on the owner’s PC.', lim: 'The crouch goal itself has never been sent. LIVE_DISPATCH_ENABLED is False. Two roadmap boxes open: the enabling commit and the local live playback.', src: [D + 'M6D_LIVE_PLAYBACK_RESULTS.md', CTL + 'spiderx_controller/m6_live_contract.py:18'] },
  { id: 'M6.1', t: 'One trot cycle on a fixed base', cat: 'progress', date: '2026-10-12', l: 'T', obj: 'Execute one approved trot cycle on a body welded in the air, with full supervision.', impl: 'Welded base model at 0.125 m, clearance analysis, gates G1 to G8, read only observer, one cycle of 4.0 s with 2 cm step and 6 mm lift.', ev: 'Committed evidence: third authorized cloud cycle PASS, 1 goal, 0 cancels, max error 0.0085 rad, requirements E1 to E11. Two earlier attempts are recorded as FAIL.', lim: 'Cloud only. The launch only observation on the owner’s PC has not been run. Feet never touched the ground.', src: [RUN3 + 'README.md', D + 'M61A_CLOUD_VERIFICATION.md:1501'] },
  { id: 'Nav', t: 'Mapping, localization, navigation configs', cat: 'unverified', date: '', l: 'R', obj: 'Prepare SLAM Toolbox, EKF, AMCL and Nav2 for later.', impl: 'Configuration and launch files in three packages.', ev: 'Reported: nodes start and parameters load in the cloud. Nav2 stops at "frame odom does not exist".', lim: 'Cannot work until the robot moves and reports odometry. Never started by bringup.', src: ['STATUS.md:190'] },
  { id: 'Walk', t: 'Gait playback and walking in simulation', cat: 'planned', date: '', l: 'P', obj: 'Static walk, then trot, with feet on the ground. Walk 1 m without falling.', impl: 'None. Unscheduled by owner decision.', ev: 'None.', lim: 'A video is required before claiming it.', src: [D + 'SPIDERX_DEVELOPMENT_ROADMAP.md:154'] },
  { id: 'M5.5', t: 'Command velocity bridge', cat: 'planned', date: '', l: 'P', obj: 'Stepping follows /cmd_vel, with a safe timeout.', impl: 'None.', ev: 'None.', lim: 'Needs its own owner approved phases, starting with a shadow session.', src: [D + 'SPIDERX_DEVELOPMENT_ROADMAP.md:161'] },
  { id: 'M7', t: 'Odometry and state estimation', cat: 'planned', date: '', l: 'P', obj: 'Leg odometry, EKF fusion with an IMU, a REP-103 base frame.', impl: 'None. The EKF configuration exists but has no inputs.', ev: 'None.', lim: 'No IMU link in the URDF.', src: [D + 'SPIDERX_DEVELOPMENT_ROADMAP.md:169'] },
  { id: 'M8', t: 'SLAM and localization', cat: 'planned', date: '', l: 'P', obj: 'Build a map of the test world and localize on it.', impl: 'Configuration only.', ev: 'None.', lim: 'Blocked by M7.', src: [D + 'SPIDERX_DEVELOPMENT_ROADMAP.md:179'] },
  { id: 'M9', t: 'Nav2 navigation', cat: 'planned', date: '', l: 'P', obj: 'Reach a goal in simulation.', impl: 'Configuration only.', ev: 'None.', lim: 'Velocity limits are placeholders until a gait is measured.', src: [D + 'SPIDERX_DEVELOPMENT_ROADMAP.md:184'] },
  { id: 'M10', t: 'Real hardware', cat: 'planned', date: '', l: 'P', obj: 'An actuator interface, then repeat every milestone on the robot.', impl: 'Templates only.', ev: 'None.', lim: 'Servo model, controller board, IMU and embedded computer are not defined in the repository.', src: [D + 'SPIDERX_DEVELOPMENT_ROADMAP.md:189', D + 'SPIDERX_HARDWARE_INTERFACE.md'] },
];
const CATS = { done: 'Completed', unverified: 'Implemented but unverified', progress: 'In progress', planned: 'Planned' };
const LESSONS = [
  { t: 'What is a robot?', obj: 'Name the three things every robot does.', body: 'A robot senses, decides and acts, in a loop. SpiderX can sense (a lidar and joint angle readings) and act (twelve joints that follow position commands). The deciding layer is the thin part today: a person runs a tool that sends one prepared motion.', vis: 'loop', src: 'README.md:62', ex: 'Open Mission Control and sort each achievement into sense, decide or act.', q: 'Which part of the loop is least developed in SpiderX today?', opts: ['Sensing', 'Deciding', 'Acting'], a: 1, why: 'Nothing turns a goal such as "walk forward" into motion yet. The gait generator and velocity bridge are planned.' },
  { t: 'Links and joints', obj: 'Tell a link from a joint.', body: 'A link is a rigid part. A joint connects two links. SpiderX’s description has 32 links and 31 joints. Only 12 joints rotate. The other 19 are fixed: they glue parts such as a servo horn to its bracket.', vis: 'robot:axes', src: DSC + 'urdf/spiderx.urdf.xacro:744', ex: 'In Explore the Robot, switch to Joint axes and count the orange axes on one leg.', q: 'Why does a 12 joint robot have 31 joints in its file?', opts: ['The file has errors', 'Most joints are fixed and only glue parts together', 'Each motor counts three times'], a: 1, why: 'The CAD export keeps every part as its own link, joined by fixed joints.' },
  { t: 'Degrees of freedom', obj: 'Explain what 12 DOF means.', body: 'A degree of freedom is one independent way to move. Each revolute joint adds one. Four legs with three joints each give twelve. A point in space needs three numbers, so three joints per leg is the minimum to place a foot anywhere within reach.', vis: 'robot:leg', src: CTL + 'config/spiderx_legs.yaml:27', ex: 'Select the front left leg and move its three sliders one at a time. Watch which direction the foot goes.', q: 'With three joints, what can a leg not control?', opts: ['The foot position', 'The foot orientation', 'The foot height'], a: 1, why: 'Three joints set three numbers: x, y and z of the foot. The project’s inverse kinematics is position only for that reason.' },
  { t: 'Why joints must be coordinated', obj: 'See why one joint alone is never enough.', body: 'Moving one joint swings the foot along an arc. To move a foot straight up by 10 mm, thigh and knee must both change, by different amounts. And all four feet share one body, so a pose is always 12 numbers sent together. The controller refuses a goal that names fewer.', vis: 'robot:crouch', src: CTL + 'config/spiderx_ros2_controllers.yaml:48', ex: 'In the Lab, raise the foot target by 10 mm and read how many joints changed.', q: 'To lift a foot straight up, how many joints of that leg change?', opts: ['One', 'Two or more', 'None'], a: 1, why: 'A single joint only draws an arc. A straight line needs at least thigh and knee together.' },
  { t: 'The robot description', obj: 'Know what the URDF is and why there is only one.', body: 'The URDF is the robot written as data: every link, mesh, mass, joint origin, axis and limit. SpiderX’s file came from a Fusion 360 export. The project treats it as the single source of truth: the kinematics code reads lengths from it instead of typing them in, so a CAD change cannot silently disagree with the math.', vis: 'robot:frames', src: DSC + 'urdf/spiderx.urdf.xacro:744', ex: 'Open the evidence drawer on any joint and note the line number of its definition.', q: 'Where does leg_kinematics.py get the leg’s dimensions?', opts: ['Typed constants', 'The expanded URDF', 'A measurement of the real robot'], a: 1, why: 'Its header says no link length or offset is typed into the module.' },
  { t: 'What ROS 2 does here', obj: 'Name nodes, topics and actions.', body: 'ROS 2 lets separate programs, called nodes, exchange messages. A topic is a continuous stream, such as /joint_states. An action is a request with a result, such as "follow this trajectory". SpiderX’s nodes come from standard packages. The project’s own code is mostly configuration, launch files and test tools.', vis: 'map:nodes', src: SRC + 'spiderx_bringup/launch/fortress_control.launch.py', ex: 'In the System Map, find which node publishes /joint_states in control mode.', q: 'Sending a trajectory and waiting for "reached" is best done with a', opts: ['topic', 'action', 'parameter'], a: 1, why: 'An action carries a goal, feedback and a final result.' },
  { t: 'What Gazebo simulates', obj: 'Know what the simulator gives you and what it does not.', body: 'Gazebo computes physics in 1 ms steps: gravity, collisions between meshes, joint limits. It also renders a lidar. It does not know your real servos. The joints may push with 100 N·m and every part weighs as if it were solid steel. Results are therefore results about this model.', vis: 'img:local_lidar', src: DSC + 'worlds/spiderx_fortress.sdf:246', ex: 'Open the lidar picture in the Evidence Gallery and find the real time factor.', q: 'Joint tracking error in simulation is tiny. What is the main reason to distrust it?', opts: ['The simulator is buggy', 'The actuator limits are placeholders far above a real servo', 'The lidar is noisy'], a: 1, why: 'The project documents call the tracking idealised for this reason.' },
  { t: 'How a command becomes movement', obj: 'Follow one command from tool to joint and back.', body: 'A tool builds a goal for 12 joints. Checks run first. The goal goes over an action to the trajectory controller, which writes position commands at 100 Hz. A plugin hands them to Gazebo, physics moves the links, and the measured angles return on /joint_states.', vis: 'flow', src: CTL + 'spiderx_controller/trajectory_client.py:65', ex: 'Step through all stages in How a Command Becomes Motion.', q: 'Which stage would a real servo driver replace?', opts: ['The trajectory controller', 'The gz_ros2_control plugin', 'robot_state_publisher'], a: 1, why: 'ros2_control keeps the controllers the same and swaps only the hardware plugin.' },
  { t: 'How we know something worked', obj: 'Separate a claim from its proof.', body: 'This project is strict about proof. A test must be able to fail: bad targets are sent on purpose to check they are refused. Reports list what was not shown. In the trot run the rule was written down: missing evidence is UNMEASURED, never PASS, and SUCCEEDED alone is not a pass. This console adds its own rule: four labels that say how far each claim was checked.', vis: 'labels', src: RUN3 + 'requirements.json', ex: 'Find one item of each label colour in Mission Control.', q: 'A report says a test passed, but the raw log is not in the repository. Which label fits?', opts: ['VERIFIED BY TEST EVIDENCE', 'REPORTED / NOT RECHECKED', 'VERIFIED IN CODE'], a: 1, why: 'A statement in a document is a report until the raw evidence is inspected or the test is rerun.' },
  { t: 'Forward kinematics', obj: 'Compute where the foot is from three angles.', body: 'Forward kinematics walks down the leg. Start at the body, apply each joint’s fixed offset, then its rotation, and continue to the foot. The result is unique: three angles give exactly one foot position.', vis: 'lab:fk', src: CTL + 'spiderx_controller/leg_kinematics.py:411', ex: 'In the Lab, set the thigh to +0.2 rad on the front left leg. Does the foot go forward or back?', q: 'For the front left leg, a positive thigh angle moves the foot', opts: ['forward', 'rearward', 'sideways'], a: 1, why: 'Its axis is [-1, 0, 0]. The sign table in the frame conventions document says rearward and slightly up.' },
  { t: 'Inverse kinematics', obj: 'Understand why the reverse problem is harder.', body: 'Inverse kinematics asks for the angles that put the foot at a wanted point. There can be several answers, or none. The project solves it with geometry: the hip angle first, then thigh and knee as a two link arm. It picks the answer closest to the neutral pose, and refuses a target that is out of reach or needs a joint beyond its limit.', vis: 'lab:ik', src: CTL + 'spiderx_controller/leg_kinematics.py:465', ex: 'In the Lab, push the foot target down until the solver refuses. Read the reason.', q: 'What does the project’s solver do with an unreachable target?', opts: ['Moves as close as it can', 'Refuses and reports why', 'Picks random angles'], a: 1, why: 'It never clamps. A refused target sends no goal.' },
  { t: 'Gaits: stance and swing', obj: 'Read a gait diagram.', body: 'In stance a foot is on the ground and moves backward relative to the body, which pushes the body forward. In swing it lifts and moves ahead. The duty factor is the fraction of time in stance. Phase offsets set the order. In a trot, diagonal legs swing together and the duty factor is 0.5.', vis: 'lab:gait', src: CTL + 'spiderx_controller/gait_phase.py:38', ex: 'In the Lab, compare wave and trot. How many feet are down at the worst moment in each?', q: 'A duty factor of 0.75 means each foot is on the ground', opts: ['a quarter of the cycle', 'three quarters of the cycle', 'the whole cycle'], a: 1, why: 'Duty factor is the stance fraction.' },
  { t: 'Safety gates and the fixed base run', obj: 'Explain what the one passed gait run did and did not prove.', body: 'Before a gait touches the ground, the project welded the body 0.125 m in the air and played one trot cycle. Gates watched body height, tilt, joint limits, stalls, data gaps and the weld itself. The run passed on the third authorized attempt. The first was refused before sending and the second was cancelled by a timing defect in the client, both recorded as FAIL. It proves the trajectory can be sent, executed and tracked. It proves nothing about walking.', vis: 'lab:replay', src: D + 'M61A_CLOUD_VERIFICATION.md:1501', ex: 'Play the recorded run in the Lab and find the moment of largest error.', q: 'Why is the passed trot cycle not evidence of walking?', opts: ['It was too slow', 'The body was welded and the feet never touched the ground', 'It used the wrong gait'], a: 1, why: 'With no ground contact there is no support, no slip and no balance to test.' },
];
const BOUNDARY = {
  known: [
    ['The model’s geometry is consistent: all four feet reach the same plane at the neutral pose.', 'C', [D + 'SPIDERX_URDF_AUDIT.md:112']],
    ['The control path from action goal to simulated joint and back is wired as described.', 'C', [CTL + 'config/spiderx_ros2_controllers.yaml']],
    ['One trot cycle was tracked within 0.0085 rad on a welded body in the cloud.', 'T', [RUN3 + 'requirements.json']],
    ['The source tree under src/ is identical to the commit the passed run was made on (b0d8e75).', 'T', [RUN3 + 'README.md'], 'Checked for this console with git diff: no differences.'],
  ],
  uncertain: [
    ['Whether any gait keeps the robot upright with feet on the ground. Two of six gaits already fail the static stability approximation.', 'R', [D + 'M4_5_TEST_RESULTS.md:118']],
    ['The real mass and its distribution. The centre of mass sits about 6 mm behind the foot centre in the CAD model.', 'R', [D + 'SPIDERX_URDF_AUDIT.md:113']],
    ['Whether the controllers hold position after a cancelled goal. It is intended, and no test proves it.', 'R', ['STATUS.md:46']],
    ['Why the simulator ran at 21 to 25 percent of real time on the first local PC.', 'R', ['STATUS.md:162']],
  ],
  retest: [
    ['The fixed base observation and the trot cycle on your own PC. Both have only run in the cloud.', 'R', ['STATUS.md:184']],
    ['The single crouch playback of M6.0-D. Implemented, never sent to a live controller.', 'R', ['STATUS.md:35']],
    ['Every joint reaching both soft limits on a lifted robot. Still an open box in M1.', 'R', [D + 'SPIDERX_DEVELOPMENT_ROADMAP.md:39']],
    ['Headless rendering. The option exists and is untested.', 'R', ['STATUS.md:163']],
  ],
  simonly: [
    ['Everything. Every verified result in the repository comes from Gazebo Fortress or from offline calculation.', 'R', ['STATUS.md']],
    ['Joint effort 100 N·m and velocity 100 rad/s are exporter placeholders.', 'C', [DSC + 'urdf/spiderx.urdf.xacro:725']],
    ['Friction 0.2 is a template default, not a measured value.', 'C', [DSC + 'urdf/spiderx_fortress.gazebo.xacro:76']],
    ['The lidar mount pose is a simulation placeholder.', 'C', [DSC + 'urdf/spiderx.urdf.xacro:894']],
  ],
  hardware: [
    ['Identify the servo model, controller board and bus. Record torque and speed from the datasheet.', 'P', [D + 'SPIDERX_HARDWARE_INTERFACE.md:36']],
    ['Map all 12 joints to servo ids, measured direction signs and zero offsets.', 'P', [SRC + 'spiderx_firmware/config/actuator_interface_template.yaml']],
    ['Write a ros2_control hardware interface that clamps to the soft limits.', 'P', [D + 'SPIDERX_HARDWARE_INTERFACE.md:38']],
    ['Weigh the parts and replace the steel density masses.', 'P', [D + 'SPIDERX_DEVELOPMENT_ROADMAP.md:192']],
    ['Add an IMU and its frame from a measured mounting pose.', 'P', [D + 'SPIDERX_HARDWARE_INTERFACE.md:53']],
    ['First powered test with the robot lifted, a current limit and an emergency stop within reach.', 'P', [D + 'SPIDERX_HARDWARE_INTERFACE.md:46']],
  ],
};
const NEXT = [
  { t: 'Review the record of the passed trot cycle', why: 'The project’s own report lists owner review as the first next step. Reading it also teaches the full safety chain.', src: D + 'M61A_CLOUD_VERIFICATION.md:1564', basis: 'Stated in the project report' },
  { t: 'Run the fixed base observation on your own PC', why: 'It has only run in the cloud. The local run confirms the provisional 0.125 m mount height and the pose tolerances on your machine.', src: D + 'SPIDERX_DEVELOPMENT_ROADMAP.md:151', basis: 'Open roadmap box' },
  { t: 'Decide on the M6.0-D crouch playback', why: 'The enabling commit is two lines and waits for a separate approval. It is the smallest possible live motion through the safety layers.', src: D + 'SPIDERX_DEVELOPMENT_ROADMAP.md:147', basis: 'Open roadmap box' },
  { t: 'Start the command velocity work with its shadow session', why: 'The cloud trot cycle no longer blocks it. A shadow session computes commands without sending them, which fits the project’s habit of proving each layer first.', src: D + 'M61A_CLOUD_VERIFICATION.md:1566', basis: 'Stated in the project report' },
  { t: 'Play a gait with the feet on the ground in simulation', why: 'This is the first test that can say anything about walking. It needs the earlier steps so that a fall can be told apart from a software fault.', src: D + 'SPIDERX_DEVELOPMENT_ROADMAP.md:154', basis: 'Roadmap order' },
  { t: 'Get real mass and servo data', why: 'Stability margins of a few millimetres depend on where the mass really is. Simulation results stay provisional until then.', src: D + 'URDF_INERTIA_AUDIT.md', basis: 'Roadmap, M10 prerequisite' },
];
