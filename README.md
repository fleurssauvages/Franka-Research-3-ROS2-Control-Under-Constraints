# FR3 Low-Level and Cartesian Control Stack

ROS 2 Jazzy control stack for a **Franka Research 3 (FR3)** with:

- hardware-facing joint velocity control,
- hardware-facing joint impedance / effort control,
- automatic residual-friction compensation,
- Cartesian inverse-kinematics velocity control,
- constrained Cartesian QP velocity control with proactive FCL capsule self-collision avoidance,
- Cartesian PID position control,
- Cartesian LMPC position control,
- gripper ratio control,
- dynamics observation and friction identification,
- config-driven UDP <-> ROS 2 bridging with Wireshark dissector generation,
- readiness-driven startup instead of fixed launch delays.

The stack is split into three ROS 2 packages:

```text
low_level/
    Hardware-facing ros2_control controllers, gripper control,
    collision configuration, dynamics observation and calibration.

franka_cartesian_control/
    Cartesian IK/QP velocity controllers and PID/LMPC position controllers.
    The QP includes optional FCL capsule self-collision constraints.

udp_bridge/
    Generic JSON-configured UDP -> ROS 2 and ROS 2 -> UDP bridge for common
    standard ROS message packages, plus Wireshark Lua dissector generation.
```

The default robot type is `fr3` and the default robot IP used by the launch files is:

```text
192.16.0.1
```

Change `robot_ip:=...` on the command line if your robot uses another address.

---

## 1. Architecture

The normal command chain is:

```text
PoseStamped
    |
    v
PID or LMPC position controller          50 Hz
    |
    | geometry_msgs/TwistStamped
    v
IK or QP Cartesian velocity controller   200 Hz
    |
    | sensor_msgs/JointState (velocity)
    v
Low-level velocity or impedance layer    1000 Hz ros2_control
    |
    v
Franka Research 3
```

For direct Cartesian velocity control, the PID/LMPC layer is omitted:

```text
TwistStamped -> IK/QP -> JointState -> low_level -> FR3
```

For direct joint control, both Cartesian layers are omitted:

```text
JointState -> low_level -> FR3
```

Startup is readiness-driven. The next controller is started only after the previous layer has published its operational `ready` state and required topics are present.

Important ready topics include:

```text
/low_level_joint_velocity_controller/ready
/low_level_joint_impedance_controller/ready
/dynamics_observer_controller/ready
/ik_velocity_controller/ready
/qp_velocity_controller/ready
```

---

## 2. Safety

This repository controls a real 7-DoF robot at high update rates. Treat every command as potentially capable of producing motion immediately.

Before enabling motion:

1. Make sure the robot is firmly mounted.
2. Clear the full reachable workspace.
3. Verify the configured end effector and payload in Franka Desk.
4. Verify the robot IP and network connection.
5. Enable FCI according to the Franka operating procedure.
6. Keep an emergency stop / safe-stop method available.
7. Start with conservative velocity, acceleration and torque limits.

### QP self-collision safety

The QP now includes proactive self-collision avoidance.  It uses the Flexible Collision Library (FCL) with conservative capsule proxies connecting successive FR3 link-frame origins.  Non-neighbouring capsule pairs become hard control-barrier inequalities in the joint-velocity QP.

The constraint is conceptually:

```text
d_dot >= -gain * (d - margin)
```

where `d` is the current FCL capsule surface-to-surface clearance.  At the configured margin the pair may not move closer; inside the margin the QP requests separating velocity, subject to the current joint velocity and acceleration bounds.

This is intentionally much lighter than MoveIt collision checking, but it is an **approximation**, not an exact mesh guarantee.  The current proxy covers the arm through `link8`; it does not model gripper fingers or arbitrary attached tools as exact meshes.  Validate the configured capsule radii and margin for the real hardware and installed end effector.

Franka's internal collision/reflex system remains the final hardware protection layer.

### Collision thresholds

The low-level launch can optionally call Franka's full collision-behavior service before activating the command controller. This is disabled by default:

```text
configure_collision_behavior:=false
```

Changing collision thresholds changes the robot's safety response. Only change them if you understand the consequences.

---

## 3. Tested workspace layout

The examples in this README assume:

```text
~/franka_ws/
    src/
    build/
    install/

~/franka_ws/
    src/
        low_level/
        franka_cartesian_control/
        udp_bridge/
        qpoases/                  # source clone, optional after install
    qpoases_install/
    data/
        calibration/
        udp/
            udp_reader.json
            udp_publisher.json
            wireshark_udp.lua     # generated example/decoder
    build/
    install/
```

The custom workspace can be located elsewhere, but the examples below use `~/franka_ws`.

---

# 4. Installation

## 4.1 Operating system and ROS 2

The project targets **ROS 2 Jazzy**. The current Franka ROS 2 repository also targets Jazzy on its `jazzy` branch.

Install ROS 2 Jazzy and development tools, for example:

```bash
sudo apt update
sudo apt install ros-jazzy-desktop ros-dev-tools
```

For a headless machine, `ros-jazzy-ros-base` can be used instead of the desktop installation.

Source ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
```

For real hardware, configure the host for reliable real-time communication according to the Franka FCI documentation. A real-time kernel is strongly recommended for 1 kHz control.

---

## 4.2 Install `franka_ros2`

Create the Franka workspace:

```bash
mkdir -p ~/franka_ws/src
cd ~/franka_ws
```

Clone the Jazzy branch:

```bash
git clone -b jazzy https://github.com/frankarobotics/franka_ros2.git src
```

Import its dependencies:

```bash
vcs import src < src/dependency.repos --recursive --skip-existing
```

Install ROS dependencies:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y --skip-keys=zed_wrapper
```

Build:

```bash
colcon build \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF
```

Source it:

```bash
source ~/franka_ws/install/setup.bash
```

Before continuing, verify that the Franka packages are visible:

```bash
ros2 pkg prefix franka_bringup
```

---

## 4.3 Install additional system dependencies

The low-level calibration scripts use NumPy, SciPy and YAML. The QP self-collision layer uses FCL. Wireshark is optional but useful for inspecting UDP traffic:

```bash
sudo apt install \
  build-essential \
  cmake \
  git \
  python3-numpy \
  python3-scipy \
  python3-yaml \
  libfcl-dev \
  wireshark
```

ROS package dependencies can also be resolved with `rosdep` after placing the custom packages in the workspace.

---

## 4.4 Install qpOASES

Both the Cartesian QP controller and LMPC use qpOASES.

Clone it into the custom workspace source directory:

```bash
cd ~/franka_ws/src
git clone https://github.com/coin-or/qpOASES.git qpoases
```

Build and install it into a workspace-local prefix:

```bash
cd ~/franka_ws/src/qpoases
mkdir -p build
cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$HOME/franka_ws/qpoases_install

cmake --build . -j$(nproc)
cmake --install .
```

The custom CMake project looks for qpOASES at:

```text
~/franka_ws/qpoases_install
```

unless `-DQPOASES_ROOT=...` is supplied explicitly.

---

## 4.5 Install the custom packages

Place the three packages in:

```text
~/franka_ws/src/low_level
~/franka_ws/src/franka_cartesian_control
~/franka_ws/src/udp_bridge
```

Keep UDP runtime configuration in the workspace data directory rather than inside the package:

```text
~/franka_ws/data/udp/udp_reader.json
~/franka_ws/data/udp/udp_publisher.json
```

Then install ROS dependencies:

```bash
cd ~/franka_ws
source /opt/ros/jazzy/setup.bash
source ~/franka_ws/install/setup.bash

rosdep install --from-paths src --ignore-src -y
```

Build all three packages:

```bash
cd ~/franka_ws

source /opt/ros/jazzy/setup.bash
source ~/franka_ws/install/setup.bash

colcon build \
  --packages-select low_level franka_cartesian_control udp_bridge \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DQPOASES_ROOT=$HOME/franka_ws/qpoases_install
```

Source the result:

```bash
source ~/franka_ws/install/setup.bash
```

A convenient shell setup is:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ws/install/setup.bash
source ~/franka_ws/install/setup.bash
```

Use that order in every new terminal.

---

# 5. Robot preparation before running controllers

1. Connect the control PC directly to the Franka network.
2. Verify connectivity to the robot IP.
3. Open Franka Desk.
4. Resolve any safety faults.
5. Configure the correct end-effector / payload profile.
6. Activate FCI as required by the Franka system image.
7. Clear the workspace.

For the commands below, replace the IP if necessary:

```bash
ROBOT_IP=192.16.0.1
```

---

# 6. Main topics

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/franka/joint_states` | `sensor_msgs/msg/JointState` | robot -> stack | Measured joint position/velocity/effort |
| `/fr3/joint_commands` | `sensor_msgs/msg/JointState` | controller -> low level | Desired joint velocity or impedance target |
| `/fr3/cartesian_twist_command` | `geometry_msgs/msg/TwistStamped` | user/PID/LMPC -> IK/QP | Desired TCP twist |
| `/fr3/cartesian_pose_command` | `geometry_msgs/msg/PoseStamped` | user -> PID/LMPC | Desired TCP pose |
| `/fr3/cartesian_pose` | `geometry_msgs/msg/PoseStamped` | PID/LMPC -> user | Current TCP pose |
| `/fr3/lmpc_obstacle_mesh` | `visualization_msgs/msg/Marker` | user -> LMPC | Runtime triangle-mesh obstacle |
| `/fr3/gripper` | `std_msgs/msg/Float32` | user -> gripper node | Gripper opening ratio |
| `/fr3/dynamics_observation` | `std_msgs/msg/Float64MultiArray` | observer -> calibration | Dynamics data |

Cartesian pose commands and LMPC obstacle meshes are expected in the controller root frame, normally `fr3_link0`. No TF transform is applied by these nodes.

The `TwistStamped` numeric components are also interpreted in the controller root-frame convention; the `frame_id` is not transformed.

---

# 7. Low-level joint velocity controller

Launch only the low-level velocity controller:

```bash
ros2 launch low_level control.launch.py \
  robot_ip:=192.16.0.1 \
  mode:=velocity
```

The controller subscribes to:

```text
/fr3/joint_commands
sensor_msgs/msg/JointState
```

The `velocity` field must address all seven joints. If `name` is omitted, exactly seven velocity entries are required in FR3 joint order.

The hardware controller runs at 1 kHz and tracks lower-rate targets with a damped first-order velocity tracker:

```text
a_des = (v_target - v_command) / tracking_time_constant
```

Acceleration and jerk are limited with direction-preserving vector scaling. This avoids sending step changes from a 200 Hz Cartesian controller directly into Franka's 1 kHz velocity interface.

Default low-level values:

```text
tracking_time_constant = 0.03 s
max_acceleration       = [2,2,2,2,2,2,2] rad/s^2
max_jerk               = [100,100,100,100,100,100,100] rad/s^3
command_timeout        = 0.1 s
```

A direct test command can be published continuously. Start with very small values:

```bash
ros2 topic pub -r 50 /fr3/joint_commands sensor_msgs/msg/JointState "{
  name: [fr3_joint1, fr3_joint2, fr3_joint3, fr3_joint4, fr3_joint5, fr3_joint6, fr3_joint7],
  velocity: [0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}"
```

Stop the publisher to allow the watchdog/tracker to return toward zero velocity.

### Important velocity options

| Launch argument | Default | Meaning |
|---|---:|---|
| `tracking_time_constant` | `0.03` | First-order target tracking time constant |
| `max_acceleration` | `[2,...,2]` | 1 kHz joint acceleration limits |
| `max_jerk` | `[100,...,100]` | 1 kHz joint jerk limits |
| `max_velocity` | `[0,...,0]` | Optional final velocity limiter; `0` means disabled per joint |
| `velocity_scale` | `1.0` | Global scaling of received velocity commands |
| `command_timeout` | `0.1` | Command watchdog |

---

# 8. Low-level joint impedance controller

Launch:

```bash
ros2 launch low_level control.launch.py \
  robot_ip:=192.16.0.1 \
  mode:=impedance
```

The impedance controller accepts the same `/fr3/joint_commands` `JointState` topic. Any complete seven-joint `position`, `velocity`, or `effort` field can be used.

The implemented residual torque law is approximately:

```text
tau = K * (q_des - q)
    + D * (dq_des - dq)
    + M(q) * (D_mass .* (dq_des - dq))
    + effort_feedforward_scale * tau_ff
    + tau_friction
```

No separate gravity term is added by this controller. The stack relies on Franka's baseline effort-interface compensation and adds only the commanded residual effort plus the optional calibrated residual-friction feedforward.

Default impedance parameters are deliberately light:

```text
stiffness    = [0,0,0,0,0,0,0]
damping      = [2,2,2,2,2,2,2]
mass_damping = [5,5,5,5,5,5,5]
delta_tau_max = 1.0
```

With zero stiffness, a high-level IK/QP can send desired joint velocity and the impedance controller behaves as a velocity-error damping layer rather than a joint-position spring.

### Main impedance options

| Launch argument | Default | Meaning |
|---|---:|---|
| `stiffness` | `[0,...,0]` | Joint position stiffness |
| `damping` | `[2,...,2]` | Joint velocity-error damping |
| `mass_damping` | `[5,...,5]` | Mass-matrix-shaped velocity damping |
| `effort_feedforward_scale` | `1.0` | Scale on incoming JointState effort field |
| `delta_tau_max` | `1.0` | Maximum per-cycle torque command change |
| `max_torque` | `[0,...,0]` | Optional torque limit; `0` disables limit per joint |
| `hold_position_on_timeout` | `false` | Hold the measured position when command watchdog expires |
| `command_timeout` | `0.1` | Command watchdog |

---

# 9. Automatic friction compensation

Friction compensation is enabled by default in impedance mode:

```text
friction_compensation_enabled:=true
```

The current controller uses a strictly odd residual-friction model:

```text
tau_f(dq) = Fc * tanh(dq / epsilon) + B * dq
```

where:

- `Fc >= 0` is the identified Coulomb magnitude,
- `B >= 0` is the identified viscous coefficient,
- `epsilon` is a fixed low-speed smoothing velocity,
- no fitted constant torque bias is applied.

The strictly odd model intentionally rejects even/configuration-dependent residual torque so gravity, payload or sensor-offset errors are less likely to be misidentified as friction.

## 9.1 Automatic file selection

When impedance mode starts, `control.launch.py` searches for:

```text
friction_calibration_*.yaml
friction_calibration_*.yml
```

in:

```text
<workspace>/data/calibration
```

The newest valid file is selected.

By default it must be younger than:

```text
24 hours
```

If no valid fresh calibration is available, the impedance controller still starts, but friction feedforward is disabled.

The file must use:

```text
model: odd_coulomb_viscous_tanh
```

and the joint names must exactly match the active robot.

## 9.2 Friction options

| Launch argument | Default | Meaning |
|---|---:|---|
| `friction_compensation_enabled` | `true` | Request automatic compensation |
| `friction_calibration_directory` | empty | Empty auto-detects `<workspace>/data/calibration` |
| `friction_calibration_file` | empty | Explicit file override |
| `friction_calibration_max_age_hours` | `24.0` | Maximum age for automatic loading |
| `friction_use_recommended_enable` | `true` | Honor per-joint quality mask |
| `friction_compensation_scale` | `1.0` | Global scale on compensation |
| `friction_apply_torque_bias` | `false` | Deprecated; ignored by the odd model |

Example: force a particular calibration file:

```bash
ros2 launch low_level control.launch.py \
  robot_ip:=192.16.0.1 \
  mode:=impedance \
  friction_calibration_file:=$HOME/franka_ws/data/calibration/friction_calibration_YYYYMMDD_HHMMSS.yaml
```

Disable friction compensation:

```bash
ros2 launch low_level control.launch.py \
  robot_ip:=192.16.0.1 \
  mode:=impedance \
  friction_compensation_enabled:=false
```

Test at half compensation strength:

```bash
ros2 launch low_level control.launch.py \
  robot_ip:=192.16.0.1 \
  mode:=impedance \
  friction_compensation_scale:=0.5
```

The controller clamps the velocity used by the friction model to the per-joint validated calibration range rather than extrapolating indefinitely beyond measured speeds.

---

# 10. Cartesian IK velocity controller

Launch the IK stack with its default low-level velocity controller:

```bash
ros2 launch franka_cartesian_control cartesian_control.launch.py \
  robot_ip:=192.16.0.1 \
  controller:=ik
```

Input:

```text
/fr3/cartesian_twist_command
geometry_msgs/msg/TwistStamped
```

Output:

```text
/fr3/joint_commands
sensor_msgs/msg/JointState
```

The IK node uses damped pseudoinverse kinematics plus configured joint/surface recovery logic.

Example low-speed command:

```bash
ros2 topic pub -r 50 /fr3/cartesian_twist_command geometry_msgs/msg/TwistStamped "{
  header: {frame_id: 'fr3_link0'},
  twist: {
    linear:  {x: 0.02, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.0}
  }
}"
```

Stop the publisher to stop commanding Cartesian motion.

---

# 11. Cartesian QP velocity controller

Launch:

```bash
ros2 launch franka_cartesian_control cartesian_control.launch.py \
  robot_ip:=192.16.0.1 \
  controller:=qp
```

The QP solves for joint velocity while tracking Cartesian twist as a soft least-squares objective subject to hard joint and configured Cartesian constraints.

The same `/fr3/cartesian_twist_command` interface is used as for IK.

### Default Cartesian velocity launch limits

The standalone `cartesian_control.launch.py` launch defaults are:

```text
frequency        = 200 Hz
max_velocity     = [4,4,4,4,3,3,3] rad/s
max_acceleration = [3,3,3,3,3,3,3] rad/s^2
joint_limit_margin = 0.08 rad
```

These launch arguments override the corresponding values from `config/controllers.yaml`.

### QP objective defaults from `config/controllers.yaml`

```text
task_weights    = [1,1,1,0.30,0.30,0.30]
regularization  = 0.0001
posture_weight  = 0.02
posture_gain    = 0.50
posture_target  = [0,0,0,-1.5708,0,1.5708,0.7854]
```

### Surface constraints

Surface half spaces use:

```text
normal . tcp_position >= offset + margin
```

The default table configuration is:

```text
normal = [0,0,1]
offset = 0.0
margin = 0.08
```

therefore the safe side is:

```text
z >= 0.08 m
```

The QP includes soft recovery behavior after a configured joint or surface boundary is violated. `repulsion_when_idle:=true` allows recovery to wake the QP without an external twist command after a joint/surface violation or when a self-collision capsule pair has moved inside its safety margin.

### QP self-collision constraints

Self-collision avoidance is enabled by default for the QP:

```text
self_collision_enabled = true
self_collision_margin = 0.035 m
self_collision_gain = 3.0 1/s
self_collision_release_distance = 0.015 m
self_collision_minimum_segment_gap = 2
self_collision_segment_radii = [0.10,0.09,0.09,0.085,0.08,0.075,0.07,0.07] m
```

The eight FCL capsules connect:

```text
link0-link1
link1-link2
...
link7-link8
```

Pairs separated by `minimum_segment_gap` or fewer center-line segments are ignored so ordinary neighbouring-link geometry does not constrain the QP. Every remaining pair contributes a hard velocity inequality based on its current capsule clearance and distance Jacobian.

Disable the feature for debugging:

```bash
ros2 launch franka_cartesian_control cartesian_control.launch.py \
  controller:=qp \
  self_collision_enabled:=false
```

Increase `self_collision_margin` for more conservative separation.  Do not reduce the margin or capsule radii merely to make a previously infeasible pose reachable without first checking the physical geometry.

The capsule model is deliberately simple. It protects against the common arm-folding self-collision modes without requiring MoveIt, but it is not an exact collision model for the hand, fingers, payload or arbitrary tools.

---

# 12. Using the Cartesian controllers with low-level impedance

Both IK and QP can feed the low-level impedance controller instead of the velocity interface.

Example QP + impedance:

```bash
ros2 launch franka_cartesian_control cartesian_control.launch.py \
  robot_ip:=192.16.0.1 \
  controller:=qp \
  low_level_mode:=impedance
```

With a fresh friction calibration, residual-friction feedforward will be loaded automatically because impedance friction compensation is enabled by default.

Useful propagated options include:

```text
impedance_stiffness
impedance_damping
impedance_mass_damping
impedance_delta_tau_max
impedance_max_torque
impedance_hold_position_on_timeout
impedance_friction_compensation_enabled
impedance_friction_calibration_directory
impedance_friction_calibration_file
impedance_friction_calibration_max_age_hours
impedance_friction_use_recommended_enable
impedance_friction_compensation_scale
```

---

# 13. Cartesian PID position controller

The position stack adds a pose-to-twist controller above IK or QP.

Recommended default combination:

```bash
ros2 launch franka_cartesian_control position_control.launch.py \
  robot_ip:=192.16.0.1 \
  position_controller:=pid \
  velocity_controller:=qp
```

Use IK instead of QP:

```bash
ros2 launch franka_cartesian_control position_control.launch.py \
  robot_ip:=192.16.0.1 \
  position_controller:=pid \
  velocity_controller:=ik
```

Input pose:

```text
/fr3/cartesian_pose_command
geometry_msgs/msg/PoseStamped
```

Current pose output:

```text
/fr3/cartesian_pose
geometry_msgs/msg/PoseStamped
```

The pose command watchdog defaults to `0.50 s`. Continue publishing the target if you want the position layer to keep commanding.

The launch defaults for PID are:

```text
frequency = 50 Hz
kp        = [2,2,2,2,2,2]
ki        = [0,0,0,0,0,0]
kd        = [0,0,0,0,0,0]
max_twist = [0.5,0.5,0.5,0.20,0.20,0.20]
```

Example workflow:

```bash
ros2 topic echo --once /fr3/cartesian_pose
```

Use the reported pose as the starting point, modify it conservatively, then publish the desired target in `fr3_link0`:

```bash
ros2 topic pub -r 10 /fr3/cartesian_pose_command geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'fr3_link0'},
  pose: {
    position: {x: YOUR_X, y: YOUR_Y, z: YOUR_Z},
    orientation: {x: YOUR_QX, y: YOUR_QY, z: YOUR_QZ, w: YOUR_QW}
  }
}"
```

No TF conversion is performed if another frame is supplied.

---

# 14. Cartesian LMPC position controller

Launch LMPC over QP:

```bash
ros2 launch franka_cartesian_control position_control.launch.py \
  robot_ip:=192.16.0.1 \
  position_controller:=lmpc \
  velocity_controller:=qp
```

LMPC over IK is also supported:

```bash
ros2 launch franka_cartesian_control position_control.launch.py \
  robot_ip:=192.16.0.1 \
  position_controller:=lmpc \
  velocity_controller:=ik
```

The LMPC state model is a first-order TCP velocity-tracking model:

```text
v[k+1] = alpha * v[k] + (1-alpha) * u[k]
alpha  = exp(-dt / tau)
```

where the decision variable `u` is the desired Cartesian twist sent to the downstream QP/IK controller.

Important launch defaults are:

```text
position_frequency                  = 50 Hz
lmpc_regularization_gain           = 0.00001
lmpc_delta_twist_weight            = 0.3
lmpc_velocity_tracking_time_constant = 0.05 s
lmpc_max_twist                     = [8,8,8,1,1,1]
lmpc_max_acceleration              = [16,16,16,8,8,8]
lmpc_velocity_weights              = [0.5,0.5,0.5,0.5,0.5,0.5]
lmpc_terminal_velocity_weights     = [50,50,50,50,50,50]
```

Other LMPC parameters, including horizon and state weights, come from `config/position_controllers.yaml`. The current configured horizon is `25`.

### LMPC obstacle input

Runtime obstacles are accepted on:

```text
/fr3/lmpc_obstacle_mesh
visualization_msgs/msg/Marker
```

Requirements:

- marker type must be `TRIANGLE_LIST`,
- point count must be a non-zero multiple of three,
- marker coordinates must already be expressed in the LMPC root frame,
- no TF transform is applied,
- runtime meshes are treated as two-sided obstacles.

A one-sided default table is enabled in the configuration and uses the `+Z` side as valid.

---

# 15. Position control with low-level impedance and friction compensation

Example PID -> QP -> impedance:

```bash
ros2 launch franka_cartesian_control position_control.launch.py \
  robot_ip:=192.16.0.1 \
  position_controller:=pid \
  velocity_controller:=qp \
  low_level_mode:=impedance
```

Example LMPC -> QP -> impedance:

```bash
ros2 launch franka_cartesian_control position_control.launch.py \
  robot_ip:=192.16.0.1 \
  position_controller:=lmpc \
  velocity_controller:=qp \
  low_level_mode:=impedance
```

The position launch propagates the impedance-friction options using the `impedance_...` argument names.

---

# 16. Gripper control

The gripper node is started automatically by `low_level/control.launch.py` when:

```text
load_gripper:=true
```

Default command topic:

```text
/fr3/gripper
std_msgs/msg/Float32
```

The value is a ratio in `[0,1]`:

```text
0.0 -> 0 m width, closed
1.0 -> max_width, normally 0.08 m
```

Open:

```bash
ros2 topic pub --once /fr3/gripper std_msgs/msg/Float32 "{data: 1.0}"
```

Close:

```bash
ros2 topic pub --once /fr3/gripper std_msgs/msg/Float32 "{data: 0.0}"
```

When the commanded ratio decreases and `gripper_close_uses_grasp:=true`, the node uses the Franka `Grasp` action. Opening uses the `Move` action.

Main options:

```text
gripper_max_width        = 0.08 m
gripper_speed            = 0.1 m/s
gripper_force            = 40 N
gripper_epsilon_inner    = 0.005 m
gripper_epsilon_outer    = 0.005 m
gripper_command_deadband = 0.001
gripper_close_uses_grasp = true
```

---

# 17. Dynamics observer

`low_level/DynamicsObserverController` is a read-only ros2_control controller used primarily by calibration.

It publishes:

```text
/fr3/dynamics_observation
std_msgs/msg/Float64MultiArray
```

Default publication rate:

```text
200 Hz
```

The 112-element payload contains:

```text
q[7]
dq[7]
filtered ddq[7]
measured tau[7]
M(q) * ddq[7]
coriolis[7]
gravity[7]
model tau = Mddq + c + g[7]
residual tau[7]
full 7x7 mass matrix[49], column-major
```

The friction-identification launch starts this observer automatically.

To spawn it manually after a compatible Franka bringup is already running:

```bash
ros2 run controller_manager spawner dynamics_observer_controller \
  --controller-manager /controller_manager
```

---

# 18. Franka Desk torque-sensor calibration

Run Franka's native torque-sensor calibration **before** identifying residual drivetrain friction if the torque offsets may have drifted.

The automatic routine is available in modern FR3 system images; Franka introduced the field torque-calibration feature in System Image 5.8.

## 18.1 When to run it

Typical reasons include:

- hand-guiding feels unbalanced,
- a joint appears to push or pull with no intended force,
- force/torque threshold violations occur unexpectedly,
- the robot has experienced significant impact or wear,
- Franka support requests recalibration.

Do not run it routinely just because a friction calibration is old. It is a sensor-offset calibration, not the same thing as this repository's friction identification.

## 18.2 Physical preparation

The official procedure requires the arm to be mounted on a **fixed, horizontal, flat surface** and positioned upright/level. Vibrations or mounting tilt can degrade the result.

Before starting:

1. Clear the robot's complete workspace. The routine moves through a large portion of the workspace.
2. Make sure the mounting surface cannot vibrate during calibration.
3. Remove the end effector from the robot arm as required by the Franka procedure.
4. Select the Franka Desk end-effector profile **No End Effector**.
5. Make sure the user performing calibration has the required Admin privileges.
6. Claim control of the arm.
7. Unlock the joints.
8. Set operating mode to **Execution**.
9. Make sure no task is running and the Watchman/system configuration is valid.

Franka notes that even small mounting-angle errors can affect calibration.

## 18.3 Run the Desk procedure

In Franka Desk:

```text
Settings -> Calibration -> Joint Torque Calibration
```

Then follow the four guided stages:

```text
1. Prepare Arm
2. Prepare system
3. Run procedure
4. Confirm values
```

During the automatic routine:

- stay outside the robot workspace,
- do not touch the robot,
- do not obstruct its path,
- do not cause vibrations in the mounting surface,
- a yellow status indication while passing through a singular configuration can be expected.

The procedure can take several minutes.

At the end, review the newly calculated offsets and choose the Desk action to apply/calibrate the torque sensors. Franka's System Image release instructions specify applying the values and then power-cycling the arm.

After the calibration:

1. Power-cycle the robot if requested by the Desk/system-image procedure.
2. Reinstall the normal end effector.
3. Restore the correct end-effector profile.
4. Restore the correct payload configuration.
5. Verify hand-guiding behavior before running this control stack.

If behavior is worse, repeat the official calibration or revert to factory torque offsets in Desk.

---

# 19. Residual-friction calibration

This repository does **not** replace Franka's rigid-body dynamics model. The current calibration keeps Franka's:

```text
M(q)
C(q,dq)
g(q)
```

fixed and identifies only residual joint friction.

The final fitted controller model is strictly odd:

```text
tau_f(dq) = Fc * tanh(dq / epsilon) + B * dq
```

The identification uses matched `+v/-v` motion and position-matched odd torque components to reject much of the configuration-dependent/even residual torque.

## 19.1 Recommended order

Use this sequence:

```text
1. Verify robot mounting and payload configuration
2. Run Franka Desk torque-sensor calibration if needed
3. Restore the normal end-effector/payload configuration
4. Reboot / power-cycle as required by Franka
5. Start FCI
6. Run the friction identification below
7. Inspect the generated quality mask
8. Run impedance control with automatic friction loading
```

## 19.2 Run friction identification

The critical command is:

```bash
ros2 launch low_level dynamics_identification.launch.py \
  robot_ip:=192.16.0.1 \
  run_motion:=true \
  motion_mode:=joint
```

### `run_motion:=true` is mandatory

For safety, the current launch default is:

```text
run_motion:=false
```

If you omit `run_motion:=true`, the script runs in **record-only** mode. The robot will not execute the friction sweep, there will be no usable plateau samples, and no controller-ready calibration YAML will be generated.

A record-only launch is useful only for checking the observer/data path:

```bash
ros2 launch low_level dynamics_identification.launch.py \
  robot_ip:=192.16.0.1 \
  run_motion:=false
```

## 19.3 Current friction-sweep defaults

```text
motion_mode                    = joint
friction_repeats               = 4
friction_ramp_time             = 0.30 s
friction_hold_time             = 0.35 s
friction_pause_time            = 0.15 s
friction_plateau_trim_time     = 0.10 s
friction_max_fit_acceleration  = 0.05 rad/s^2
friction_pair_grid_points      = 15
friction_min_pair_samples      = 6
friction_smoothing_velocity    = 0.01 rad/s
joint_max_joint_displacement   = 0.20 rad
```

The sweep uses several positive and negative steady speeds and alternating repeat order to expose drift/order dependence.

The experiment aborts when its safety guards are exceeded.

## 19.4 Output files

The default output directory is:

```text
~/franka_ws/data/calibration/
```

A successful run writes:

```text
friction_identification_YYYYMMDD_HHMMSS.csv
friction_identification_YYYYMMDD_HHMMSS.json
friction_calibration_YYYYMMDD_HHMMSS.yaml
```

The CSV contains the recorded dynamics data.

The JSON contains the complete fitting diagnostics, quality metrics and repeatability information.

The YAML is the compact file consumed automatically by the low-level impedance controller.

Override the directory if needed:

```bash
ros2 launch low_level dynamics_identification.launch.py \
  robot_ip:=192.16.0.1 \
  run_motion:=true \
  motion_mode:=joint \
  output_directory:=$HOME/franka_ws/data/calibration/test_01
```

## 19.5 Calibration quality mask

The YAML contains per-joint quality and:

```text
recommended_enable
```

By default the impedance launch honors that mask. A poor joint is left uncompensated rather than forcing a questionable friction estimate into the effort controller.

You can ignore the mask deliberately with:

```text
friction_use_recommended_enable:=false
```

but that should normally be avoided.

## 19.6 Freshness rule

A calibration is considered automatically usable only when it is newer than the configured maximum age, default:

```text
24 hours
```

The launch uses the embedded generation timestamp when available and otherwise falls back to file modification time.

This prevents an old calibration from being silently reused after conditions have changed.

---

# 20. Legacy QP excitation mode in the identification launch

`dynamics_identification.launch.py` still accepts:

```text
motion_mode:=qp
```

This drives a larger Cartesian multi-sine excitation through the existing QP and is useful for dynamics observation/diagnostics.

It is **not** the recommended friction-calibration path. Use:

```text
motion_mode:=joint
```

for the controller friction YAML.

When the identification launch runs without the gripper, it automatically uses `fr3_link8` as the QP tip. With `load_gripper:=true`, it uses `fr3_hand_tcp`. An explicit `qp_tip_link:=...` override is also available.

---

# 21. Main launch options

## 21.1 `low_level/control.launch.py`

| Argument | Default |
|---|---|
| `mode` | `impedance` |
| `robot_ip` | `192.16.0.1` |
| `robot_type` | `fr3` |
| `load_gripper` | `true` |
| `use_fake_hardware` | `false` |
| `load_franka_robot_state_broadcaster` | `false` |
| `joint_state_rate` | `250` |
| `command_topic` | `/fr3/joint_commands` |
| `command_timeout` | `0.1` |
| `tracking_time_constant` | `0.03` |
| `friction_compensation_enabled` | `true` |
| `friction_calibration_max_age_hours` | `24.0` |
| `configure_collision_behavior` | `false` |

The optional 1 kHz Franka full-state broadcaster is disabled by default to leave more real-time headroom.

## 21.2 `franka_cartesian_control/cartesian_control.launch.py`

| Argument | Default |
|---|---|
| `controller` | `qp` |
| `start_low_level` | `true` |
| `low_level_mode` | `velocity` |
| `frequency` | `200.0` |
| `max_velocity` | `[4,4,4,4,3,3,3]` |
| `max_acceleration` | `[3,3,3,3,3,3,3]` |
| `joint_limit_margin` | `0.08` |
| `self_collision_enabled` | `true` |
| `self_collision_margin` | `0.035` m |
| `self_collision_gain` | `3.0` 1/s |
| `self_collision_release_distance` | `0.015` m |
| `self_collision_minimum_segment_gap` | `2` |
| `self_collision_segment_radii` | `[0.10,0.09,0.09,0.085,0.08,0.075,0.07,0.07]` m |
| `repulsion_when_idle` | `true` |
| `idle_safety_frequency` | `50.0` |
| `twist_topic` | `/fr3/cartesian_twist_command` |

## 21.3 `franka_cartesian_control/position_control.launch.py`

| Argument | Default |
|---|---|
| `position_controller` | `pid` |
| `velocity_controller` | `qp` |
| `start_low_level` | `true` |
| `low_level_mode` | `velocity` |
| `position_frequency` | `50.0` |
| `pose_watchdog_timeout` | `0.50` |
| `max_velocity` | `[1,1,1,1,1.5,1.5,1.5]` |
| `max_acceleration` | `[1.5,...,1.5]` |
| `desired_pose_topic` | `/fr3/cartesian_pose_command` |
| `twist_topic` | `/fr3/cartesian_twist_command` |
| `current_pose_topic` | `/fr3/cartesian_pose` |

## 21.4 `low_level/dynamics_identification.launch.py`

| Argument | Default |
|---|---|
| `motion_mode` | `joint` |
| `run_motion` | `false` |
| `friction_repeats` | `4` |
| `friction_ramp_time` | `0.30` |
| `friction_hold_time` | `0.35` |
| `friction_plateau_trim_time` | `0.10` |
| `friction_max_fit_acceleration` | `0.05` |
| `friction_smoothing_velocity` | `0.01` |
| `observer_publish_rate` | `200.0` |
| `acceleration_filter_tau` | `0.02` |
| `load_gripper` | `false` |

---

# 22. Collision-behavior configuration

Enable collision configuration during low-level startup:

```bash
ros2 launch low_level control.launch.py \
  robot_ip:=192.16.0.1 \
  mode:=impedance \
  configure_collision_behavior:=true
```

The launch waits until Franka bringup is ready, configures collision behavior while the command controller is still idle, then activates the low-level controller.

The main scale arguments are:

```text
collision_torque_scale
collision_force_scale
```

Both default to `1.0`.

Default threshold arrays are also individually configurable through the launch file.

---

# 23. Using an already-running low-level stack

The Cartesian launch files normally start `low_level` themselves.

If low-level bringup is already running, disable the nested bringup:

```bash
ros2 launch franka_cartesian_control cartesian_control.launch.py \
  controller:=qp \
  start_low_level:=false
```

or:

```bash
ros2 launch franka_cartesian_control position_control.launch.py \
  position_controller:=pid \
  velocity_controller:=qp \
  start_low_level:=false
```

Make sure the expected `/franka/joint_states`, `/fr3/joint_commands` and low-level ready topic already exist.

---

# 24. Fake hardware

The low-level launch accepts:

```text
use_fake_hardware:=true
```

Example:

```bash
ros2 launch low_level control.launch.py \
  use_fake_hardware:=true \
  mode:=velocity
```

The mock hardware does not export Franka's model semantic interface, so mass-matrix damping is disabled automatically in fake-hardware mode.

Use fake hardware for software/interface checks, not for validating real torque/friction behavior.

---

# 25. Useful ROS 2 inspection commands

List controllers:

```bash
ros2 control list_controllers
```

Check joint states:

```bash
ros2 topic echo /franka/joint_states
```

Check low-level readiness:

```bash
ros2 topic echo /low_level_joint_velocity_controller/ready
```

or:

```bash
ros2 topic echo /low_level_joint_impedance_controller/ready
```

Check QP readiness:

```bash
ros2 topic echo /qp_velocity_controller/ready
```

Check IK readiness:

```bash
ros2 topic echo /ik_velocity_controller/ready
```

Check the current Cartesian pose when PID/LMPC is running:

```bash
ros2 topic echo /fr3/cartesian_pose
```

Inspect topic QoS and publishers/subscribers:

```bash
ros2 topic info -v /fr3/joint_commands
ros2 topic info -v /fr3/cartesian_twist_command
```

---

# 26. Recommended controller combinations

For simple Cartesian velocity control:

```text
IK -> low-level velocity
```

For constrained Cartesian velocity control:

```text
QP -> low-level velocity
```

For constrained velocity control with residual-friction compensation:

```text
QP -> low-level impedance
```

For general Cartesian point-to-point position control:

```text
PID -> QP -> low-level velocity
```

For position control where low-level residual-friction compensation is desired:

```text
PID -> QP -> low-level impedance
```

For predictive Cartesian position control / obstacle experiments:

```text
LMPC -> QP -> low-level velocity or impedance
```

---

# 27. UDP <-> ROS 2 bridge

The `udp_bridge` package provides two generic nodes driven by JSON configuration files in:

```text
~/franka_ws/data/udp/
    udp_reader.json
    udp_publisher.json
```

No message-specific bridge code is required for common ROS 2 message packages.  The bridge intentionally limits dynamic message loading to standard packages such as `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `trajectory_msgs`, `shape_msgs`, `visualization_msgs`, `diagnostic_msgs` and `builtin_interfaces`.

## 27.1 UDP -> ROS 2

`udp_reader.json` defines one or more receiving sockets. Each endpoint specifies:

```text
bind_ip / bind_port
optional source_ip / source_port filter
socket receive-buffer size
binary or JSON payload encoding
field layout
ROS topic
ROS message type
QoS
default ROS fields
optional automatic header timestamp
```

The included binary example receives six little-endian `float64` values on UDP port `15000`:

```text
vx vy vz wx wy wz
```

and publishes them as:

```text
/fr3/cartesian_twist_command
geometry_msgs/msg/TwistStamped
```

A binary field mapping looks like:

```json
{"name":"vx", "type":"float64", "ros_field":"twist.linear.x"}
```

Supported binary scalar types are:

```text
int8 uint8 int16 uint16 int32 uint32 int64 uint64
float32 float64 bool padding
```

Use `count` for fixed-size arrays. `endianness` can be `little`, `big` or `network`.

For variable/structured traffic, use JSON encoding and map a JSON path into a ROS field.

## 27.2 ROS 2 -> UDP

`udp_publisher.json` performs the reverse mapping. Each endpoint specifies the subscribed ROS topic/type, destination address/port, optional local bind address/port, socket options and binary/JSON field mapping.

The included example subscribes to:

```text
/franka/joint_states
sensor_msgs/msg/JointState
```

and sends 14 little-endian doubles:

```text
q[0..6], dq[0..6]
```

to the configured UDP destination.

## 27.3 Run the bridge

Build it with the rest of the workspace, then:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ws/install/setup.bash

ros2 launch udp_bridge udp_bridge.launch.py
```

Reader only:

```bash
ros2 launch udp_bridge udp_bridge.launch.py \
  enable_reader:=true \
  enable_publisher:=false
```

Publisher only:

```bash
ros2 launch udp_bridge udp_bridge.launch.py \
  enable_reader:=false \
  enable_publisher:=true
```

Override either configuration file:

```bash
ros2 launch udp_bridge udp_bridge.launch.py \
  reader_config:=$HOME/franka_ws/data/udp/udp_reader.json \
  publisher_config:=$HOME/franka_ws/data/udp/udp_publisher.json
```

---

# 28. Rebuilding after code changes

Clean only the three custom packages:

```bash
cd ~/franka_ws
rm -rf \
  build/low_level install/low_level \
  build/franka_cartesian_control install/franka_cartesian_control
```

Rebuild:

```bash
source /opt/ros/jazzy/setup.bash
source ~/franka_ws/install/setup.bash

colcon build \
  --packages-select low_level franka_cartesian_control udp_bridge \
  --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DQPOASES_ROOT=$HOME/franka_ws/qpoases_install
```

Then:

```bash
source ~/franka_ws/install/setup.bash
```

---

# 29. Configuration files

Main files:

```text
low_level/config/controllers.yaml
franka_cartesian_control/config/controllers.yaml
franka_cartesian_control/config/position_controllers.yaml
data/udp/udp_reader.json
data/udp/udp_publisher.json
```

Remember that launch arguments passed as node parameters override values from the YAML configuration where both define the same parameter.

This is particularly relevant for:

- Cartesian max joint velocity/acceleration,
- PID gains and max twist,
- several LMPC tuning values.

Use the effective launch arguments shown by your command line as the final source of truth.

---

# 30. External references

Official Franka resources:

- Franka ROS 2 repository: https://github.com/frankarobotics/franka_ros2
- Franka Control Interface documentation: https://frankarobotics.github.io/docs/
- Franka product documentation / current operating manuals: https://franka.de/documents
- Franka Research 3 releases: https://franka.de/products/franka-research-3/releases

qpOASES:

- https://github.com/coin-or/qpOASES

FCL and Wireshark:

- https://github.com/flexible-collision-library/fcl
- https://www.wireshark.org/docs/wsug_html/
- https://www.wireshark.org/docs/wsdg_html_chunked/wsluarm_modules.html

For torque-sensor calibration, always follow the operating manual corresponding to the System Image installed on the robot. The exact Desk labels can change between system-image versions.

---

# 31. Package license note

`franka_cartesian_control/package.xml` currently declares Apache-2.0.
