# Hand-guiding changes (experimental)

## Friction and timeout
Calibrated Stribeck friction compensation now remains active even if the ROS JointState command stream stops. `command_timeout` still expires ordinary position, velocity and effort commands, and the optional position-hold behavior is unchanged. Disable the controller to stop all active assistance; a ROS command timeout is **not** an emergency stop. Do not enable auto-friction assistance without a valid calibration.

Do **not** set `friction_smoothing_velocity: 0.0`. The controller requires it to be >0 and evaluates tanh(dq/epsilon). Try 0.01 -> 0.005 -> 0.003 rad/s incrementally after inspecting velocity noise and rate-limited torque, with a safe, supervised procedure. No one value guarantees better transparency.

## Payload correction comes first
The correct global gravity adjustment is a verified robot payload configuration (mass, flange-to-center-of-mass and inertia), **not** an online static torque bias. `franka::Robot::setLoad(mass, F_x_Cload, inertia)` is for configured external payloads, not for setting the end-effector parameters that belong in the robot's administrator interface. Run payload configuration through the robot's documented configuration/service interface **outside an active controller update**, with truthful payload parameters; do not repeatedly call setLoad in the real-time loop. This archive deliberately does not invent payload mass, center of mass or inertia, and does not automatically call `setLoad`.

The renamed `gravity_error_*` option retains the preceding bounded LOCAL residual calibration. It is **not** a full gravity-error identifier: the observed tau_measured - model_gravity includes static friction and potential human/external torque. Use only during an expressly authorized, externally unloaded calibration and verify the correction sign with low amplitude on a secured robot. It fades when the pose moves away from calibration. It does not replace setLoad.

Local gravity-error calibration topic: `~/gravity_error_calibration_enable` (repeated `std_msgs/Bool` true messages during controlled, unloaded stationary calibration; stop sending to end lease). Diagnostics: `~/gravity_error_state` (same 16-element layout as the previous residual diagnostic).

Parameters: `gravity_error_compensation_enabled` (default false), `gravity_error_*` (renamed from old `residual_*`).

## Breakaway
`breakaway_enabled` is **false by default**. When explicitly enabled, the prototype computes a small, filtered, slew-limited positive assistance near zero measured velocity from an estimated joint torque residual:

  measured_joint_effort - model_gravity - previous_output_torque

This **is not** a reliable measurement of a person's intended motion and can include static friction, bias, and dynamics. It can destabilize physical interaction. Before enabling on hardware, compare against robot-native `tau_ext_hat_filtered`/contact states, test the sign in a simulation or supervised rig, and add/pass a passivity or energy-tank evaluation. Do not use it for unsupervised hand guidance. Recommended start: leave disabled.

Breakaway parameters (declared in controller and launch):
- `breakaway_velocity_epsilon: 0.004` rad/s
- `breakaway_external_torque_deadband: 0.6` Nm
- `breakaway_gain: 0.10`
- `breakaway_max_torque: 0.08` Nm per joint
- `breakaway_slew_rate: 0.10` Nm/s

Zero stiffness/damping and verified collision protection remain prerequisites for supervised free-motion tests. Keep external torque/power/rate monitoring active. Do not use this prototype for contact-rich production use.

## Checks performed
- Python launch syntax compile
- Config YAML parse
- Existing standalone residual-estimator C++ tests

**Not completed:** full ROS 2 `colcon build`, robot or simulated hardware-in-the-loop validation, load-parameter configuration, dynamic/passivity testing.
