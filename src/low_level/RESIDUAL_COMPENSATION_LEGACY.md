# ARCHIVED: for historical reference only; use GRAVITY_BREAKAWAY_GUIDE.md for current parameters.

# Experimental stationary residual-torque calibration (FR3)

This modification is **OFF by default** and is **not validated on an actual robot**.
Do not treat the compensation as a safety feature or an external-contact detector.

## Purpose and limits

The effort command is an *additional* torque: Franka already provides gravity
and motor-friction compensation. This feature takes samples of

    r_i = tau_J_measured_i - g_model_i(q)

only when the operator explicitly authorizes a *known unloaded* configuration,
all 7 command velocities are below epsilon, all measured joint velocities are
below epsilon for a dwell period, all previously commanded torques are close to
zero, and every candidate residual is within a small bound.

The controller **does not** automatically infer lack of human contact from zero
velocity: a person can exert force against a stationary robot. Nor can one
stationary sample distinguish gravity-model errors from torque offsets and
static-friction reactions. The estimated bias is therefore experimental and
configuration-local, and its sign must be checked on hardware. For a robust
whole-workspace gravity correction, identify/update payload mass and center of
mass instead. This is NOT an online gravity parameter estimator.

Sampling never changes the applied bias. When the operator stops authorizing
calibration, a sufficiently long, fully stationary sample is committed. The
correction is then applied with a slow slew and is frozen while moving. It is
faded away when the arm moves farther than `residual_pose_radius` from the
calibration pose, reaching zero at radius + `residual_pose_fade_width`.
The feature resets when the controller is activated and has no persistent
calibration cache.

## Enable and test (operator supervised)

1. First verify the normal controller has zero stiffness, zero damping, zero
   mass damping, no unintended effort feedforward, correctly configured tool
   and payload, proper contact/collision safeguards, an accessible stop, and
   a safe workspace. Check that the system's 1 kHz loop is stable. An arm
   should be supported if it could drift or move unexpectedly.
2. Launch the impedance controller with the experimental feature enabled,
   leaving the starting torque limit small:

       ros2 launch low_level control.launch.py mode:=impedance residual_compensation_enabled:=true

3. Keep publishing fresh `JointState` commands on the configured joint command
   topic with **all seven velocity values exactly zero**. A velocity array is
   required (a position-only command is deliberately NOT an authorization to
   learn). Effort commands must be zero. Command updates must remain faster
   than `command_timeout` (0.1 s by default). Do NOT touch the robot during
   calibration. Keep it motionless with appropriate safety precautions.
4. In a separate terminal, begin the explicit calibration authorization:

       ros2 topic pub -r 10 /low_level_joint_impedance_controller/residual_calibration_enable std_msgs/msg/Bool '{data: true}'

   This is an operator consent signal, NOT a robot-derived proof of no contact.
   Keep it running for at least 0.5 s dwell + 1.0 s samples + a margin.
   Stop the publisher (Ctrl+C) while the robot remains stationary to commit
   a valid sample after the 0.35-second authorization lease expires.
5. Inspect the 10 Hz diagnostics:

       ros2 topic echo /low_level_joint_impedance_controller/residual_state

   `data[0:7]`: estimated correction [Nm]. `data[7:14]`: correction currently
   being applied [Nm]. `data[14]`: 1 when a valid calibration exists.
   `data[15]`: 1 while taking qualifying samples. No compensation is applied
   unless data[14] becomes 1. Verify torque sign and absence of unintended
   motion before attempting physical guidance.
6. After calibration, carefully test manual movements with assistance and a
   clear stop method. Corrections should stay fixed while moving, and fade to
   zero outside the configuration-local validity radius.

   The fully-qualified topic name depends on your controller namespace. If
   namespaced, adjust the paths above accordingly.

## Parameters

- `residual_compensation_enabled: false` — opt-in experimental feature.
- `residual_command_velocity_epsilon: 0.002` rad/s (all seven commands).
- `residual_measured_velocity_epsilon: 0.003` rad/s (all seven measured values).
- `residual_stationary_dwell_s: 0.5` s.
- `residual_minimum_sample_s: 1.0` s after dwell.
- `residual_filter_tau_s: 2.0` s (EWMA).
- `residual_sampling_command_torque_epsilon: 0.03` Nm — commanded-torque guard.
- `residual_sample_deviation_limit: 0.10` Nm — reject strongly changing residual.
- `residual_max_torque: [0.15]*7` Nm — limit per joint, samples above rejected.
- `residual_output_slew_rate: 0.10` Nm/s — smooth enabling/disabling.
- `residual_pose_radius: 0.25` rad and `residual_pose_fade_width: 0.25` rad.
- `residual_calibration_lease_s: 0.35` s — repeated authorization required.

These are conservative starting settings for bench validation, NOT tuned or
certified FR3 values. The existing overall `max_torque` and `delta_tau_max`
limit the sum of friction, bias and other effort requests as before. The
calibration will refuse samples if any joint exceeds its residual torque
limit. With no calibration signal or with nonzero commanded velocity, the
estimator will not update. During any command timeout, the applied residual
ramps toward zero (bounded by `residual_output_slew_rate`).

## Current implementation caveats

- **No automatic human-contact detection.** A stationary person exerting an
  external wrench below the configured torque limit can contaminate the
  sample. Explicit unloaded calibration remains mandatory.
- **Not a friction breakaway compensator.** At zero measured velocity, the
  existing odd Stribeck feedforward is also zero.
- **Not global gravity compensation.** A locally measured constant torque can
  become invalid at another pose; a payload model is preferable.
- **No automatic bias adaptation while moving.** This avoids learning from
  external torques, but does not create a full adaptive transparency loop.
- Verify the sign of the residual using a supervised test. A torque estimate
  alone cannot prove the bias will reduce the force needed for guidance.

## Smoke test without ROS

    g++ -std=c++17 -Wall -Wextra -Werror -I include \
        tests/test_residual_torque_compensator.cpp -o /tmp/test_residual
    /tmp/test_residual

The source was edited and the pure C++ estimator smoke test executed, but a
full ROS 2 / Franka build and hardware test must be performed in the target
workspace (`colcon build --packages-select low_level` then controlled bench
verification). Do not enable this code on a running arm without those steps.
