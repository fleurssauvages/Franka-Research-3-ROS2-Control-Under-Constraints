import ast
import glob
import json
import math
import os
from pathlib import Path
import tempfile
import time

import yaml

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.logging import get_logger


_LOG = get_logger("low_level.control")


def _as_bool(value: str) -> bool:
    return value.strip().lower() in ("1", "true", "yes", "on")


def _array_n(value: str, name: str, size: int):
    parsed = ast.literal_eval(value)
    if not isinstance(parsed, (list, tuple)) or len(parsed) != size:
        raise RuntimeError(f"{name} must be a list of exactly {size} numbers")
    return [float(x) for x in parsed]


def _array7(value: str, name: str):
    return _array_n(value, name, 7)


def _array6(value: str, name: str):
    return _array_n(value, name, 6)


def _yaml_array(values):
    return "[" + ", ".join(repr(float(x)) for x in values) + "]"


def _scaled(values, scale):
    return [float(scale) * x for x in values]


def _default_calibration_directory() -> str:
    try:
        prefix = Path(get_package_prefix("low_level"))
        # Standard colcon layout: <workspace>/install/low_level.
        if prefix.parent.name == "install":
            return str(prefix.parent.parent / "data" / "calibration")
        parts = prefix.parts
        install_indices = [i for i, part in enumerate(parts) if part == "install"]
        if install_indices:
            return str(Path(*parts[:install_indices[-1]]) / "data" / "calibration")
    except Exception:
        pass
    return str(Path.cwd() / "data" / "calibration")


def _float_array(value, name, size=7):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{name} must contain exactly {size} values")
    result = [float(x) for x in value]
    if not all(math.isfinite(x) for x in result):
        raise ValueError(f"{name} contains a non-finite value")
    return result


def _parse_friction_calibration(path: str, expected_joint_names, require_stribeck: bool):
    with open(path, "r", encoding="utf-8") as f:
        root = yaml.safe_load(f)
    if not isinstance(root, dict) or not isinstance(root.get("friction_calibration"), dict):
        raise ValueError("missing friction_calibration mapping")
    cal = root["friction_calibration"]
    model = str(cal.get("model", ""))
    expected_model = (
        "odd_stribeck_viscous_tanh" if require_stribeck else "odd_coulomb_viscous_tanh"
    )
    if model != expected_model:
        raise ValueError(f"model '{model}' is not the selected '{expected_model}'")
    if require_stribeck and int(cal.get("schema_version", 0)) != 4:
        raise ValueError("Stribeck calibration requires schema_version 4")
    joint_names = [str(x) for x in cal.get("joint_names", [])]
    if joint_names != expected_joint_names:
        raise ValueError(
            f"joint_names mismatch: expected {expected_joint_names}, got {joint_names}"
        )
    smoothing = float(cal.get("smoothing_velocity_rad_s", 0.0))
    if not math.isfinite(smoothing) or smoothing <= 0.0:
        raise ValueError("smoothing_velocity_rad_s must be > 0")
    coulomb = _float_array(cal.get("coulomb_Nm", []), "coulomb_Nm")
    viscous = _float_array(cal.get("viscous_Nm_per_rad_s", []), "viscous_Nm_per_rad_s")
    velocity_min = _float_array(
        cal.get("validated_velocity_min_rad_s", []),
        "validated_velocity_min_rad_s",
    )
    velocity_max = _float_array(
        cal.get("validated_velocity_max_rad_s", []),
        "validated_velocity_max_rad_s",
    )
    if any(x < 0.0 for x in coulomb + viscous + velocity_min):
        raise ValueError("friction coefficients/velocity minima must be non-negative")
    if any(vmax <= vmin for vmin, vmax in zip(velocity_min, velocity_max)):
        raise ValueError("each validated velocity range must satisfy min < max")
    if require_stribeck:
        static = _float_array(cal.get("static_friction_amplitude_Nm", []),
                              "static_friction_amplitude_Nm")
        stribeck_velocity = _float_array(
            cal.get("stribeck_velocity_rad_s", []), "stribeck_velocity_rad_s")
        if any(fs + 1e-8 < fc for fs, fc in zip(static, coulomb)):
            raise ValueError("static_friction_amplitude_Nm must be >= coulomb_Nm")
        if any(vs <= 0.0 for vs in stribeck_velocity):
            raise ValueError("stribeck_velocity_rad_s must be positive")
    else:
        static = coulomb.copy()
        stribeck_velocity = [0.05] * 7
    enable = cal.get("recommended_enable", [True] * 7)
    if not isinstance(enable, (list, tuple)) or len(enable) != 7 or not all(
            isinstance(x, bool) for x in enable):
        raise ValueError("recommended_enable must contain exactly 7 booleans")
    generated_at = cal.get("generated_at_unix_s", os.path.getmtime(path))
    generated_at = float(generated_at)
    if not math.isfinite(generated_at):
        raise ValueError("generated_at_unix_s is not finite")
    return {
        "source": os.path.abspath(path),
        "generated_at": generated_at,
        "smoothing": smoothing,
        "velocity_min": velocity_min,
        "velocity_max": velocity_max,
        "coulomb": coulomb,
        "viscous": viscous,
        "static": static,
        "stribeck_velocity": stribeck_velocity,
        "recommended_enable": enable,
    }


def _resolve_friction_calibration(context, robot_type, arm_prefix, mode):
    zeros = [0.0] * 7
    result = {
        "requested": _as_bool(LaunchConfiguration("friction_compensation_enabled").perform(context)),
        "valid": False,
        "source": "",
        "age_hours": -1.0,
        "scale": float(LaunchConfiguration("friction_compensation_scale").perform(context)),
        "smoothing": 0.001,
        "velocity_min": [0.02] * 7,
        "velocity_max": [0.25] * 7,
        "enable": zeros.copy(),
        "coulomb": zeros.copy(),
        "viscous": zeros.copy(),
        "static": zeros.copy(),
        "stribeck_velocity": [0.05] * 7,
    }
    if not math.isfinite(result["scale"]) or not 0.0 <= result["scale"] <= 1.0:
        raise RuntimeError("friction_compensation_scale must be finite and in [0, 1] for this experimental controller")
    if _as_bool(LaunchConfiguration("friction_apply_torque_bias").perform(context)):
        _LOG.warning(
            "friction_apply_torque_bias is ignored: the current compensation model is strictly odd"
        )
    if not result["requested"] or mode != "impedance":
        return result

    require_stribeck = _as_bool(LaunchConfiguration("friction_stribeck_enabled").perform(context))
    max_age_hours = float(
        LaunchConfiguration("friction_calibration_max_age_hours").perform(context)
    )
    if not math.isfinite(max_age_hours) or max_age_hours <= 0.0:
        raise RuntimeError("friction_calibration_max_age_hours must be finite and > 0")

    expected_prefix = (arm_prefix + "_") if arm_prefix else ""
    expected_joint_names = [f"{expected_prefix}{robot_type}_joint{i}" for i in range(1, 8)]
    explicit_file = LaunchConfiguration("friction_calibration_file").perform(context).strip()
    directory = LaunchConfiguration("friction_calibration_directory").perform(context).strip()
    if not directory:
        directory = _default_calibration_directory()

    if explicit_file:
        candidates = [os.path.expanduser(explicit_file)]
    else:
        directory = os.path.expanduser(directory)
        candidates = sorted(
            glob.glob(os.path.join(directory, "friction_calibration_*.yaml")) +
            glob.glob(os.path.join(directory, "friction_calibration_*.yml"))
        )

    parsed = []
    for candidate in candidates:
        try:
            parsed.append(_parse_friction_calibration(candidate, expected_joint_names, require_stribeck))
        except Exception as exc:
            _LOG.warning(f"Ignoring friction calibration '{candidate}': {exc}")

    if not parsed:
        _LOG.warning(
            f"Friction compensation requested, but no valid calibration was found in '{directory}'. "
            "Starting impedance control without friction feedforward."
        )
        return result

    chosen = max(parsed, key=lambda item: item["generated_at"])
    age_hours = max(0.0, (time.time() - chosen["generated_at"]) / 3600.0)
    result["source"] = chosen["source"]
    result["age_hours"] = age_hours
    if age_hours > max_age_hours:
        _LOG.warning(
            f"Newest friction calibration is stale ({age_hours:.2f} h > {max_age_hours:.2f} h): "
            f"'{chosen['source']}'. Starting without friction feedforward."
        )
        return result

    use_recommended = _as_bool(
        LaunchConfiguration("friction_use_recommended_enable").perform(context)
    )
    enable = chosen["recommended_enable"] if use_recommended else [True] * 7
    result.update({
        "valid": any(enable),
        "smoothing": chosen["smoothing"],
        "velocity_min": chosen["velocity_min"],
        "velocity_max": chosen["velocity_max"],
        "enable": [1.0 if x else 0.0 for x in enable],
        "coulomb": chosen["coulomb"],
        "viscous": chosen["viscous"],
        "static": chosen["static"],
        "stribeck_velocity": chosen["stribeck_velocity"],
    })
    enabled_joints = [str(i + 1) for i, enabled in enumerate(enable) if enabled]
    _LOG.info(
        f"Using fresh friction calibration '{chosen['source']}' ({age_hours:.2f} h old); "
        f"enabled joints: {','.join(enabled_joints) if enabled_joints else 'none'}; "
        f"strictly odd {'Stribeck' if require_stribeck else 'Coulomb-viscous'} model"
    )
    return result


def _setup(context):
    mode = LaunchConfiguration("mode").perform(context).strip().lower()
    if mode not in ("velocity", "impedance"):
        raise RuntimeError("mode must be 'velocity' or 'impedance'")

    robot_type = LaunchConfiguration("robot_type").perform(context)
    arm_prefix = LaunchConfiguration("arm_prefix").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    command_topic = LaunchConfiguration("command_topic").perform(context)
    command_timeout = float(LaunchConfiguration("command_timeout").perform(context))
    load_gripper = _as_bool(LaunchConfiguration("load_gripper").perform(context))
    use_fake_hardware = _as_bool(LaunchConfiguration("use_fake_hardware").perform(context))
    joint_state_rate = int(LaunchConfiguration("joint_state_rate").perform(context))
    load_franka_robot_state_broadcaster = _as_bool(
        LaunchConfiguration("load_franka_robot_state_broadcaster").perform(context)
    )
    readiness_stable_time = float(
        LaunchConfiguration("readiness_stable_time").perform(context)
    )
    readiness_warn_after = float(
        LaunchConfiguration("readiness_warn_after").perform(context)
    )
    if joint_state_rate <= 0:
        raise RuntimeError("joint_state_rate must be > 0")
    if readiness_stable_time < 0.0:
        raise RuntimeError("readiness_stable_time must be >= 0")

    stiffness = _array7(LaunchConfiguration("stiffness").perform(context), "stiffness")
    damping = _array7(LaunchConfiguration("damping").perform(context), "damping")
    mass_damping = _array7(LaunchConfiguration("mass_damping").perform(context), "mass_damping")
    max_torque = _array7(LaunchConfiguration("max_torque").perform(context), "max_torque")
    max_velocity = _array7(LaunchConfiguration("max_velocity").perform(context), "max_velocity")
    max_acceleration = _array7(LaunchConfiguration("max_acceleration").perform(context), "max_acceleration")
    max_jerk = _array7(LaunchConfiguration("max_jerk").perform(context), "max_jerk")
    velocity_scale = float(LaunchConfiguration("velocity_scale").perform(context))
    tracking_time_constant = float(LaunchConfiguration("tracking_time_constant").perform(context))
    effort_ff_scale = float(LaunchConfiguration("effort_feedforward_scale").perform(context))
    delta_tau_max = float(LaunchConfiguration("delta_tau_max").perform(context))
    hold_on_timeout = _as_bool(LaunchConfiguration("hold_position_on_timeout").perform(context))
    friction = _resolve_friction_calibration(context, robot_type, arm_prefix, mode)
    breakaway_enabled = (
        mode == "impedance"
        and _as_bool(
            LaunchConfiguration("breakaway_enabled").perform(context)
        )
    )
    breakaway_values = {name: float(LaunchConfiguration(name).perform(context)) for name in (
        "breakaway_velocity_epsilon", "breakaway_external_torque_deadband",
        "breakaway_gain", "breakaway_max_torque", "breakaway_slew_rate")}
    if any(not math.isfinite(v) for v in breakaway_values.values()):
        raise RuntimeError("breakaway parameters must be finite")
    residual_enabled = (
        mode == "impedance"
        and _as_bool(
            LaunchConfiguration("gravity_error_compensation_enabled").perform(context)
        )
    )
    residual_args = {
        "calibration_lease_s": "gravity_error_calibration_lease_s",
        "command_velocity_epsilon": "gravity_error_command_velocity_epsilon",
        "measured_velocity_epsilon": "gravity_error_measured_velocity_epsilon",
        "stationary_dwell_s": "gravity_error_stationary_dwell_s",
        "minimum_sample_s": "gravity_error_minimum_sample_s",
        "filter_tau_s": "gravity_error_filter_tau_s",
        "sampling_command_torque_epsilon": "gravity_error_sampling_command_torque_epsilon",
        "sample_deviation_limit": "gravity_error_sample_deviation_limit",
        "output_slew_rate": "gravity_error_output_slew_rate",
        "pose_radius": "gravity_error_pose_radius",
        "pose_fade_width": "gravity_error_pose_fade_width",
    }
    residual_values = {
        key: float(LaunchConfiguration(arg).perform(context))
        for key, arg in residual_args.items()
    }
    residual_limits = _array7(LaunchConfiguration("gravity_error_max_torque").perform(context), "gravity_error_max_torque")
    if any(not math.isfinite(x) or x <= 0.0 for x in residual_limits):
        raise RuntimeError("gravity_error_max_torque must be seven positive finite numbers")
    if any(not math.isfinite(x) for x in residual_values.values()):
        raise RuntimeError("residual calibration parameters must be finite")
    stribeck_enabled = _as_bool(LaunchConfiguration("friction_stribeck_enabled").perform(context))
    friction_limits = _array7(
        LaunchConfiguration("friction_max_compensation_torque").perform(context),
        "friction_max_compensation_torque",
    )
    if any((not math.isfinite(limit)) or limit <= 0 for limit in friction_limits):
        raise RuntimeError("friction_max_compensation_torque must contain seven positive finite limits")


    configure_collision = _as_bool(
        LaunchConfiguration("configure_collision_behavior").perform(context)
    )
    collision_service_timeout = float(
        LaunchConfiguration("collision_service_timeout").perform(context)
    )
    torque_scale = float(LaunchConfiguration("collision_torque_scale").perform(context))
    force_scale = float(LaunchConfiguration("collision_force_scale").perform(context))
    if torque_scale <= 0.0 or force_scale <= 0.0:
        raise RuntimeError("collision threshold scales must be > 0")

    lower_tau_acc = _scaled(_array7(
        LaunchConfiguration("lower_torque_thresholds_acceleration").perform(context),
        "lower_torque_thresholds_acceleration"), torque_scale)
    upper_tau_acc = _scaled(_array7(
        LaunchConfiguration("upper_torque_thresholds_acceleration").perform(context),
        "upper_torque_thresholds_acceleration"), torque_scale)
    lower_tau_nom = _scaled(_array7(
        LaunchConfiguration("lower_torque_thresholds_nominal").perform(context),
        "lower_torque_thresholds_nominal"), torque_scale)
    upper_tau_nom = _scaled(_array7(
        LaunchConfiguration("upper_torque_thresholds_nominal").perform(context),
        "upper_torque_thresholds_nominal"), torque_scale)

    lower_force_acc = _scaled(_array6(
        LaunchConfiguration("lower_force_thresholds_acceleration").perform(context),
        "lower_force_thresholds_acceleration"), force_scale)
    upper_force_acc = _scaled(_array6(
        LaunchConfiguration("upper_force_thresholds_acceleration").perform(context),
        "upper_force_thresholds_acceleration"), force_scale)
    lower_force_nom = _scaled(_array6(
        LaunchConfiguration("lower_force_thresholds_nominal").perform(context),
        "lower_force_thresholds_nominal"), force_scale)
    upper_force_nom = _scaled(_array6(
        LaunchConfiguration("upper_force_thresholds_nominal").perform(context),
        "upper_force_thresholds_nominal"), force_scale)

    # Franka mock hardware does not export the robot model semantic interface.
    if use_fake_hardware:
        mass_damping = [0.0] * 7
        if residual_enabled or breakaway_enabled:
            raise RuntimeError("gravity_error_compensation_enabled requires real Franka model and measured torque interfaces")

    runtime_yaml = os.path.join(tempfile.gettempdir(), f"low_level_controllers_{os.getpid()}.yaml")
    with open(runtime_yaml, "w", encoding="utf-8") as f:
        f.write(f'''/**:
  controller_manager:
    ros__parameters:
      update_rate: 1000
      thread_priority: 95
      joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster
      franka_robot_state_broadcaster:
        type: franka_robot_state_broadcaster/FrankaRobotStateBroadcaster
      low_level_joint_velocity_controller:
        type: low_level/JointVelocityController
      low_level_joint_impedance_controller:
        type: low_level/JointImpedanceController
      dynamics_observer_controller:
        type: low_level/DynamicsObserverController

  low_level_joint_velocity_controller:
    ros__parameters:
      robot_type: "{robot_type}"
      arm_prefix: "{arm_prefix}"
      command_topic: "{command_topic}"
      command_timeout: {command_timeout}
      velocity_scale: {velocity_scale}
      tracking_time_constant: {tracking_time_constant}
      max_velocity: {_yaml_array(max_velocity)}
      max_acceleration: {_yaml_array(max_acceleration)}
      max_jerk: {_yaml_array(max_jerk)}
      controller_version: "velocity_tracker_v5"

  low_level_joint_impedance_controller:
    ros__parameters:
      robot_type: "{robot_type}"
      arm_prefix: "{arm_prefix}"
      command_topic: "{command_topic}"
      command_timeout: {command_timeout}
      stiffness: {_yaml_array(stiffness)}
      damping: {_yaml_array(damping)}
      mass_damping: {_yaml_array(mass_damping)}
      effort_feedforward_scale: {effort_ff_scale}
      delta_tau_max: {delta_tau_max}
      max_torque: {_yaml_array(max_torque)}
      hold_position_on_timeout: {str(hold_on_timeout).lower()}
      breakaway_enabled: {str(breakaway_enabled).lower()}
      breakaway_velocity_epsilon: {breakaway_values['breakaway_velocity_epsilon']}
      breakaway_external_torque_deadband: {breakaway_values['breakaway_external_torque_deadband']}
      breakaway_gain: {breakaway_values['breakaway_gain']}
      breakaway_max_torque: {breakaway_values['breakaway_max_torque']}
      breakaway_slew_rate: {breakaway_values['breakaway_slew_rate']}
      gravity_error_compensation_enabled: {str(residual_enabled).lower()}
      gravity_error_calibration_lease_s: {residual_values['calibration_lease_s']}
      gravity_error_command_velocity_epsilon: {residual_values['command_velocity_epsilon']}
      gravity_error_measured_velocity_epsilon: {residual_values['measured_velocity_epsilon']}
      gravity_error_stationary_dwell_s: {residual_values['stationary_dwell_s']}
      gravity_error_minimum_sample_s: {residual_values['minimum_sample_s']}
      gravity_error_filter_tau_s: {residual_values['filter_tau_s']}
      gravity_error_sampling_command_torque_epsilon: {residual_values['sampling_command_torque_epsilon']}
      gravity_error_sample_deviation_limit: {residual_values['sample_deviation_limit']}
      gravity_error_output_slew_rate: {residual_values['output_slew_rate']}
      gravity_error_pose_radius: {residual_values['pose_radius']}
      gravity_error_pose_fade_width: {residual_values['pose_fade_width']}
      gravity_error_max_torque: {_yaml_array(residual_limits)}
      friction_compensation_enabled: {str(bool(friction["requested"])).lower()}
      friction_calibration_valid: {str(bool(friction["valid"])).lower()}
      friction_calibration_source: {json.dumps(str(friction["source"]))}
      friction_calibration_age_hours: {float(friction["age_hours"])}
      friction_compensation_scale: {float(friction["scale"])}
      friction_smoothing_velocity: {float(friction["smoothing"])}
      friction_stribeck_enabled: {str(stribeck_enabled).lower()}
      friction_stribeck_velocity_per_joint: {_yaml_array(friction["stribeck_velocity"])}
      friction_static: {_yaml_array(friction["static"])}
      friction_max_compensation_torque: {_yaml_array(friction_limits)}
      # Calibration range metadata only; no speed-based friction gating at runtime.
      friction_validated_velocity_min: {float(min(friction["velocity_min"]))}
      friction_validated_velocity_min_per_joint: {_yaml_array(friction["velocity_min"])}
      # Legacy scalar for compatibility with older controller binaries.
      friction_validated_velocity_max: {float(min(
          vmax for vmax, enabled in zip(friction["velocity_max"], friction["enable"])
          if enabled > 0.5
      ) if any(enabled > 0.5 for enabled in friction["enable"]) else min(friction["velocity_max"]))}
      friction_validated_velocity_max_per_joint: {_yaml_array(friction["velocity_max"])}
      friction_joint_enable: {_yaml_array(friction["enable"])}
      friction_coulomb: {_yaml_array(friction["coulomb"])}
      friction_viscous: {_yaml_array(friction["viscous"])}

  dynamics_observer_controller:
    ros__parameters:
      robot_type: "{robot_type}"
      arm_prefix: "{arm_prefix}"
      observation_topic: "{LaunchConfiguration("dynamics_observation_topic").perform(context)}"
      publish_rate: {float(LaunchConfiguration("dynamics_publish_rate").perform(context))}
      acceleration_filter_tau: {float(LaunchConfiguration("dynamics_acceleration_filter_tau").perform(context))}
''')

    franka_launch = os.path.join(
        get_package_share_directory("franka_bringup"), "launch", "franka.launch.py"
    )
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(franka_launch),
        launch_arguments={
            "robot_type": robot_type,
            "arm_prefix": arm_prefix,
            "namespace": namespace,
            "robot_ip": robot_ip,
            "load_gripper": str(load_gripper).lower(),
            "use_fake_hardware": str(use_fake_hardware).lower(),
            # The Cartesian PID/IK stack only requires joint states. The full
            # Franka robot-state broadcaster runs at 1 kHz and is optional, so
            # keep it disabled by default to reduce controller-manager RT load.
            "load_franka_robot_state_broadcaster": str(load_franka_robot_state_broadcaster).lower(),
            "joint_state_rate": str(joint_state_rate),
            "controllers_yaml": runtime_yaml,
        }.items(),
    )

    controller_name = (
        "low_level_joint_velocity_controller"
        if mode == "velocity"
        else "low_level_joint_impedance_controller"
    )

    spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller_name, "--controller-manager", "controller_manager"],
        namespace=namespace,
        output="screen",
    )

    bringup_ready = Node(
        package="low_level",
        executable="readiness_gate.py",
        name="low_level_bringup_ready",
        namespace=namespace,
        output="screen",
        parameters=[{
            "required_publisher_topics": ["/franka/joint_states"],
            "stable_time": readiness_stable_time,
            "warn_after": readiness_warn_after,
        }],
    )

    actions = [bringup, bringup_ready]

    # Collision behavior can only be changed while Franka hardware is idle.
    # Wait for actual bringup readiness rather than an arbitrary delay, then
    # configure collision behavior before activating the command controller.
    if configure_collision and not use_fake_hardware:
        collision_configurator = Node(
            package="low_level",
            executable="collision_behavior_configurator",
            namespace=namespace,
            output="screen",
            parameters=[{
                "service_name": LaunchConfiguration("collision_service_name"),
                "service_timeout": collision_service_timeout,
                "lower_torque_thresholds_acceleration": lower_tau_acc,
                "upper_torque_thresholds_acceleration": upper_tau_acc,
                "lower_torque_thresholds_nominal": lower_tau_nom,
                "upper_torque_thresholds_nominal": upper_tau_nom,
                "lower_force_thresholds_acceleration": lower_force_acc,
                "upper_force_thresholds_acceleration": upper_force_acc,
                "lower_force_thresholds_nominal": lower_force_nom,
                "upper_force_thresholds_nominal": upper_force_nom,
            }],
        )
        actions.extend([
            RegisterEventHandler(
                OnProcessExit(target_action=bringup_ready, on_exit=[collision_configurator])
            ),
            RegisterEventHandler(
                OnProcessExit(target_action=collision_configurator, on_exit=[spawner])
            ),
        ])
    else:
        actions.append(
            RegisterEventHandler(
                OnProcessExit(target_action=bringup_ready, on_exit=[spawner])
            )
        )

    if load_gripper:
        gripper = Node(
            package="low_level",
            executable="gripper_control_node",
            namespace=namespace,
            output="screen",
            parameters=[{
                "command_topic": LaunchConfiguration("gripper_command_topic"),
                "move_action": LaunchConfiguration("gripper_move_action"),
                "grasp_action": LaunchConfiguration("gripper_grasp_action"),
                "max_width": LaunchConfiguration("gripper_max_width"),
                "speed": LaunchConfiguration("gripper_speed"),
                "force": LaunchConfiguration("gripper_force"),
                "epsilon_inner": LaunchConfiguration("gripper_epsilon_inner"),
                "epsilon_outer": LaunchConfiguration("gripper_epsilon_outer"),
                "command_deadband": LaunchConfiguration("gripper_command_deadband"),
                "close_uses_grasp": LaunchConfiguration("gripper_close_uses_grasp"),
            }],
        )
        actions.append(gripper)

    return actions


def generate_launch_description():
    default_joint_collision = "[20.0,20.0,18.0,18.0,16.0,14.0,12.0]"
    default_cartesian_collision = "[20.0,20.0,20.0,25.0,25.0,25.0]"

    args = [
        DeclareLaunchArgument("mode", default_value="impedance", description="velocity or impedance"),
        DeclareLaunchArgument("robot_ip", default_value="192.16.0.1"),
        DeclareLaunchArgument("robot_type", default_value="fr3"),
        DeclareLaunchArgument("arm_prefix", default_value=""),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("load_gripper", default_value="true"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument(
            "load_franka_robot_state_broadcaster",
            default_value="false",
            description="Load the 1 kHz Franka full-state broadcaster. PID/IK velocity control does not require it.",
        ),
        DeclareLaunchArgument(
            "joint_state_rate",
            default_value="250",
            description="Joint-state publication rate in Hz; control loop remains at 1000 Hz.",
        ),
        DeclareLaunchArgument(
            "controller_start_delay",
            default_value="0.0",
            description="Deprecated compatibility argument; startup now uses readiness detection.",
        ),
        DeclareLaunchArgument(
            "readiness_stable_time", default_value="0.25",
            description="How long readiness conditions must remain continuously true before progressing.",
        ),
        DeclareLaunchArgument(
            "readiness_warn_after", default_value="5.0",
            description="Seconds before readiness gates start logging missing conditions.",
        ),
        DeclareLaunchArgument("command_topic", default_value="/fr3/joint_commands"),
        DeclareLaunchArgument("dynamics_observation_topic", default_value="/fr3/dynamics_observation"),
        DeclareLaunchArgument("dynamics_publish_rate", default_value="200.0"),
        DeclareLaunchArgument("dynamics_acceleration_filter_tau", default_value="0.03"),
        DeclareLaunchArgument("command_timeout", default_value="0.1"),
        DeclareLaunchArgument("stiffness", default_value="[0.0,0.0,0.0,0.0,0.0,0.0,0.0]"),
        DeclareLaunchArgument("damping", default_value="[0.0,0.0,0.0,0.0,0.0,0.0,0.0]"),
        DeclareLaunchArgument("mass_damping", default_value="[0.0,0.0,0.0,0.0,0.0,0.0,0.0]"),
        DeclareLaunchArgument("effort_feedforward_scale", default_value="1.0"),
        DeclareLaunchArgument("delta_tau_max", default_value="1.0"),
        DeclareLaunchArgument("max_torque", default_value="[0.0,0.0,0.0,0.0,0.0,0.0,0.0]"),
        DeclareLaunchArgument("hold_position_on_timeout", default_value="false"),
        DeclareLaunchArgument("breakaway_enabled", default_value="true",
                              description="Experimental low-speed positive assistance, OFF by default."),
        DeclareLaunchArgument("breakaway_velocity_epsilon", default_value="0.001"),
        DeclareLaunchArgument("breakaway_external_torque_deadband", default_value="0.6"),
        DeclareLaunchArgument("breakaway_gain", default_value="1.0"),
        DeclareLaunchArgument("breakaway_max_torque", default_value="8.0"),
        DeclareLaunchArgument("breakaway_slew_rate", default_value="0.10"),
        DeclareLaunchArgument("gravity_error_compensation_enabled", default_value="true",
                              description="Experimental unloaded calibration and bounded local torque correction; disabled by default."),
        DeclareLaunchArgument("gravity_error_calibration_lease_s", default_value="0.35"),
        DeclareLaunchArgument("gravity_error_command_velocity_epsilon", default_value="0.002"),
        DeclareLaunchArgument("gravity_error_measured_velocity_epsilon", default_value="0.003"),
        DeclareLaunchArgument("gravity_error_stationary_dwell_s", default_value="0.5"),
        DeclareLaunchArgument("gravity_error_minimum_sample_s", default_value="1.0"),
        DeclareLaunchArgument("gravity_error_filter_tau_s", default_value="2.0"),
        DeclareLaunchArgument("gravity_error_sampling_command_torque_epsilon", default_value="0.03"),
        DeclareLaunchArgument("gravity_error_sample_deviation_limit", default_value="0.10"),
        DeclareLaunchArgument("gravity_error_output_slew_rate", default_value="0.10"),
        DeclareLaunchArgument("gravity_error_pose_radius", default_value="0.25"),
        DeclareLaunchArgument("gravity_error_pose_fade_width", default_value="0.25"),
        DeclareLaunchArgument("gravity_error_max_torque", default_value="[8.0,8.0,8.0,8.0,8.0,8.0,8.0]"),
        DeclareLaunchArgument(
            "friction_compensation_enabled", default_value="true",
            description="Enable calibrated friction feedforward in impedance mode when a fresh calibration is available.",
        ),
        DeclareLaunchArgument(
            "friction_calibration_directory", default_value="",
            description="Directory containing friction_calibration_*.yaml. Empty auto-resolves to <workspace>/data/calibration.",
        ),
        DeclareLaunchArgument(
            "friction_calibration_file", default_value="",
            description="Optional explicit calibration YAML. Empty selects the newest valid file in the calibration directory.",
        ),
        DeclareLaunchArgument(
            "friction_calibration_max_age_hours", default_value="24.0",
            description="Maximum calibration age allowed for automatic friction compensation.",
        ),
        DeclareLaunchArgument(
            "friction_use_recommended_enable", default_value="true",
            description="Honor the per-joint recommended_enable mask from the calibration file.",
        ),
        DeclareLaunchArgument(
            "friction_apply_torque_bias", default_value="false",
            description="Deprecated compatibility argument. Torque bias is never applied by the strictly odd friction model.",
        ),
        DeclareLaunchArgument(
            "friction_compensation_scale", default_value="1.0",
            description="Global scale applied to calibrated friction feedforward (0 disables output without disabling file loading).",
        ),
        DeclareLaunchArgument("friction_stribeck_enabled", default_value="true",
                              description="Default true: require fresh schema-v4 Stribeck calibration; false explicitly selects legacy Coulomb mode"),
        DeclareLaunchArgument(
            "friction_max_compensation_torque",
            default_value="[0.75,0.75,0.75,0.75,0.75,0.75,0.75]",
            description="Per-joint absolute friction feedforward limit (Nm).",
        ),
        DeclareLaunchArgument("velocity_scale", default_value="1.0"),
        DeclareLaunchArgument(
            "tracking_time_constant", default_value="0.03",
            description="Low-level velocity tracking time constant in seconds; larger is quieter/smoother",
        ),
        DeclareLaunchArgument("max_velocity", default_value="[0.0,0.0,0.0,0.0,0.0,0.0,0.0]"),
        DeclareLaunchArgument(
            "max_acceleration",
            default_value="[2.0,2.0,2.0,2.0,2.0,2.0,2.0]",
            description="Low-level 1 kHz interpolation acceleration limits [rad/s^2]",
        ),
        DeclareLaunchArgument(
            "max_jerk",
            default_value="[100.0,100.0,100.0,100.0,100.0,100.0,100.0]",
            description="Low-level 1 kHz tracking jerk limits [rad/s^3]",
        ),

        DeclareLaunchArgument("configure_collision_behavior", default_value="true"),
        DeclareLaunchArgument("collision_service_name", default_value="/service_server/set_full_collision_behavior"),
        DeclareLaunchArgument("collision_service_timeout", default_value="15.0"),
        DeclareLaunchArgument("collision_torque_scale", default_value="2.0"),
        DeclareLaunchArgument("collision_force_scale", default_value="2.0"),
        DeclareLaunchArgument("lower_torque_thresholds_acceleration", default_value=default_joint_collision),
        DeclareLaunchArgument("upper_torque_thresholds_acceleration", default_value=default_joint_collision),
        DeclareLaunchArgument("lower_torque_thresholds_nominal", default_value=default_joint_collision),
        DeclareLaunchArgument("upper_torque_thresholds_nominal", default_value=default_joint_collision),
        DeclareLaunchArgument("lower_force_thresholds_acceleration", default_value=default_cartesian_collision),
        DeclareLaunchArgument("upper_force_thresholds_acceleration", default_value=default_cartesian_collision),
        DeclareLaunchArgument("lower_force_thresholds_nominal", default_value=default_cartesian_collision),
        DeclareLaunchArgument("upper_force_thresholds_nominal", default_value=default_cartesian_collision),

        DeclareLaunchArgument("gripper_command_topic", default_value="/fr3/gripper"),
        DeclareLaunchArgument("gripper_move_action", default_value="/franka_gripper/move"),
        DeclareLaunchArgument("gripper_grasp_action", default_value="/franka_gripper/grasp"),
        DeclareLaunchArgument("gripper_max_width", default_value="0.08"),
        DeclareLaunchArgument("gripper_speed", default_value="0.1"),
        DeclareLaunchArgument("gripper_force", default_value="40.0"),
        DeclareLaunchArgument("gripper_epsilon_inner", default_value="0.005"),
        DeclareLaunchArgument("gripper_epsilon_outer", default_value="0.005"),
        DeclareLaunchArgument("gripper_command_deadband", default_value="0.001"),
        DeclareLaunchArgument("gripper_close_uses_grasp", default_value="true"),
    ]
    return LaunchDescription(args + [OpaqueFunction(function=_setup)])
