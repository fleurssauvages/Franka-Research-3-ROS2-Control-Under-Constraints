import ast
import os

from ament_index_python.packages import get_package_share_directory
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


def _as_bool(text):
    return text.strip().lower() in ("1", "true", "yes", "on")


def _array7(text, name):
    try:
        value = ast.literal_eval(text)
    except (ValueError, SyntaxError) as exc:
        raise RuntimeError(f"{name} must be a Python-style list of 7 numbers") from exc
    if not isinstance(value, (list, tuple)) or len(value) != 7:
        raise RuntimeError(f"{name} must contain exactly 7 numbers")
    return [float(x) for x in value]


def _array6(text, name):
    try:
        value = ast.literal_eval(text)
    except (ValueError, SyntaxError) as exc:
        raise RuntimeError(f"{name} must be a Python-style list of 6 numbers") from exc
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise RuntimeError(f"{name} must contain exactly 6 numbers")
    return [float(x) for x in value]


def _setup(context):
    package_share = get_package_share_directory("franka_cartesian_control")
    controller = LaunchConfiguration("controller").perform(context).strip().lower()
    if controller not in ("ik", "qp", "ik_force"):
        raise RuntimeError("controller must be 'ik', 'qp', or 'ik_force'")

    config_file = LaunchConfiguration("config_file").perform(context)
    robot_type = LaunchConfiguration("robot_type").perform(context)
    arm_prefix = LaunchConfiguration("arm_prefix").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    twist_topic = LaunchConfiguration("twist_topic").perform(context)
    joint_state_topic = LaunchConfiguration("joint_state_topic").perform(context)
    joint_command_topic = LaunchConfiguration("joint_command_topic").perform(context)
    tau_command_topic = LaunchConfiguration("tau_command_topic").perform(context)
    frequency = float(LaunchConfiguration("frequency").perform(context))
    max_velocity = _array7(
        LaunchConfiguration("max_velocity").perform(context), "max_velocity"
    )
    max_acceleration = _array7(
        LaunchConfiguration("max_acceleration").perform(context), "max_acceleration"
    )
    joint_limit_margin = float(LaunchConfiguration("joint_limit_margin").perform(context))
    repulsion_when_idle = _as_bool(LaunchConfiguration("repulsion_when_idle").perform(context))
    idle_safety_frequency = float(LaunchConfiguration("idle_safety_frequency").perform(context))
    low_level_mode = LaunchConfiguration("low_level_mode").perform(context).strip().lower()
    if low_level_mode == "auto":
        low_level_mode = "torque" if controller == "ik_force" else "velocity"
    if low_level_mode not in ("velocity", "impedance", "torque"):
        raise RuntimeError("low_level_mode must be 'auto', 'velocity', 'impedance', or 'torque'")
    if (controller == "ik_force") != (low_level_mode == "torque"):
        raise RuntimeError("ik_force requires low_level_mode=torque; ik/qp require velocity or impedance")

    executable = {
        "ik": "ik_velocity_controller",
        "qp": "qp_velocity_controller",
        "ik_force": "ik_force_controller",
    }[controller]
    node_parameters = {
        "robot_type": robot_type,
        "arm_prefix": arm_prefix,
        "twist_topic": twist_topic,
        "joint_state_topic": joint_state_topic,
        "joint_command_topic": joint_command_topic,
        "franka_robot_state_topic": LaunchConfiguration("franka_robot_state_topic").perform(context),
        "franka_state_timeout": float(LaunchConfiguration("franka_state_timeout").perform(context)),
        "frequency": frequency,
        "joint_limit_margin": joint_limit_margin,
        "repulsion_when_idle": repulsion_when_idle,
        "idle_safety_frequency": idle_safety_frequency,
    }

    # One shared set of joint motion limits is applied to both IK and QP.
    # This avoids the previous hidden behavior where QP silently kept the YAML
    # 0.3 rad/s^2 limit while launch arguments only changed IK.
    node_parameters["max_velocity"] = max_velocity
    if controller != "ik_force":
        node_parameters["max_acceleration"] = max_acceleration
    else:
        # Cartesian force output uses the same KDL IK geometry but publishes
        # JointState.effort rather than any joint velocity command.
        node_parameters.update({
            "force_input_mode": LaunchConfiguration("force_input_mode").perform(context),
            "wrench_topic": LaunchConfiguration("wrench_topic").perform(context),
            "torque_command_topic": tau_command_topic,
            "joint_state_timeout": float(LaunchConfiguration("force_joint_state_timeout").perform(context)),
            "velocity_to_wrench_gains": _array6(
                LaunchConfiguration("force_velocity_to_wrench_gains").perform(context),
                "force_velocity_to_wrench_gains"),
            "max_cartesian_force": float(LaunchConfiguration("force_max_cartesian_force").perform(context)),
            "max_cartesian_moment": float(LaunchConfiguration("force_max_cartesian_moment").perform(context)),
            "max_joint_torque": _array7(
                LaunchConfiguration("force_max_joint_torque").perform(context),
                "force_max_joint_torque"),
        })
        node_parameters.pop("joint_command_topic")

    node = Node(
        package="franka_cartesian_control",
        executable=executable,
        name=executable,
        namespace=namespace,
        output="screen",
        parameters=[config_file, node_parameters],
    )

    actions = []
    if _as_bool(LaunchConfiguration("start_low_level").perform(context)):
        low_level_launch = os.path.join(
            get_package_share_directory("low_level"), "launch", "control.launch.py"
        )
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(low_level_launch),
                launch_arguments={
                    "mode": low_level_mode,
                    "robot_ip": LaunchConfiguration("robot_ip").perform(context),
                    "robot_type": robot_type,
                    "arm_prefix": arm_prefix,
                    "namespace": namespace,
                    "use_fake_hardware": LaunchConfiguration("use_fake_hardware").perform(context),
                    "enable_gripper": LaunchConfiguration("enable_gripper").perform(context),
                    "load_franka_robot_state_broadcaster": "true",  # Required for live Desk F_T_EE.
                    "joint_state_rate": LaunchConfiguration("joint_state_rate").perform(context),
                    "controller_start_delay": LaunchConfiguration("low_level_controller_start_delay").perform(context),
                    "readiness_stable_time": LaunchConfiguration("readiness_stable_time").perform(context),
                    "readiness_warn_after": LaunchConfiguration("readiness_warn_after").perform(context),
                    "command_topic": joint_command_topic,
                    "tau_command_topic": tau_command_topic,
                    "torque_max_torque": LaunchConfiguration("torque_max_torque").perform(context),
                    "torque_delta_tau_max": LaunchConfiguration("torque_delta_tau_max").perform(context),
                    "command_timeout": LaunchConfiguration("low_level_command_timeout").perform(context),
                    "tracking_time_constant": LaunchConfiguration("low_level_tracking_time_constant").perform(context),
                    "max_acceleration": LaunchConfiguration("low_level_max_acceleration").perform(context),
                    "max_jerk": LaunchConfiguration("low_level_max_jerk").perform(context),
                    "stiffness": LaunchConfiguration("impedance_stiffness").perform(context),
                    "damping": LaunchConfiguration("impedance_damping").perform(context),
                    "mass_damping": LaunchConfiguration("impedance_mass_damping").perform(context),
                    "delta_tau_max": LaunchConfiguration("impedance_delta_tau_max").perform(context),
                    "max_torque": LaunchConfiguration("impedance_max_torque").perform(context),
                    "hold_position_on_timeout": LaunchConfiguration("impedance_hold_position_on_timeout").perform(context),
                    "breakaway_enabled": LaunchConfiguration("impedance_breakaway_enabled").perform(context),
                    "breakaway_velocity_epsilon": LaunchConfiguration("impedance_breakaway_velocity_epsilon").perform(context),
                    "breakaway_external_torque_deadband": LaunchConfiguration("impedance_breakaway_external_torque_deadband").perform(context),
                    "breakaway_gain": LaunchConfiguration("impedance_breakaway_gain").perform(context),
                    "breakaway_max_torque": LaunchConfiguration("impedance_breakaway_max_torque").perform(context),
                    "breakaway_slew_rate": LaunchConfiguration("impedance_breakaway_slew_rate").perform(context),
                    "gravity_error_compensation_enabled": LaunchConfiguration("impedance_gravity_error_compensation_enabled").perform(context),
                    "gravity_error_calibration_lease_s": LaunchConfiguration("impedance_gravity_error_calibration_lease_s").perform(context),
                    "gravity_error_command_velocity_epsilon": LaunchConfiguration("impedance_gravity_error_command_velocity_epsilon").perform(context),
                    "gravity_error_measured_velocity_epsilon": LaunchConfiguration("impedance_gravity_error_measured_velocity_epsilon").perform(context),
                    "gravity_error_stationary_dwell_s": LaunchConfiguration("impedance_gravity_error_stationary_dwell_s").perform(context),
                    "gravity_error_minimum_sample_s": LaunchConfiguration("impedance_gravity_error_minimum_sample_s").perform(context),
                    "gravity_error_filter_tau_s": LaunchConfiguration("impedance_gravity_error_filter_tau_s").perform(context),
                    "gravity_error_sampling_command_torque_epsilon": LaunchConfiguration("impedance_gravity_error_sampling_command_torque_epsilon").perform(context),
                    "gravity_error_sample_deviation_limit": LaunchConfiguration("impedance_gravity_error_sample_deviation_limit").perform(context),
                    "gravity_error_output_slew_rate": LaunchConfiguration("impedance_gravity_error_output_slew_rate").perform(context),
                    "gravity_error_pose_radius": LaunchConfiguration("impedance_gravity_error_pose_radius").perform(context),
                    "gravity_error_pose_fade_width": LaunchConfiguration("impedance_gravity_error_pose_fade_width").perform(context),
                    "gravity_error_max_torque": LaunchConfiguration("impedance_gravity_error_max_torque").perform(context),
                    "friction_compensation_enabled": LaunchConfiguration("impedance_friction_compensation_enabled").perform(context),
                    "friction_calibration_directory": LaunchConfiguration("impedance_friction_calibration_directory").perform(context),
                    "friction_calibration_file": LaunchConfiguration("impedance_friction_calibration_file").perform(context),
                    "friction_calibration_max_age_hours": LaunchConfiguration("impedance_friction_calibration_max_age_hours").perform(context),
                    "friction_use_recommended_enable": LaunchConfiguration("impedance_friction_use_recommended_enable").perform(context),
                    "friction_apply_torque_bias": LaunchConfiguration("impedance_friction_apply_torque_bias").perform(context),
                    "friction_compensation_scale": LaunchConfiguration("impedance_friction_compensation_scale").perform(context),
                    "friction_stribeck_enabled": LaunchConfiguration("impedance_friction_stribeck_enabled").perform(context),
                    "friction_max_compensation_torque": LaunchConfiguration("impedance_friction_max_compensation_torque").perform(context),
                    "configure_collision_behavior": LaunchConfiguration("configure_collision_behavior").perform(context),
                    "collision_torque_scale": LaunchConfiguration("collision_torque_scale").perform(context),
                    "collision_force_scale": LaunchConfiguration("collision_force_scale").perform(context),
                }.items(),
            )
        )
    low_level_controller_name = (
        "low_level_joint_velocity_controller"
        if low_level_mode == "velocity"
        else "low_level_joint_impedance_controller"
    )
    low_level_ready = Node(
        package="low_level",
        executable="readiness_gate.py",
        name="cartesian_low_level_ready",
        namespace=namespace,
        output="screen",
        parameters=[{
            "required_true_topics": [f"{low_level_controller_name}/ready"],
            "required_publisher_topics": [joint_state_topic],
            "stable_time": float(LaunchConfiguration("readiness_stable_time").perform(context)),
            "warn_after": float(LaunchConfiguration("readiness_warn_after").perform(context)),
        }],
    )
    actions.append(low_level_ready)
    actions.append(
        RegisterEventHandler(
            OnProcessExit(target_action=low_level_ready, on_exit=[node])
        )
    )
    return actions


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("franka_cartesian_control"), "config", "controllers.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("controller", default_value="qp", description="ik, qp, or ik_force"),
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("robot_type", default_value="fr3"),
        DeclareLaunchArgument("arm_prefix", default_value=""),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("robot_ip", default_value="192.16.0.1"),
        DeclareLaunchArgument("start_low_level", default_value="true"),
        DeclareLaunchArgument("enable_gripper", default_value="false",
                              description="Start the Franka Hand driver and gripper command adapter."),
        DeclareLaunchArgument("high_level_start_delay", default_value="0.0",
                              description="Deprecated compatibility argument; startup now uses readiness detection"),
        DeclareLaunchArgument("readiness_stable_time", default_value="0.25",
                              description="Readiness conditions must stay true for this long before startup continues"),
        DeclareLaunchArgument("readiness_warn_after", default_value="5.0",
                              description="Seconds before readiness gates log missing conditions"),
        DeclareLaunchArgument("low_level_mode", default_value="auto",
                              description="auto: velocity for ik/qp and torque for ik_force; or velocity/impedance/torque"),
        DeclareLaunchArgument("low_level_command_timeout", default_value="0.1"),
        DeclareLaunchArgument(
            "low_level_tracking_time_constant", default_value="0.03",
            description="Low-level velocity tracking time constant in seconds",
        ),
        DeclareLaunchArgument(
            "low_level_max_acceleration",
            default_value="[8.0,8.0,8.0,8.0,8.0,8.0,8.0]",
            description="Hardware-facing 1 kHz interpolation acceleration limits [rad/s^2]",
        ),
        DeclareLaunchArgument(
            "low_level_max_jerk",
            default_value="[100.0,100.0,100.0,100.0,100.0,100.0,100.0]",
            description="Hardware-facing 1 kHz tracking jerk limits [rad/s^3]",
        ),
        DeclareLaunchArgument("impedance_stiffness", default_value="[0.0,0.0,0.0,0.0,0.0,0.0,0.0]"),
        DeclareLaunchArgument("impedance_damping", default_value="[2.0,2.0,2.0,2.0,2.0,2.0,2.0]",
                              description="Impedance velocity damping; in torque mode uses desired velocity zero."),
        DeclareLaunchArgument("impedance_mass_damping", default_value="[5.0,5.0,5.0,5.0,5.0,5.0,5.0]",
                              description="Mass-weighted velocity damping; active for both impedance and torque on real hardware."),
        DeclareLaunchArgument("impedance_delta_tau_max", default_value="1.0"),
        DeclareLaunchArgument("torque_delta_tau_max", default_value="0.2",
                              description="Low-level 1 kHz torque slew cap [Nm/update], ik_force mode"),
        DeclareLaunchArgument("torque_max_torque", default_value="[8.0,8.0,8.0,6.0,4.0,4.0,4.0]",
                              description="Low-level absolute joint torque caps [Nm], ik_force mode"),
        DeclareLaunchArgument("impedance_max_torque", default_value="[0.0,0.0,0.0,0.0,0.0,0.0,0.0]"),
        DeclareLaunchArgument("impedance_hold_position_on_timeout", default_value="false"),
        # These compensators and gains are passed through unchanged in both
        # low-level impedance and compensated torque mode (controller=ik_force).
        DeclareLaunchArgument("impedance_breakaway_enabled", default_value="true"),
        DeclareLaunchArgument("impedance_breakaway_velocity_epsilon", default_value="0.001"),
        DeclareLaunchArgument("impedance_breakaway_external_torque_deadband", default_value="0.6"),
        DeclareLaunchArgument("impedance_breakaway_gain", default_value="1.0"),
        DeclareLaunchArgument("impedance_breakaway_max_torque", default_value="8.0"),
        DeclareLaunchArgument("impedance_breakaway_slew_rate", default_value="0.10"),
        DeclareLaunchArgument("impedance_gravity_error_compensation_enabled", default_value="true"),
        DeclareLaunchArgument("impedance_gravity_error_calibration_lease_s", default_value="0.35"),
        DeclareLaunchArgument("impedance_gravity_error_command_velocity_epsilon", default_value="0.002"),
        DeclareLaunchArgument("impedance_gravity_error_measured_velocity_epsilon", default_value="0.003"),
        DeclareLaunchArgument("impedance_gravity_error_stationary_dwell_s", default_value="0.5"),
        DeclareLaunchArgument("impedance_gravity_error_minimum_sample_s", default_value="1.0"),
        DeclareLaunchArgument("impedance_gravity_error_filter_tau_s", default_value="2.0"),
        DeclareLaunchArgument("impedance_gravity_error_sampling_command_torque_epsilon", default_value="0.03"),
        DeclareLaunchArgument("impedance_gravity_error_sample_deviation_limit", default_value="0.10"),
        DeclareLaunchArgument("impedance_gravity_error_output_slew_rate", default_value="0.10"),
        DeclareLaunchArgument("impedance_gravity_error_pose_radius", default_value="0.25"),
        DeclareLaunchArgument("impedance_gravity_error_pose_fade_width", default_value="0.25"),
        DeclareLaunchArgument("impedance_gravity_error_max_torque", default_value="[8.0,8.0,8.0,8.0,8.0,8.0,8.0]"),
        DeclareLaunchArgument("impedance_friction_compensation_enabled", default_value="true"),
        DeclareLaunchArgument(
            "impedance_friction_calibration_directory", default_value="",
            description="Calibration directory; empty auto-resolves to <workspace>/data/calibration.",
        ),
        DeclareLaunchArgument("impedance_friction_calibration_file", default_value=""),
        DeclareLaunchArgument("impedance_friction_calibration_max_age_hours", default_value="24.0"),
        DeclareLaunchArgument("impedance_friction_use_recommended_enable", default_value="true"),
        DeclareLaunchArgument("impedance_friction_apply_torque_bias", default_value="false", description="Deprecated; strictly odd friction compensation never applies torque bias"),
        DeclareLaunchArgument("impedance_friction_compensation_scale", default_value="1.0"),
        DeclareLaunchArgument("impedance_friction_stribeck_enabled", default_value="true",
                              description="Require schema-v4 Stribeck friction calibration (false selects legacy Coulomb)"),
        DeclareLaunchArgument("impedance_friction_max_compensation_torque",
                              default_value="[0.75,0.75,0.75,0.75,0.75,0.75,0.75]",
                              description="Per-joint friction feedforward caps [Nm]"),
        DeclareLaunchArgument("franka_robot_state_topic",
                              default_value="/franka_robot_state_broadcaster/robot_state"),
        DeclareLaunchArgument("franka_state_timeout", default_value="0.5"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument(
            "joint_state_rate", default_value="250",
            description="Joint-state publication rate in Hz (controller-manager stays at 1000 Hz)",
        ),
        DeclareLaunchArgument(
            "low_level_controller_start_delay", default_value="0.0",
            description="Deprecated compatibility argument; low-level startup now uses readiness detection",
        ),
        DeclareLaunchArgument("frequency", default_value="200.0"),
        DeclareLaunchArgument(
            "max_velocity",
            default_value="[4.0,4.0,4.0,4.0,3.0,3.0,3.0]",
            description="Joint velocity limits used by both IK and QP [rad/s]",
        ),
        DeclareLaunchArgument(
            "max_acceleration",
            default_value="[3.0,3.0,3.0,3.0,3.0,3.0,3.0]",
            description="Joint acceleration limits used by both IK and QP [rad/s^2]",
        ),
        DeclareLaunchArgument("joint_limit_margin", default_value="0.08"),
        DeclareLaunchArgument("repulsion_when_idle", default_value="true",
                              description="Wake QP without a twist only after a configured limit is violated"),
        DeclareLaunchArgument("idle_safety_frequency", default_value="50.0",
                              description="Surface violation check frequency while no Cartesian command is active"),
        DeclareLaunchArgument("configure_collision_behavior", default_value="false"),
        DeclareLaunchArgument("collision_torque_scale", default_value="1.0"),
        DeclareLaunchArgument("collision_force_scale", default_value="1.0"),
        DeclareLaunchArgument("twist_topic", default_value="/fr3/cartesian_twist_command"),
        DeclareLaunchArgument("joint_state_topic", default_value="/franka/joint_states"),
        DeclareLaunchArgument("joint_command_topic", default_value="/fr3/joint_commands"),
        DeclareLaunchArgument("tau_command_topic", default_value="/fr3/tau_command",
                              description="effort-only JointState output from ik_force to low_level torque mode"),
        DeclareLaunchArgument("force_input_mode", default_value="wrench",
                              description="ik_force input: wrench (WrenchStamped) or twist (TwistStamped + IK feedback)"),
        DeclareLaunchArgument("wrench_topic", default_value="/fr3/cartesian_wrench_command"),
        DeclareLaunchArgument("force_joint_state_timeout", default_value="0.2"),
        DeclareLaunchArgument("force_velocity_to_wrench_gains", default_value="[40.0,40.0,40.0,5.0,5.0,5.0]",
                              description="Cartesian damping gains: 3 N/(m/s) and 3 Nm/(rad/s)"),
        DeclareLaunchArgument("force_max_cartesian_force", default_value="15.0",
                              description="Cartesian force magnitude limit [N]"),
        DeclareLaunchArgument("force_max_cartesian_moment", default_value="2.0",
                              description="Cartesian moment magnitude limit [Nm]"),
        DeclareLaunchArgument("force_max_joint_torque", default_value="[8.0,8.0,8.0,6.0,4.0,4.0,4.0]",
                              description="Additional high-level uniform joint torque saturation [Nm]"),
        OpaqueFunction(function=_setup),
    ])
