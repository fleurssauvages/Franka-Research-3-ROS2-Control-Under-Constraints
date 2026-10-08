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


def _setup(context):
    package_share = get_package_share_directory("franka_cartesian_control")
    controller = LaunchConfiguration("controller").perform(context).strip().lower()
    if controller not in ("ik", "qp"):
        raise RuntimeError("controller must be 'ik' or 'qp'")

    config_file = LaunchConfiguration("config_file").perform(context)
    robot_type = LaunchConfiguration("robot_type").perform(context)
    arm_prefix = LaunchConfiguration("arm_prefix").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    twist_topic = LaunchConfiguration("twist_topic").perform(context)
    joint_state_topic = LaunchConfiguration("joint_state_topic").perform(context)
    joint_command_topic = LaunchConfiguration("joint_command_topic").perform(context)
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
    load_gripper = _as_bool(LaunchConfiguration("load_gripper").perform(context))
    requested_tip_link = LaunchConfiguration("tip_link").perform(context).strip()
    low_level_mode = LaunchConfiguration("low_level_mode").perform(context).strip().lower()
    if low_level_mode not in ("velocity", "impedance"):
        raise RuntimeError("low_level_mode must be 'velocity' or 'impedance'")

    executable = "ik_velocity_controller" if controller == "ik" else "qp_velocity_controller"
    node_parameters = {
        "robot_type": robot_type,
        "arm_prefix": arm_prefix,
        "load_gripper": load_gripper,
        "twist_topic": twist_topic,
        "joint_state_topic": joint_state_topic,
        "joint_command_topic": joint_command_topic,
        "frequency": frequency,
        "joint_limit_margin": joint_limit_margin,
        "repulsion_when_idle": repulsion_when_idle,
        "idle_safety_frequency": idle_safety_frequency,
    }
    # Launch value overrides any stale YAML tip. auto requests strict URDF-based
    # auto-resolution instead of trusting load_gripper or an obsolete YAML default.
    node_parameters["tip_link"] = requested_tip_link or "auto"

    # One shared set of joint motion limits is applied to both IK and QP.
    # This avoids the previous hidden behavior where QP silently kept the YAML
    # 0.3 rad/s^2 limit while launch arguments only changed IK.
    node_parameters["max_velocity"] = max_velocity
    node_parameters["max_acceleration"] = max_acceleration

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
                    "load_gripper": LaunchConfiguration("load_gripper").perform(context),
                    "use_fake_hardware": LaunchConfiguration("use_fake_hardware").perform(context),
                    "load_franka_robot_state_broadcaster": LaunchConfiguration("load_franka_robot_state_broadcaster").perform(context),
                    "joint_state_rate": LaunchConfiguration("joint_state_rate").perform(context),
                    "controller_start_delay": LaunchConfiguration("low_level_controller_start_delay").perform(context),
                    "readiness_stable_time": LaunchConfiguration("readiness_stable_time").perform(context),
                    "readiness_warn_after": LaunchConfiguration("readiness_warn_after").perform(context),
                    "command_topic": joint_command_topic,
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
        DeclareLaunchArgument("controller", default_value="qp", description="ik or qp"),
        DeclareLaunchArgument("config_file", default_value=default_config),
        DeclareLaunchArgument("robot_type", default_value="fr3"),
        DeclareLaunchArgument("arm_prefix", default_value=""),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("robot_ip", default_value="192.16.0.1"),
        DeclareLaunchArgument("start_low_level", default_value="true"),
        DeclareLaunchArgument("high_level_start_delay", default_value="0.0",
                              description="Deprecated compatibility argument; startup now uses readiness detection"),
        DeclareLaunchArgument("readiness_stable_time", default_value="0.25",
                              description="Readiness conditions must stay true for this long before startup continues"),
        DeclareLaunchArgument("readiness_warn_after", default_value="5.0",
                              description="Seconds before readiness gates log missing conditions"),
        DeclareLaunchArgument("low_level_mode", default_value="velocity", description="velocity (default) or impedance"),
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
        DeclareLaunchArgument("impedance_damping", default_value="[2.0,2.0,2.0,2.0,2.0,2.0,2.0]"),
        DeclareLaunchArgument("impedance_mass_damping", default_value="[5.0,5.0,5.0,5.0,5.0,5.0,5.0]"),
        DeclareLaunchArgument("impedance_delta_tau_max", default_value="1.0"),
        DeclareLaunchArgument("impedance_max_torque", default_value="[0.0,0.0,0.0,0.0,0.0,0.0,0.0]"),
        DeclareLaunchArgument("impedance_hold_position_on_timeout", default_value="false"),
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
        DeclareLaunchArgument("load_gripper", default_value="false",
                              description="Include Franka hand in URDF bringup; tip is resolved from /robot_description"),
        DeclareLaunchArgument("tip_link", default_value="auto",
                              description="auto resolves a seven-joint TCP from /robot_description; otherwise specify a URDF link"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument(
            "load_franka_robot_state_broadcaster", default_value="false",
            description="Disable the optional 1 kHz full-state broadcaster by default for RT headroom",
        ),
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
        OpaqueFunction(function=_setup),
    ])
