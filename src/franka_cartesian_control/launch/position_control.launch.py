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
    position_controller = LaunchConfiguration("position_controller").perform(context).strip().lower()
    velocity_controller = LaunchConfiguration("velocity_controller").perform(context).strip().lower()
    if position_controller not in ("pid", "lmpc"):
        raise RuntimeError("position_controller must be 'pid' or 'lmpc'")
    if velocity_controller not in ("ik", "qp"):
        raise RuntimeError("velocity_controller must be 'ik' or 'qp'")

    config_file = LaunchConfiguration("position_config_file").perform(context)
    executable = "pid_position_controller" if position_controller == "pid" else "lmpc_position_controller"

    low_level_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "cartesian_control.launch.py")
        ),
        launch_arguments={
            "controller": velocity_controller,
            "robot_type": LaunchConfiguration("robot_type").perform(context),
            "arm_prefix": LaunchConfiguration("arm_prefix").perform(context),
            "namespace": LaunchConfiguration("namespace").perform(context),
            "robot_ip": LaunchConfiguration("robot_ip").perform(context),
            "start_low_level": LaunchConfiguration("start_low_level").perform(context),
            "low_level_mode": LaunchConfiguration("low_level_mode").perform(context),
            "load_gripper": LaunchConfiguration("load_gripper").perform(context),
            "tip_link": LaunchConfiguration("tip_link").perform(context),
            "use_fake_hardware": LaunchConfiguration("use_fake_hardware").perform(context),
            "load_franka_robot_state_broadcaster": LaunchConfiguration("load_franka_robot_state_broadcaster").perform(context),
            "joint_state_rate": LaunchConfiguration("joint_state_rate").perform(context),
            "low_level_controller_start_delay": LaunchConfiguration("low_level_controller_start_delay").perform(context),
            "readiness_stable_time": LaunchConfiguration("readiness_stable_time").perform(context),
            "readiness_warn_after": LaunchConfiguration("readiness_warn_after").perform(context),
            "low_level_tracking_time_constant": LaunchConfiguration("low_level_tracking_time_constant").perform(context),
            "low_level_max_acceleration": LaunchConfiguration("low_level_max_acceleration").perform(context),
            "low_level_max_jerk": LaunchConfiguration("low_level_max_jerk").perform(context),
            "impedance_stiffness": LaunchConfiguration("impedance_stiffness").perform(context),
            "impedance_damping": LaunchConfiguration("impedance_damping").perform(context),
            "impedance_mass_damping": LaunchConfiguration("impedance_mass_damping").perform(context),
            "impedance_delta_tau_max": LaunchConfiguration("impedance_delta_tau_max").perform(context),
            "impedance_max_torque": LaunchConfiguration("impedance_max_torque").perform(context),
            "impedance_hold_position_on_timeout": LaunchConfiguration("impedance_hold_position_on_timeout").perform(context),
            "impedance_friction_compensation_enabled": LaunchConfiguration("impedance_friction_compensation_enabled").perform(context),
            "impedance_friction_calibration_directory": LaunchConfiguration("impedance_friction_calibration_directory").perform(context),
            "impedance_friction_calibration_file": LaunchConfiguration("impedance_friction_calibration_file").perform(context),
            "impedance_friction_calibration_max_age_hours": LaunchConfiguration("impedance_friction_calibration_max_age_hours").perform(context),
            "impedance_friction_use_recommended_enable": LaunchConfiguration("impedance_friction_use_recommended_enable").perform(context),
            "impedance_friction_apply_torque_bias": LaunchConfiguration("impedance_friction_apply_torque_bias").perform(context),
            "impedance_friction_compensation_scale": LaunchConfiguration("impedance_friction_compensation_scale").perform(context),
            "impedance_friction_stribeck_enabled": LaunchConfiguration("impedance_friction_stribeck_enabled").perform(context),
            "impedance_friction_max_compensation_torque": LaunchConfiguration("impedance_friction_max_compensation_torque").perform(context),
            "configure_collision_behavior": LaunchConfiguration("configure_collision_behavior").perform(context),
            "collision_torque_scale": LaunchConfiguration("collision_torque_scale").perform(context),
            "collision_force_scale": LaunchConfiguration("collision_force_scale").perform(context),
            "twist_topic": LaunchConfiguration("twist_topic").perform(context),
            "max_velocity": LaunchConfiguration("max_velocity").perform(context),
            "max_acceleration": LaunchConfiguration("max_acceleration").perform(context),
        }.items(),
    )

    load_gripper = LaunchConfiguration("load_gripper").perform(context).strip().lower() in (
        "1", "true", "yes", "on"
    )
    requested_tip_link = LaunchConfiguration("tip_link").perform(context).strip()
    controller_parameters = {
        "robot_type": LaunchConfiguration("robot_type").perform(context),
        "arm_prefix": LaunchConfiguration("arm_prefix").perform(context),
        "load_gripper": load_gripper,
        "joint_state_topic": LaunchConfiguration("joint_state_topic").perform(context),
        "desired_pose_topic": LaunchConfiguration("desired_pose_topic").perform(context),
        "twist_topic": LaunchConfiguration("twist_topic").perform(context),
        "current_pose_topic": LaunchConfiguration("current_pose_topic").perform(context),
        "frequency": float(LaunchConfiguration("position_frequency").perform(context)),
        # Desired poses must be continuously refreshed. On timeout the position
        # controller stops publishing TwistStamped commands.
        "command_timeout": float(LaunchConfiguration("pose_watchdog_timeout").perform(context)),
    }

    controller_parameters["tip_link"] = requested_tip_link or "auto"  # URDF auto resolution

    if position_controller == "pid":
        controller_parameters.update({
            "kp": _array6(LaunchConfiguration("pid_kp").perform(context), "pid_kp"),
            "ki": _array6(LaunchConfiguration("pid_ki").perform(context), "pid_ki"),
            "kd": _array6(LaunchConfiguration("pid_kd").perform(context), "pid_kd"),
            "max_twist": _array6(
                LaunchConfiguration("pid_max_twist").perform(context), "pid_max_twist"
            ),
        })
    else:
        controller_parameters.update({
            "regularization_gain": float(
                LaunchConfiguration("lmpc_regularization_gain").perform(context)
            ),
            "delta_twist_weight": float(
                LaunchConfiguration("lmpc_delta_twist_weight").perform(context)
            ),
            "velocity_tracking_time_constant": float(
                LaunchConfiguration("lmpc_velocity_tracking_time_constant").perform(context)
            ),
            "max_twist": _array6(
                LaunchConfiguration("lmpc_max_twist").perform(context), "lmpc_max_twist"
            ),
            "max_acceleration": _array6(
                LaunchConfiguration("lmpc_max_acceleration").perform(context), "lmpc_max_acceleration"
            ),
            "velocity_weights": _array6(
                LaunchConfiguration("lmpc_velocity_weights").perform(context), "lmpc_velocity_weights"
            ),
            "terminal_velocity_weights": _array6(
                LaunchConfiguration("lmpc_terminal_velocity_weights").perform(context),
                "lmpc_terminal_velocity_weights",
            ),
        })

    position_node = Node(
        package="franka_cartesian_control",
        executable=executable,
        name=executable,
        namespace=LaunchConfiguration("namespace").perform(context),
        output="screen",
        parameters=[config_file, controller_parameters],
    )

    velocity_executable = (
        "ik_velocity_controller" if velocity_controller == "ik" else "qp_velocity_controller"
    )
    position_gate = Node(
        package="low_level",
        executable="readiness_gate.py",
        name="position_velocity_ready",
        namespace=LaunchConfiguration("namespace").perform(context),
        output="screen",
        parameters=[{
            "required_true_topics": [f"{velocity_executable}/ready"],
            "stable_time": float(LaunchConfiguration("readiness_stable_time").perform(context)),
            "warn_after": float(LaunchConfiguration("readiness_warn_after").perform(context)),
        }],
    )
    return [
        low_level_stack,
        position_gate,
        RegisterEventHandler(
            OnProcessExit(target_action=position_gate, on_exit=[position_node])
        ),
    ]


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("franka_cartesian_control"),
        "config",
        "position_controllers.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("position_controller", default_value="pid", description="pid or lmpc"),
        DeclareLaunchArgument("velocity_controller", default_value="qp", description="ik or qp"),
        DeclareLaunchArgument("position_config_file", default_value=default_config),
        DeclareLaunchArgument("position_start_delay", default_value="0.0",
                              description="Deprecated compatibility argument; position startup now waits for IK/QP ready"),
        DeclareLaunchArgument("readiness_stable_time", default_value="0.25",
                              description="Readiness conditions must stay true for this long before startup continues"),
        DeclareLaunchArgument("readiness_warn_after", default_value="5.0",
                              description="Seconds before readiness gates log missing conditions"),
        DeclareLaunchArgument("position_frequency", default_value="50.0"),
        DeclareLaunchArgument(
            "max_velocity",
            default_value="[1.0,1.0,1.0,1.0,1.5,1.5,1.5]",
            description="Joint velocity limits used by both IK and QP [rad/s]",
        ),
        DeclareLaunchArgument(
            "max_acceleration",
            default_value="[1.5,1.5,1.5,1.5,1.5,1.5,1.5]",
            description="Joint acceleration limits used by both IK and QP [rad/s^2]",
        ),
        DeclareLaunchArgument(
            "pose_watchdog_timeout",
            default_value="0.50",
            description="Seconds without a desired PoseStamped before PID/LMPC stops commanding; must be > 0",
        ),
        DeclareLaunchArgument(
            "pid_kp",
            default_value="[2.0, 2.0, 2.0, 2.0, 2.0, 2.0]",
            description="Cartesian PID proportional gains [x,y,z,rx,ry,rz]",
        ),
        DeclareLaunchArgument(
            "pid_ki",
            default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
            description="Cartesian PID integral gains [x,y,z,rx,ry,rz]",
        ),
        DeclareLaunchArgument(
            "pid_kd",
            default_value="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]",
            description="Cartesian PID derivative gains [x,y,z,rx,ry,rz]",
        ),
        DeclareLaunchArgument(
            "pid_max_twist",
            default_value="[0.5, 0.5, 0.5, 0.20, 0.20, 0.20]",
            description="PID Cartesian twist limits [vx,vy,vz,wx,wy,wz] in m/s and rad/s",
        ),
        DeclareLaunchArgument(
            "lmpc_regularization_gain",
            default_value="0.00001",
            description="LMPC desired-twist regularization; larger values reduce commanded Cartesian speed.",
        ),
        DeclareLaunchArgument(
            "lmpc_delta_twist_weight",
            default_value="0.3",
            description="LMPC penalty on changes in desired Cartesian twist between horizon steps.",
        ),
        DeclareLaunchArgument(
            "lmpc_velocity_tracking_time_constant",
            default_value="0.05",
            description="First-order LMPC actuator time constant [s]; increase if measured TCP velocity responds more slowly than commanded twist.",
        ),
        DeclareLaunchArgument(
            "lmpc_max_twist",
            default_value="[8.0, 8.0, 8.0, 1.0, 1.0, 1.0]",
            description="LMPC Cartesian twist limits [vx,vy,vz,wx,wy,wz] in m/s and rad/s",
        ),
        DeclareLaunchArgument(
            "lmpc_max_acceleration",
            default_value="[16.0, 16.0, 16.0, 8.0, 8.0, 8.0]",
            description="LMPC full-horizon desired-twist slew limits [m/s^2, rad/s^2]",
        ),
        DeclareLaunchArgument(
            "lmpc_velocity_weights",
            default_value="[0.5, 0.5, 0.5, 0.5, 0.5, 0.5]",
            description="LMPC running weights on predicted actual TCP velocity",
        ),
        DeclareLaunchArgument(
            "lmpc_terminal_velocity_weights",
            default_value="[50.0, 50.0, 50.0, 50.0, 50.0, 50.0]",
            description="LMPC terminal actual-velocity weights; high values force planned arrival with little residual speed",
        ),
        DeclareLaunchArgument("robot_type", default_value="fr3"),
        DeclareLaunchArgument("arm_prefix", default_value=""),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("robot_ip", default_value="192.16.0.1"),
        DeclareLaunchArgument("start_low_level", default_value="true"),
        DeclareLaunchArgument("low_level_mode", default_value="velocity"),
        DeclareLaunchArgument("load_gripper", default_value="false",
                              description="Include Franka hand in URDF bringup; Cartesian tip is determined from /robot_description"),
        DeclareLaunchArgument("tip_link", default_value="auto",
                              description="auto resolves the 7-DOF TCP from /robot_description; otherwise specify a link name"),
        DeclareLaunchArgument("use_fake_hardware", default_value="false"),
        DeclareLaunchArgument(
            "load_franka_robot_state_broadcaster", default_value="false",
            description="Disable the optional 1 kHz Franka full-state broadcaster by default for RT headroom",
        ),
        DeclareLaunchArgument(
            "joint_state_rate", default_value="250",
            description="Joint-state publication rate in Hz (controller-manager stays at 1000 Hz)",
        ),
        DeclareLaunchArgument(
            "low_level_controller_start_delay", default_value="0.0",
            description="Deprecated compatibility argument; low-level startup now uses readiness detection",
        ),
        DeclareLaunchArgument(
            "low_level_tracking_time_constant", default_value="0.03",
            description="Low-level velocity tracking time constant in seconds; larger is quieter/smoother",
        ),
        DeclareLaunchArgument(
            "low_level_max_acceleration",
            default_value="[2.0,2.0,2.0,2.0,2.0,2.0,2.0]",
            description="1 kHz low-level interpolation acceleration limits [rad/s^2]",
        ),
        DeclareLaunchArgument(
            "low_level_max_jerk",
            default_value="[100.0,100.0,100.0,100.0,100.0,100.0,100.0]",
            description="1 kHz low-level tracking jerk limits [rad/s^3]",
        ),
        DeclareLaunchArgument("impedance_stiffness", default_value="[200.0,200.0,200.0,200.0,100.0,100.0,100.0]"),
        DeclareLaunchArgument("impedance_damping", default_value="[30.0,30.0,30.0,30.0,20.0,20.0,20.0]"),
        DeclareLaunchArgument("impedance_mass_damping", default_value="[30.0,30.0,30.0,30.0,20.0,20.0,20.0]"),
        DeclareLaunchArgument("impedance_delta_tau_max", default_value="1.00"),
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
        DeclareLaunchArgument("configure_collision_behavior", default_value="false"),
        DeclareLaunchArgument("collision_torque_scale", default_value="3.0"),
        DeclareLaunchArgument("collision_force_scale", default_value="3.0"),
        DeclareLaunchArgument("joint_state_topic", default_value="/franka/joint_states"),
        DeclareLaunchArgument("desired_pose_topic", default_value="/fr3/cartesian_pose_command"),
        DeclareLaunchArgument("twist_topic", default_value="/fr3/cartesian_twist_command"),
        DeclareLaunchArgument("current_pose_topic", default_value="/fr3/cartesian_pose"),
        OpaqueFunction(function=_setup),
    ])
