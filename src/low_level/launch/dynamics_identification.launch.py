import ast
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(text):
    return text.strip().lower() in ('1', 'true', 'yes', 'on')


def _default_output_directory():
    try:
        prefix = Path(get_package_prefix('low_level'))
        parts = prefix.parts
        install_indices = [i for i, part in enumerate(parts) if part == 'install']
        if install_indices:
            workspace_root = Path(*parts[:install_indices[-1]])
            return str(workspace_root / 'data' / 'calibration')
    except Exception:  # noqa: BLE001
        pass
    return str(Path.cwd() / 'data' / 'calibration')


def _array(text, name, size):
    value = ast.literal_eval(text)
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise RuntimeError(f'{name} must contain exactly {size} numbers')
    return [float(x) for x in value]


def _positive_levels(text, name):
    value = ast.literal_eval(text)
    if not isinstance(value, (list, tuple)) or len(value) < 6:
        raise RuntimeError(f'{name} requires at least six positive speeds')
    speeds = [float(x) for x in value]
    if any(x <= 0 for x in speeds):
        raise RuntimeError(f'{name} requires positive speeds')
    return speeds


def _array7(text, name):
    return _array(text, name, 7)


def _array6(text, name):
    return _array(text, name, 6)


def _setup(context):
    namespace = LaunchConfiguration('namespace').perform(context)
    robot_type = LaunchConfiguration('robot_type').perform(context)
    arm_prefix = LaunchConfiguration('arm_prefix').perform(context)
    observation_topic = LaunchConfiguration('observation_topic').perform(context)
    command_topic = LaunchConfiguration('command_topic').perform(context)
    twist_topic = LaunchConfiguration('twist_topic').perform(context)
    joint_state_topic = LaunchConfiguration('joint_state_topic').perform(context)
    motion_mode = LaunchConfiguration('motion_mode').perform(context).strip().lower()
    if motion_mode not in ('joint', 'qp'):
        raise RuntimeError("motion_mode must be 'joint' or 'qp'")

    load_gripper = _as_bool(LaunchConfiguration('load_gripper').perform(context))
    expanded_robot_prefix = (f'{arm_prefix}_' if arm_prefix else '') + robot_type
    requested_qp_tip_link = LaunchConfiguration('qp_tip_link').perform(context).strip()
    qp_tip_link = requested_qp_tip_link or "auto"  # Resolve tip from /robot_description

    low_level_launch = os.path.join(
        get_package_share_directory('low_level'), 'launch', 'control.launch.py'
    )
    low_level = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(low_level_launch),
        launch_arguments={
            'mode': 'velocity',
            'robot_ip': LaunchConfiguration('robot_ip').perform(context),
            'robot_type': robot_type,
            'arm_prefix': arm_prefix,
            'namespace': namespace,
            'load_gripper': str(load_gripper).lower(),
            'use_fake_hardware': 'false',
            'load_franka_robot_state_broadcaster': 'false',
            'joint_state_rate': LaunchConfiguration('joint_state_rate').perform(context),
            'command_topic': command_topic,
            'tracking_time_constant': LaunchConfiguration('low_level_tracking_time_constant').perform(context),
            'max_acceleration': LaunchConfiguration('low_level_max_acceleration').perform(context),
            'max_jerk': LaunchConfiguration('low_level_max_jerk').perform(context),
            'dynamics_observation_topic': observation_topic,
            'dynamics_publish_rate': LaunchConfiguration('observer_publish_rate').perform(context),
            'dynamics_acceleration_filter_tau': LaunchConfiguration('acceleration_filter_tau').perform(context),
            'readiness_stable_time': LaunchConfiguration('readiness_stable_time').perform(context),
            'readiness_warn_after': LaunchConfiguration('readiness_warn_after').perform(context),
        }.items(),
    )

    velocity_ready = Node(
        package='low_level',
        executable='readiness_gate.py',
        name='identification_velocity_ready',
        namespace=namespace,
        output='screen',
        parameters=[{
            'required_true_topics': ['low_level_joint_velocity_controller/ready'],
            'required_publisher_topics': [joint_state_topic],
            'stable_time': float(LaunchConfiguration('readiness_stable_time').perform(context)),
            'warn_after': float(LaunchConfiguration('readiness_warn_after').perform(context)),
        }],
    )

    observer_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        output='screen',
        arguments=['dynamics_observer_controller', '--controller-manager', 'controller_manager'],
    )

    observer_ready = Node(
        package='low_level',
        executable='readiness_gate.py',
        name='identification_observer_ready',
        namespace=namespace,
        output='screen',
        parameters=[{
            'required_true_topics': ['dynamics_observer_controller/ready'],
            'required_publisher_topics': [observation_topic],
            'stable_time': float(LaunchConfiguration('readiness_stable_time').perform(context)),
            'warn_after': float(LaunchConfiguration('readiness_warn_after').perform(context)),
        }],
    )

    selected_guard = (
        float(LaunchConfiguration('qp_max_joint_displacement').perform(context))
        if motion_mode == 'qp'
        else float(LaunchConfiguration('joint_max_joint_displacement').perform(context))
    )

    experiment = Node(
        package='low_level',
        executable='dynamics_identification_experiment.py',
        name='dynamics_identification_experiment',
        namespace=namespace,
        output='screen',
        parameters=[{
            'robot_type': robot_type,
            'arm_prefix': arm_prefix,
            'motion_mode': motion_mode,
            'command_topic': command_topic,
            'twist_topic': twist_topic,
            'observation_topic': observation_topic,
            'root_frame': f'{arm_prefix + "_" if arm_prefix else ""}{robot_type}_link0',
            'run_motion': _as_bool(LaunchConfiguration('run_motion').perform(context)),
            'command_rate': float(LaunchConfiguration('command_rate').perform(context)),
            'settle_time': float(LaunchConfiguration('settle_time').perform(context)),
            'joint_duration': float(LaunchConfiguration('joint_duration').perform(context)),
            'qp_axis_duration': float(LaunchConfiguration('qp_axis_duration').perform(context)),
            'qp_mixed_duration': float(LaunchConfiguration('qp_mixed_duration').perform(context)),
            'stop_time': float(LaunchConfiguration('stop_time').perform(context)),
            'record_only_duration': float(LaunchConfiguration('record_only_duration').perform(context)),
            'velocity_amplitudes': _array7(
                LaunchConfiguration('velocity_amplitudes').perform(context), 'velocity_amplitudes'
            ),
            'frequencies': _array7(
                LaunchConfiguration('frequencies').perform(context), 'frequencies'
            ),
            'qp_twist_amplitudes': _array6(
                LaunchConfiguration('qp_twist_amplitudes').perform(context), 'qp_twist_amplitudes'
            ),
            'qp_axis_frequencies': _array6(
                LaunchConfiguration('qp_axis_frequencies').perform(context), 'qp_axis_frequencies'
            ),
            'qp_mixed_scale': float(LaunchConfiguration('qp_mixed_scale').perform(context)),
            'max_joint_displacement': selected_guard,
            'max_measured_joint_velocity': float(
                LaunchConfiguration('max_measured_joint_velocity').perform(context)
            ),
            'max_measured_joint_acceleration': float(
                LaunchConfiguration('max_measured_joint_acceleration').perform(context)
            ),
            'observation_timeout': float(LaunchConfiguration('observation_timeout').perform(context)),
            'friction_velocity_levels': _positive_levels(
                LaunchConfiguration('friction_velocity_levels').perform(context),
                'friction_velocity_levels'),
            'friction_repeats': int(LaunchConfiguration('friction_repeats').perform(context)),
            'friction_ramp_time': float(LaunchConfiguration('friction_ramp_time').perform(context)),
            'friction_hold_time': float(LaunchConfiguration('friction_hold_time').perform(context)),
            'friction_max_hold_time': float(LaunchConfiguration('friction_max_hold_time').perform(context)),
            'friction_min_plateau_travel_rad': float(
                LaunchConfiguration('friction_min_plateau_travel_rad').perform(context)),
            'friction_min_fit_velocity': float(
                LaunchConfiguration('friction_min_fit_velocity').perform(context)),
            'friction_speed_tolerance_ratio': float(
                LaunchConfiguration('friction_speed_tolerance_ratio').perform(context)),
            'friction_other_joint_max_drift_rad': float(
                LaunchConfiguration('friction_other_joint_max_drift_rad').perform(context)),
            'friction_pause_time': float(LaunchConfiguration('friction_pause_time').perform(context)),
            'friction_plateau_trim_time': float(LaunchConfiguration('friction_plateau_trim_time').perform(context)),
            'friction_max_fit_acceleration': float(LaunchConfiguration('friction_max_fit_acceleration').perform(context)),
            'friction_pair_grid_points': int(LaunchConfiguration('friction_pair_grid_points').perform(context)),
            'friction_min_pair_samples': int(LaunchConfiguration('friction_min_pair_samples').perform(context)),
            'friction_smoothing_velocity': float(
                LaunchConfiguration('friction_smoothing_velocity').perform(context)
            ),
            'output_directory': LaunchConfiguration('output_directory').perform(context),
        }],
    )

    actions = [low_level, velocity_ready]

    if motion_mode == 'qp':
        qp_config = os.path.join(
            get_package_share_directory('franka_cartesian_control'),
            'config',
            'controllers.yaml',
        )
        qp_node = Node(
            package='franka_cartesian_control',
            executable='qp_velocity_controller',
            name='qp_velocity_controller',
            namespace=namespace,
            output='screen',
            parameters=[qp_config, {
                'robot_type': robot_type,
                'arm_prefix': arm_prefix,
                'root_link': f'{expanded_robot_prefix}_link0',
                'tip_link': qp_tip_link,
                'twist_topic': twist_topic,
                'joint_state_topic': joint_state_topic,
                'joint_command_topic': command_topic,
                'frequency': float(LaunchConfiguration('qp_frequency').perform(context)),
                'max_velocity': _array7(
                    LaunchConfiguration('qp_max_velocity').perform(context), 'qp_max_velocity'
                ),
                'max_acceleration': _array7(
                    LaunchConfiguration('qp_max_acceleration').perform(context), 'qp_max_acceleration'
                ),
                'joint_limit_margin': float(
                    LaunchConfiguration('qp_joint_limit_margin').perform(context)
                ),
                'task_weights': _array6(
                    LaunchConfiguration('qp_task_weights').perform(context), 'qp_task_weights'
                ),
                'regularization': float(LaunchConfiguration('qp_regularization').perform(context)),
                'posture_weight': float(LaunchConfiguration('qp_posture_weight').perform(context)),
                'posture_gain': float(LaunchConfiguration('qp_posture_gain').perform(context)),
                'repulsion_when_idle': False,
            }],
        )
        qp_ready = Node(
            package='low_level',
            executable='readiness_gate.py',
            name='identification_qp_ready',
            namespace=namespace,
            output='screen',
            parameters=[{
                'required_true_topics': ['qp_velocity_controller/ready'],
                'required_publisher_topics': [joint_state_topic],
                'stable_time': float(LaunchConfiguration('readiness_stable_time').perform(context)),
                'warn_after': float(LaunchConfiguration('readiness_warn_after').perform(context)),
            }],
        )
        actions.extend([
            RegisterEventHandler(OnProcessExit(target_action=velocity_ready, on_exit=[qp_node])),
            RegisterEventHandler(OnProcessExit(target_action=velocity_ready, on_exit=[qp_ready])),
            RegisterEventHandler(OnProcessExit(target_action=qp_ready, on_exit=[observer_spawner])),
        ])
    else:
        actions.append(
            RegisterEventHandler(OnProcessExit(target_action=velocity_ready, on_exit=[observer_spawner]))
        )

    actions.extend([
        RegisterEventHandler(OnProcessExit(target_action=observer_spawner, on_exit=[observer_ready])),
        RegisterEventHandler(OnProcessExit(target_action=observer_ready, on_exit=[experiment])),
        RegisterEventHandler(
            OnProcessExit(
                target_action=experiment,
                on_exit=[EmitEvent(event=Shutdown(reason='Dynamics identification experiment completed'))],
            )
        ),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.16.0.1'),
        DeclareLaunchArgument('robot_type', default_value='fr3'),
        DeclareLaunchArgument('arm_prefix', default_value=''),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('load_gripper', default_value='false'),
        DeclareLaunchArgument(
            'qp_tip_link', default_value='auto',
            description=(
                'QP TCP frame. auto resolves the tip from the loaded /robot_description.'
            ),
        ),
        DeclareLaunchArgument('joint_state_rate', default_value='250'),
        DeclareLaunchArgument('joint_state_topic', default_value='/franka/joint_states'),
        DeclareLaunchArgument('command_topic', default_value='/fr3/joint_commands'),
        DeclareLaunchArgument('twist_topic', default_value='/fr3/cartesian_twist_command'),
        DeclareLaunchArgument('observation_topic', default_value='/fr3/dynamics_observation'),
        DeclareLaunchArgument('observer_publish_rate', default_value='200.0'),
        DeclareLaunchArgument('acceleration_filter_tau', default_value='0.02'),
        DeclareLaunchArgument('readiness_stable_time', default_value='0.25'),
        DeclareLaunchArgument('readiness_warn_after', default_value='5.0'),
        DeclareLaunchArgument(
            'motion_mode', default_value='joint',
            description="Existing experiment motion source: 'joint' or 'qp'",
        ),
        DeclareLaunchArgument(
            'run_motion', default_value='false',
            description='Must be explicitly true to command identification motion',
        ),
        DeclareLaunchArgument('command_rate', default_value='100.0'),
        DeclareLaunchArgument('settle_time', default_value='2.0'),
        DeclareLaunchArgument('joint_duration', default_value='8.0'),
        DeclareLaunchArgument('qp_axis_duration', default_value='10.0'),
        DeclareLaunchArgument('qp_mixed_duration', default_value='12.0'),
        DeclareLaunchArgument('stop_time', default_value='2.0'),
        DeclareLaunchArgument('record_only_duration', default_value='15.0'),
        DeclareLaunchArgument(
            'velocity_amplitudes', default_value='[0.06,0.06,0.06,0.05,0.08,0.08,0.08]'
        ),
        DeclareLaunchArgument(
            'frequencies', default_value='[0.13,0.15,0.17,0.12,0.19,0.21,0.23]'
        ),
        DeclareLaunchArgument(
            'qp_twist_amplitudes',
            default_value='[0.08,0.08,0.06,0.16,0.16,0.16]',
            description='QP sequential Cartesian [vx,vy,vz,wx,wy,wz] amplitudes',
        ),
        DeclareLaunchArgument(
            'qp_axis_frequencies', default_value='[0.20,0.20,0.20,0.20,0.20,0.20]'
        ),
        DeclareLaunchArgument('qp_mixed_scale', default_value='0.30'),
        DeclareLaunchArgument('joint_max_joint_displacement', default_value='0.20'),
        DeclareLaunchArgument('qp_max_joint_displacement', default_value='0.35'),
        DeclareLaunchArgument('max_measured_joint_velocity', default_value='0.35'),
        DeclareLaunchArgument('max_measured_joint_acceleration', default_value='3.0'),
        DeclareLaunchArgument('observation_timeout', default_value='0.15'),
        DeclareLaunchArgument(
            'friction_velocity_levels',
            default_value='[0.008,0.012,0.02,0.03,0.045,0.065,0.10,0.15]',
            description='Absolute speed levels used for one-start-pose +/- sweeps',
        ),
        DeclareLaunchArgument(
            'friction_repeats',
            default_value='4',
            description='Number of complete friction sweeps per joint; alternating order exposes repeatability/drift',
        ),
        DeclareLaunchArgument('friction_ramp_time', default_value='0.35'),
        DeclareLaunchArgument('friction_hold_time', default_value='0.45'),
        DeclareLaunchArgument('friction_max_hold_time', default_value='2.0'),
        DeclareLaunchArgument('friction_min_plateau_travel_rad', default_value='0.01'),
        DeclareLaunchArgument('friction_min_fit_velocity', default_value='0.004'),
        DeclareLaunchArgument('friction_speed_tolerance_ratio', default_value='0.35'),
        DeclareLaunchArgument('friction_other_joint_max_drift_rad', default_value='0.04'),
        DeclareLaunchArgument('friction_pause_time', default_value='0.15'),
        DeclareLaunchArgument('friction_plateau_trim_time', default_value='0.10'),
        DeclareLaunchArgument(
            'friction_max_fit_acceleration', default_value='0.05',
            description='Only samples with |ddq| below this threshold are eligible for friction pairing',
        ),
        DeclareLaunchArgument('friction_pair_grid_points', default_value='15'),
        DeclareLaunchArgument('friction_min_pair_samples', default_value='6'),
        DeclareLaunchArgument(
            'friction_smoothing_velocity',
            default_value='0.01',
            description='Velocity epsilon for smoothing only the Coulomb term in the fitted/runtime model',
        ),
        DeclareLaunchArgument(
            'output_directory',
            default_value=_default_output_directory(),
            description='Directory for calibration CSV/JSON output (default: <workspace>/data/calibration)',
        ),
        DeclareLaunchArgument('qp_frequency', default_value='200.0'),
        DeclareLaunchArgument(
            'qp_max_velocity', default_value='[0.8,0.8,0.8,0.8,1.2,1.2,1.2]'
        ),
        DeclareLaunchArgument(
            'qp_max_acceleration', default_value='[1.5,1.5,1.5,1.5,2.0,2.0,2.0]'
        ),
        DeclareLaunchArgument('qp_joint_limit_margin', default_value='0.15'),
        DeclareLaunchArgument('qp_task_weights', default_value='[1.0,1.0,1.0,1.0,1.0,1.0]'),
        DeclareLaunchArgument('qp_regularization', default_value='0.001'),
        DeclareLaunchArgument('qp_posture_weight', default_value='0.03'),
        DeclareLaunchArgument('qp_posture_gain', default_value='0.30'),
        DeclareLaunchArgument('low_level_tracking_time_constant', default_value='0.03'),
        DeclareLaunchArgument(
            'low_level_max_acceleration', default_value='[3.0,3.0,3.0,3.0,3.5,3.5,3.5]'
        ),
        DeclareLaunchArgument(
            'low_level_max_jerk', default_value='[150.0,150.0,150.0,150.0,180.0,180.0,180.0]'
        ),
        OpaqueFunction(function=_setup),
    ])
