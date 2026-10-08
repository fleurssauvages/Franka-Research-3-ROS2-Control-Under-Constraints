from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_data_dir():
    try:
        prefix = Path(get_package_prefix('udp_bridge'))
        if prefix.parent.name == 'install':
            return str(prefix.parent.parent / 'data' / 'udp')
        parts = prefix.parts
        indices = [i for i, value in enumerate(parts) if value == 'install']
        if indices:
            return str(Path(*parts[:indices[-1]]) / 'data' / 'udp')
    except Exception:
        pass
    return str(Path.cwd() / 'data' / 'udp')


def generate_launch_description():
    data_dir = _default_data_dir()
    reader_default = str(Path(data_dir) / 'udp_reader.json')
    publisher_default = str(Path(data_dir) / 'udp_publisher.json')

    return LaunchDescription([
        DeclareLaunchArgument('enable_reader', default_value='true'),
        DeclareLaunchArgument('enable_publisher', default_value='true'),
        DeclareLaunchArgument('reader_config', default_value=reader_default),
        DeclareLaunchArgument('publisher_config', default_value=publisher_default),
        Node(
            package='udp_bridge',
            executable='udp_to_ros',
            name='udp_to_ros',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_reader')),
            parameters=[{'config_file': LaunchConfiguration('reader_config')}],
        ),
        Node(
            package='udp_bridge',
            executable='ros_to_udp',
            name='ros_to_udp',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_publisher')),
            parameters=[{'config_file': LaunchConfiguration('publisher_config')}],
        ),
    ])
