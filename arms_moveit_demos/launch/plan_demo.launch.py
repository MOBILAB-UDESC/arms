from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.actions import Node


def generate_launch_description():
    """Spawn a demo node."""
    this_pkg = get_package_share_directory('arms_moveit_demos')

    demo_spawn_node = Node(
        package='arms_moveit_demos',
        executable='plan_demo_node',
        output='screen',
        parameters=[LaunchConfiguration('demo_config_file')],
    )

    args = [
        DeclareLaunchArgument(
            name='demo_config_file',
            default_value=PathJoinSubstitution([this_pkg, 'config', 'demos.yaml']),
            description='Absolute path to demos configuration YAML file'
        )
    ]

    return LaunchDescription([
        *args,
        demo_spawn_node,
    ])
