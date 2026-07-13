import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch_ros.actions import Node


# Custom worlds in worlds directory.
AVAILABLE_WORLDS = ['empty', 'mobilab', 'pick_and_place', 'playground']
DEFAULT_WORLD = 'playground'


def generate_launch_description():
    """Launch Gazebo world and bridge clock."""
    bringup_pkg_path = get_package_share_directory('arms_bringup')

    ament_prefix_path = os.getenv('AMENT_PREFIX_PATH', '')
    packages_paths = [os.path.join(p, 'share') for p in ament_prefix_path.split(':')]

    # Ensure gazebo can find models.
    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            ':'.join(packages_paths),
            ':',
            os.getenv('GZ_SIM_RESOURCE_PATH', '')
        ]
    )

    # Save world path if specified, otherwise use one of the available worlds.
    gazebo_world_path = PythonExpression([
        "'",
        LaunchConfiguration('world_path'),
        "' if '",
        LaunchConfiguration('world_path'),
        "' != '' else '",
        PathJoinSubstitution([
            bringup_pkg_path,
            'worlds',
            [LaunchConfiguration('world_name'), '.sdf']
        ]),
        "'"
    ])

    gazebo_world_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            ])
        ),
        launch_arguments={
            'render_engine': 'ogre2',
            'gz_args': ['-r -v 1 ', gazebo_world_path],
            'on_exit_shutdown': 'true'
        }.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock' + '@rosgraph_msgs/msg/Clock' + '[gz.msgs.Clock']
    )

    args = [
        DeclareLaunchArgument(
            'world_name',
            default_value=DEFAULT_WORLD,
            choices=AVAILABLE_WORLDS,
            description='Gazebo world name available in worlds directory '
            '(not used if world_path specified).'
        ),
        DeclareLaunchArgument(
            'world_path',
            default_value='',
            description='Gazebo world path.'
        ),
    ]

    return LaunchDescription([
        *args,
        gz_sim_resource_path,
        gazebo_world_node,
        clock_bridge,
    ])
