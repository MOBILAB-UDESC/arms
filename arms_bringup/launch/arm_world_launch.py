from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Gazebo world and bridge clock topic."""
    packagePath = get_package_share_directory("arms_bringup")

    gazebo_path = PathJoinSubstitution([packagePath, 'worlds', LaunchConfiguration('world_name')])

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
            'gz_args': ['-r -v 1 ', gazebo_path],
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

    # Declare launch arguments
    args = [
        DeclareLaunchArgument(
            'world_name',
            default_value='empty.sdf',
            choices=[
                'empty.sdf',
                'mobilab.sdf',
                'playground.sdf'
            ],
            description='Gazebo world file (custom worlds should be added to the "worlds" folder of this package'
        ),
    ]

    return LaunchDescription([
        *args,
        gazebo_world_node,
        clock_bridge,
    ])
