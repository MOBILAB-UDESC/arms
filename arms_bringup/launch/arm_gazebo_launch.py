from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def bridge_camera(context, *args, **kwargs):
    """Create and return a ROS 2 node to bridge the robot's camera topics from Gazebo."""
    # Dynamically set topic names based on the 'prefix' launch argument.
    prefix = LaunchConfiguration('prefix', default='').perform(context)
    camera_topic = f'/{prefix}camera_head' if prefix else '/camera_head'

    camera_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        name=f'{prefix}_camera_bridge',
        arguments=[
            # ros2_topic@ros2_type[gz_type          gz->ros2
            f'{camera_topic}/image' + '@sensor_msgs/msg/Image' + '[gz.msgs.Image',
            f'{camera_topic}/depth_image' + '@sensor_msgs/msg/Image' + '[gz.msgs.Image',
            f'{camera_topic}/points' + '@sensor_msgs/msg/PointCloud2' + '[gz.msgs.PointCloudPacked',
            f'{camera_topic}/info' + '@sensor_msgs/msg/CameraInfo' + '[gz.msgs.CameraInfo',
        ],
    )

    return [camera_bridge_node]


def generate_launch_description():
    """Spawn the robot and optionally bridging its camera."""
    namespace = LaunchConfiguration('namespace', default='')

    # Spawn the robot in Gazebo using the robot_description topic
    gazebo_spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        namespace=namespace,
        arguments=[
            '-topic', 'robot_description',
            '-name', LaunchConfiguration('name', default='gen3_lite'),
            '-allow_renaming', 'true',
            '-x', LaunchConfiguration('x', default=0.0),
            '-y', LaunchConfiguration('y', default=0.0),
            '-z', LaunchConfiguration('z', default=0.0),
            '-Y', LaunchConfiguration('Y', default=0.0),
        ],
    )

    return LaunchDescription([
        gazebo_spawn_node,
        OpaqueFunction(
            function=bridge_camera,
            condition=IfCondition(LaunchConfiguration('use_camera', default=False))
        ),
    ])
