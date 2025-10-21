from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    Command,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """
    Configure and launch robotic arm nodes based on provided arguments.

    Returns
    ----------
        list: Launch actions ready for execution.
    """
    arm = LaunchConfiguration('arm').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    prefix = LaunchConfiguration('prefix')
    gripper = LaunchConfiguration('gripper')
    test_urdf = LaunchConfiguration('test_urdf')

    # Get packages path based on the selected arm and gripper.
    arm_pkg = f'{arm}_description'
    arm_pkg_path = get_package_share_directory(arm_pkg)
    arm_rviz2_file = f'{arm}.rviz'
    robot_name = arm
    current_package_path = get_package_share_directory("arms_bringup")

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([arm_pkg_path, 'urdf', arm]),
        '.urdf.xacro',
        ' sim_gazebo:=', use_sim_time,
        ' prefix:=', prefix,
        ' gripper:=', gripper,
        ' name:=', robot_name,
        ' use_camera:=', LaunchConfiguration('use_camera')
    ])

    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
    )

    # Launch joint state GUI for URDF testing/checking
    joint_state_publisher_gui = Node(
        condition=IfCondition(test_urdf),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz2_path = PathJoinSubstitution([arm_pkg_path, 'rviz', arm_rviz2_file])
    rviz2_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package="rviz2",
        executable="rviz2",
        name="arm_rviz2",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz2_path],
    )

    # Launch Gazebo simulation (when not testing URDF)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [current_package_path, 'launch', 'arm_gazebo_launch.py']
        )),
        condition=IfCondition(AndSubstitution(use_sim_time, NotSubstitution(test_urdf))),
        launch_arguments={
            'name': robot_name,
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'Y': LaunchConfiguration('Y'),
            'prefix': prefix,
            'use_camera': LaunchConfiguration('use_camera')
        }.items()
    )

    # Launch ROS2 controllers (skipped during URDF testing)
    robot_controllers = PathJoinSubstitution([current_package_path, 'config', 'ros2_control.yaml'])
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [current_package_path, 'launch', 'arm_ros2_control_launch.py']
        )),
        condition=UnlessCondition(test_urdf),
        launch_arguments={
            'arm': arm,
            'use_sim_time': use_sim_time,
            'ros2_control_params': robot_controllers,
            'prefix': prefix,
            'gripper': gripper
        }.items()
    )

    return [
        robot_state_node,
        joint_state_publisher_gui,
        gazebo_launch,
        ros2_control_launch,
        rviz2_node,
    ]


def generate_launch_description():
    # Declare launch arguments
    args = [
        DeclareLaunchArgument(
            'arm',
            default_value='unitree_z1',
            choices=[
                'gen3_lite',
                'unitree_d1',
                'unitree_z1',
            ],
            description='Arm model'
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix added to the robot link and joint names'
        ),
        DeclareLaunchArgument(
            'gripper',
            default_value='',
            choices=[
                '',  # No gripper
                'd1_2f',  # Standard gripper for Unitree D1 arm
                'kinova_2f_lite',
                'z1_1f',  # Standard gripper for Unitree Z1 arm
            ],
            description='Gripper model. No gripper if unspecified.'
        ),
        DeclareLaunchArgument(
            'test_urdf',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to test/check the URDF with joint_state_publisher_gui.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to use simulation time.'
        ),
        DeclareLaunchArgument(
            'use_camera',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to use a camera.'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to execute rviz2.'
        ),
        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='Robot initial pose x.'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Robot initial pose y.'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.0',
            description='Robot initial pose z.'
        ),
        DeclareLaunchArgument(
            'Y',
            default_value='0.0',
            description='Robot initial yaw (rotation around Z axis).'
        ),
    ]

    # LAUNCH
    return LaunchDescription([
        *args,
        OpaqueFunction(function=launch_setup),
    ])
