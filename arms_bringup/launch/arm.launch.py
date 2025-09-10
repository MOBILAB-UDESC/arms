from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """
    Set up and return a list of ROS 2 launch actions for bringing up a robotic arm.

    This function performs the following tasks:
    - Dynamically determine package paths based on the selected arm.
    - Generate the robot description using xacro with the specified namespace, prefix, and gripper.
    - Launch the robot_state_publisher node to publish the robot's state.
    - Optionally launch the joint_state_publisher_gui node if simulation time is disabled.
    - Optionally launch RViz2 if visualization is enabled.
    - Include the Gazebo simulation launch file if simulation time is enabled.
    - Include the ROS 2 control launch file for robot controllers if simulation time is enabled.

    Returns:
        list: Launch actions (nodes and included launch descriptions) ready for execution.
    """
    arm = LaunchConfiguration('arm').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    prefix = LaunchConfiguration('prefix').perform(context)
    gripper = LaunchConfiguration('gripper')

    # Modify prefix to namespace/prefix if namespace is used
    if namespace.perform(context):
        print("No namespace")
        prefix = namespace.perform(context) + "/" + prefix

    armPackage = f'{arm}_description'
    armRviz2File = f'{arm}.rviz'
    armpackagePath = get_package_share_directory(armPackage)
    current_package_path = get_package_share_directory("arms_bringup")
    robot_name = f'{prefix}{arm}'

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([armpackagePath, 'urdf', arm]),
        '.urdf.xacro',
        ' namespace:=', namespace,
        ' prefix:=', prefix,
        ' gripper:=', gripper,
        ' use_camera:=', LaunchConfiguration('use_camera')
    ])

    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher',
        namespace=namespace,
        parameters=[{
            #  'frame_prefix': [namespace, '/'],
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
    )

    # Test with joint states gui
    joint_state_publisher_gui = Node(
        condition=UnlessCondition(use_sim_time),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        name='joint_state_publisher_gui',
        namespace=namespace,
    )

    rviz2_path = PathJoinSubstitution([armpackagePath, 'rviz', armRviz2File])

    rviz2_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package="rviz2",
        executable="rviz2",
        name="arm_rviz2",
        output="screen",
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz2_path],
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [current_package_path, 'launch', 'arm_gazebo_launch.py']
        )),
        condition=IfCondition(LaunchConfiguration('use_sim_time')),
        launch_arguments={
            'namespace': namespace,
            'name': robot_name,
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'Y': LaunchConfiguration('Y'),
            'prefix': prefix,
            'use_camera': LaunchConfiguration('use_camera')
        }.items()
    )

    # ROS2 Control Node
    robot_controllers = PathJoinSubstitution([current_package_path, 'config', 'ros2_control.yaml'])

    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [current_package_path, 'launch', 'arm_ros2_control_launch.py']
        )),
        condition=IfCondition(LaunchConfiguration('use_sim_time')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'ros2_control_params': robot_controllers,
            'namespace': namespace,
            'prefix': prefix,
            'gripper': gripper
        }.items()
    )

    return [
        robot_state_node,
        rviz2_node,
        joint_state_publisher_gui,
        gazebo_launch,
        ros2_control_launch,
    ]


def generate_launch_description():
    # Declare launch arguments
    args = [
        DeclareLaunchArgument(
            'arm',
            default_value='gen3_lite',
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
                'kinova_2f_lite',
                'robotiq_2f_85',
            ],
            description='Gripper model'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to use simulation time'
        ),
        DeclareLaunchArgument(
            'use_camera',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to use a camera'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to execute rviz2'
        ),
        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='Robot initial pose x'
        ),
        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Robot initial pose y'
        ),
        DeclareLaunchArgument(
            'z',
            default_value='0.0',
            description='Robot initial pose z'
        ),
        DeclareLaunchArgument(
            'Y',
            default_value='0.0',
            description='Robot initial yaw (rotation around Z axis)'
        ),
    ]

    # LAUNCH
    return LaunchDescription([
        *args,
        OpaqueFunction(function=launch_setup),
    ])
