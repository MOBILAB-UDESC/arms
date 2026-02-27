from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.conditions import (
    IfCondition,
    UnlessCondition
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    Command,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node

# Arm and gripper models.
AVAILABLE_ARMS = ['gen3_lite', 'unitree_d1', 'unitree_z1']
DEFAULT_ARM = 'gen3_lite'
# d1_2f: Standard gripper for Unitree D1 arm
# z1_1f: Standard gripper for Unitree Z1 arm
AVAILABLE_GRIPPERS = ['', 'kinova_2f_lite', 'd1_2f', 'z1_1f']
DEFAULT_GRIPPER = ''  # No gripper


def launch_setup(context):
    """
        Set up and return a list of ROS 2 launch actions for bringing up arms and grippers.

        This function performs the following tasks:
        - Robot State Publisher
        - Gazebo simulation: if use_sim_time is True
        - ROS 2 control
        - Rviz2

        Returns:
            list: List of nodes ready for execution.
    """
    arm = LaunchConfiguration('arm').perform(context)
    gripper = LaunchConfiguration('gripper')
    prefix = LaunchConfiguration('prefix')
    robot_name = LaunchConfiguration('robot_name')
    test_urdf = LaunchConfiguration('test_urdf')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get packages path based on the selected arm and gripper.
    arm_pkg_name = f'{arm}_description'
    arm_pkg_path = get_package_share_directory(arm_pkg_name)
    bringup_pkg_path = get_package_share_directory("arms_bringup")

    # === Robot Description and State Publishing ===
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

    nodes = []

    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
        }],
    )

    # Joint state GUI for testing
    joint_state_publisher_gui = Node(
        condition=IfCondition(test_urdf),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    nodes += [
        robot_state_node,
        joint_state_publisher_gui
    ]

    # === Gazebo Simulation ===
    gazebo_spawn_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [bringup_pkg_path, 'launch', 'arm_gazebo.launch.py']
        )),
        condition=IfCondition(AndSubstitution(use_sim_time, NotSubstitution(test_urdf))),
        launch_arguments={
            'robot_name': robot_name,
            'prefix': prefix,
            'use_camera': LaunchConfiguration('use_camera'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'Y': LaunchConfiguration('Y'),
        }.items()
    )

    nodes += [gazebo_spawn_node]

    # === ROS 2 Control (skipped during URDF testing) ===
    robot_controllers = PathJoinSubstitution([bringup_pkg_path, 'config', 'ros2_control.yaml'])
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [bringup_pkg_path, 'launch', 'arm_ros2_control.launch.py']
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

    nodes += [ros2_control_launch]

    # === Visualization ===
    rviz2_path = PathJoinSubstitution([arm_pkg_path, 'rviz', f'{arm}.rviz'])
    rviz2_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package="rviz2",
        executable="rviz2",
        name="arm_rviz2",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz2_path],
    )

    nodes += [rviz2_node]

    return nodes


def generate_launch_description():
    # Declare launch arguments
    args = [
        DeclareLaunchArgument(
            'arm',
            default_value=DEFAULT_ARM,
            choices=AVAILABLE_ARMS,
            description='Arm model'
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix to prepend to all arm link and joint names'
        ),
        DeclareLaunchArgument(
            'gripper',
            default_value=DEFAULT_GRIPPER,
            choices=AVAILABLE_GRIPPERS,
            description='Gripper model. No gripper if unspecified.'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to execute rviz2.'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value=LaunchConfiguration('arm'),
            description='Name of the robot.'
        ),
        DeclareLaunchArgument(
            'test_urdf',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to test/check the URDF with joint_state_publisher_gui.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to use simulation time.'
        ),
        DeclareLaunchArgument(
            'use_camera',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to use a camera.'
        ),
        DeclareLaunchArgument('x', default_value='0.0', description='Robot initial pose x.'),
        DeclareLaunchArgument('y', default_value='0.0', description='Robot initial pose y.'),
        DeclareLaunchArgument('z', default_value='0.0', description='Robot initial pose z.'),
        DeclareLaunchArgument('Y', default_value='0.0', description='Robot initial yaw angle.'),
    ]

    # LAUNCH
    return LaunchDescription([
        *args,
        OpaqueFunction(function=launch_setup),
    ])
