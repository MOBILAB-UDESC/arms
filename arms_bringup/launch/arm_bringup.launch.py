from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
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
from launch_ros.actions import (
    Node,
    PushRosNamespace
)
import yaml


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

    arm_profile_path = LaunchConfiguration('arm_profile_path').perform(context)
    arm_profile = LaunchConfiguration('arm_profile').perform(context)
    prefix = LaunchConfiguration('prefix')
    test_urdf = LaunchConfiguration('test_urdf')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    with open(arm_profile_path, 'r') as f:
        arm_profiles_yaml = yaml.load(f, Loader=yaml.FullLoader)
        selected_arm_profile = arm_profiles_yaml['profiles'][arm_profile]

    nodes = []

    arm = selected_arm_profile['arm']
    arm_pkg_name = f'{arm}_description'
    arm_pkg_path = get_package_share_directory(arm_pkg_name)
    bringup_pkg_path = get_package_share_directory("arms_bringup")

    robot_name = namespace.perform(context)
    if not robot_name:
        robot_name = arm
        print(f'robot_name: {robot_name}')

    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([arm_pkg_path, 'urdf', arm]),
        '.urdf.xacro',
        ' sim_gazebo:=', use_sim_time,
        ' prefix:=', prefix,
        ' gripper:=', selected_arm_profile['xacro_args']['gripper'],
        ' gripper_xyz:="', str(selected_arm_profile['xacro_args']['gripper_xyz']), '"',
        ' gripper_rpy:="', str(selected_arm_profile['xacro_args']['gripper_rpy']), '"',
        ' name:=', robot_name,
        ' namespace:=', namespace,
        ' use_camera:=', str(selected_arm_profile['xacro_args']['use_camera']),
    ])

    robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace=namespace,
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    joint_state_publisher_gui = Node(
        condition=IfCondition(test_urdf),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        namespace=namespace,
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    nodes += [robot_state_node, joint_state_publisher_gui]

    # === Gazebo Simulation ===
    gazebo_spawn_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [bringup_pkg_path, 'launch', 'arm_gazebo.launch.py']
        )),
        condition=IfCondition(AndSubstitution(use_sim_time, NotSubstitution(test_urdf))),
        launch_arguments={
            'robot_name': robot_name,
            'prefix': prefix,
            'use_camera': str(selected_arm_profile['xacro_args']['use_camera']),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'Y': LaunchConfiguration('Y'),
        }.items()
    )

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
            'namespace': namespace,
            'gripper': selected_arm_profile['xacro_args']['gripper']
        }.items()
    )

    launch_nodes_with_namespace = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            gazebo_spawn_node,
            ros2_control_launch
        ]
    )

    nodes += [launch_nodes_with_namespace]

    rviz2_path = PathJoinSubstitution([arm_pkg_path, 'rviz', f'{arm}.rviz'])
    rviz2_node = Node(
        condition=IfCondition(LaunchConfiguration('rviz')),
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz2_path],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    nodes += [rviz2_node]

    return nodes


def generate_launch_description():
    default_profile_path = PathJoinSubstitution([
        get_package_share_directory('arms_bringup'),
        'config',
        'profiles.yaml',
    ])

    # Declare launch arguments
    args = [
        DeclareLaunchArgument(
            'arm_profile',
            default_value='gen3_lite_kinova_2f_lite',
            description='Robot configuration name.'
        ),
        DeclareLaunchArgument(
            'arm_profile_path',
            default_value=default_profile_path,
            description='YAML path of the robot configuration.'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace.'
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix to prepend to all arm link and joint names'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to execute rviz2.'
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
