import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def launch_setup(context, *args, **kwargs):
    """
        Configure ROS 2 controllers for a robotic arm.

        Controllers are launched in sequence:
        1. controller_manager: if use_sim_time is false
        2. joint_state_broadcaster
        3. arm_controller (after joint_state_broadcaster)
        4. gripper_controller (after arm_controller, if 'gripper' is enabled)
    """
    bringup_pkg_path = get_package_share_directory('arms_bringup')

    arm = LaunchConfiguration('arm', default='gen3_lite').perform(context)
    gripper = LaunchConfiguration('gripper', default='').perform(context)
    ros2_control_params = LaunchConfiguration(
        'ros2_control_params',
        default=PathJoinSubstitution([bringup_pkg_path, 'config', 'ros2_control.yaml'])
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    ros2_control_params = ReplaceString(
        source_file=ros2_control_params,
        replacements={'<robot_prefix>': (LaunchConfiguration('prefix', default=''))},
    )

    nodes = []
    # Check for arm-specific setup launch file
    arm_pkg = get_package_share_directory(f'{arm}_description')
    arm_setup_launch = PathJoinSubstitution([
        arm_pkg,
        'launch',
        f'{arm}_setup.launch.py'
    ])

    arm_setup_node = None

    if os.path.exists(arm_setup_launch.perform(context)):
        arm_setup_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arm_setup_launch),
            condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
        )

    if arm_setup_node is not None:
        nodes = [arm_setup_node]

    # Launch controller manager for real robot hardware (when not using sim time)
    controller_manager_node = Node(
        condition=UnlessCondition(use_sim_time),
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[ros2_control_params],
        output="both",
    )

    # Launch joint state broadcaster (first in sequence)
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster',
        output='screen',
        name='joint_state_broadcaster',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Launch arm trajectory controller (second in sequence)
    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        name=f'{arm}_trajectory_controller',
        output='screen',
        arguments=[f'{arm}_arm_controller', '--param-file', ros2_control_params],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    joint_to_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller],
        )
    )

    nodes += [
        controller_manager_node,
        joint_state_broadcaster,
        joint_to_arm,
    ]

    # Launch gripper controller if gripper is specified (third in sequence)
    if gripper:
        gripper_controller = Node(
            package='controller_manager',
            executable='spawner',
            name=f'{gripper}_gripper_controller',
            output='screen',
            arguments=[
                f'{gripper}_gripper_controller',
                '--param-file',
                ros2_control_params,
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        )

        arm_to_gripper = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=arm_controller,
                on_exit=[gripper_controller],
            )
        )

        nodes += [arm_to_gripper]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
