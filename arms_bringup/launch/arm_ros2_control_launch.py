from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def launch_setup(context, *args, **kwargs):
    """
    Set up ROS 2 control for a robotic arm.

    This function performs the following tasks:
    - Dynamically determine package paths based on the selected arm (and gripper if enabled).
    - Replace the <robot_prefix> placeholder in the ros2_control.yaml file with the actual namespace/prefix.
    Controllers are launched in sequence:
        1. joint_state_broadcaster
        2. arm_controller (after joint_state_broadcaster exits)
        3. gripper_controller (after arm_controller exits, if 'gripper' is enabled)
    """
    package = "arms_bringup"
    packagePath = get_package_share_directory(package)

    arm = LaunchConfiguration('arm', default='gen3_lite').perform(context)
    gripper = LaunchConfiguration('gripper', default='').perform(context)
    namespace = LaunchConfiguration('namespace', default='')
    ros2_control_params = LaunchConfiguration(
        'ros2_control_params',
        default=PathJoinSubstitution([packagePath, 'config', 'ros2_control.yaml'])
    ).perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    # Replaces <robot_prefix> in ros2_control.yaml with the namespace/prefix
    # from arm.launch.py, so joints are correctly namespaced.
    ros2_control_params = ReplaceString(
        source_file=ros2_control_params,
        replacements={'<robot_prefix>': (LaunchConfiguration('prefix', default=''))},
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        name='joint_state_broadcaster',
        namespace=namespace,
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ARM
    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        name='joint_trajectory_controller',
        namespace=namespace,
        arguments=[f'{arm}_arm_controller', '--param-file', ros2_control_params],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    joint_to_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller],
        )
    )

    # GRIPPER
    if gripper:
        gripper_controller = Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            name='gripper_controller',
            namespace=namespace,
            arguments=[f'{gripper}_gripper_controller', '--param-file', ros2_control_params],
            parameters=[{'use_sim_time': use_sim_time}],
        )

        arm_to_gripper = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=arm_controller,
                on_exit=[gripper_controller],
            )
        )

        return [joint_state_broadcaster, joint_to_arm, arm_to_gripper]

    return [joint_state_broadcaster, joint_to_arm]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
