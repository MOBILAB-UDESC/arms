import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from nav2_common.launch import ReplaceString
import yaml


def launch_setup(context, *args, **kwargs):
    """Configure and launch MoveIt motion planning node with optional RViz visualization."""
    arm_profile_path = LaunchConfiguration('arm_profile_path').perform(context)
    arm_profile = LaunchConfiguration('arm_profile').perform(context)
    octomap = LaunchConfiguration('octomap').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    namespace_str = namespace.perform(context)

    with open(arm_profile_path, 'r') as f:
        arm_profiles_yaml = yaml.load(f, Loader=yaml.FullLoader)
        selected_arm_profile = arm_profiles_yaml['profiles'][arm_profile]

    arm = selected_arm_profile['arm']
    gripper = selected_arm_profile['xacro_args']['gripper']
    robot_name = namespace_str

    if not robot_name:
        robot_name = arm
        print(f'robot_name: {robot_name}')

    # Build MoveIt configuration with planning pipelines and robot-specific configs
    pkg_path = get_package_share_directory(f'{arm}_moveit_config')

    joint_limits_path = ReplaceString(
        source_file=f'{pkg_path}/config/{arm}_{gripper}/joint_limits_template.yaml',
        replacements={'<robot_prefix>': prefix},
    )

    robot_description_semantic_path = ReplaceString(
        source_file=f'{pkg_path}/config/{arm}_{gripper}/{arm}_template.srdf',
        replacements={'<robot_prefix>': prefix, '<robot_name>': robot_name},
    )

    trajectory_execution_path = ReplaceString(
        source_file=f'{pkg_path}/config/{arm}_{gripper}/moveit_controllers_template.yaml',
        replacements={'<robot_prefix>': prefix},
    )

    if octomap == 'true':
        sensors_3d_path = ReplaceString(
            source_file=f'{pkg_path}/config/sensors_3d_template.yaml',
            replacements={'<robot_prefix>': prefix},
        )
    else:
        sensors_3d_path = PathJoinSubstitution([pkg_path, 'config', 'sensors_3d_empty.yaml'])

    moveit_config = (
        MoveItConfigsBuilder(robot_name=arm)
        .joint_limits(joint_limits_path.perform(context))
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp", "chomp"],
            default_planning_pipeline="ompl"
        )
        .robot_description_semantic(robot_description_semantic_path.perform(context))
        .trajectory_execution(trajectory_execution_path.perform(context))
        .sensors_3d(sensors_3d_path.perform(context))
        .to_moveit_configs()
    )

    # For executing Task Constructor solutions
    # move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        namespace=namespace,
        parameters=[
            moveit_config.to_dict(),
            # move_group_capabilities,
            {
                'use_sim_time': use_sim_time,
                'publish_robot_description_semantic': True,
                'octomap_resolution': 0.005
            },
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [get_package_share_directory(f'{arm}_moveit_config'), 'config', 'moveit.rviz']
    )

    if namespace_str:
        rviz_config_file = ReplaceString(
            source_file=rviz_config_file,
            replacements={'Move Group Namespace: ""': f'Move Group Namespace: "/{namespace_str}"'},
        )

    rviz_node = Node(
        condition=IfCondition(rviz),
        package='rviz2',
        executable='rviz2',
        output='log',
        namespace=namespace,
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                'use_sim_time': use_sim_time,
            },
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    return [move_group_node, rviz_node]


def generate_launch_description():
    default_profile_path = PathJoinSubstitution([
        get_package_share_directory('arms_bringup'),
        'config',
        'profiles.yaml',
    ])

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
            'octomap',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to use octomap.',
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix for the link/joint names of the robot.'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Whether to execute rviz2.'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Whether to use simulation time.',
        ),
    ]

    return LaunchDescription([
        *args,
        OpaqueFunction(function=launch_setup),
    ])
