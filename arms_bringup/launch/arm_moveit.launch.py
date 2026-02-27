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

# Arm and gripper models.
AVAILABLE_ARMS = ['gen3_lite', 'unitree_d1', 'unitree_z1']
DEFAULT_ARM = 'gen3_lite'
# d1_2f: Standard gripper for Unitree D1 arm
# z1_1f: Standard gripper for Unitree Z1 arm
AVAILABLE_GRIPPERS = ['', 'kinova_2f_lite', 'd1_2f', 'z1_1f']
DEFAULT_GRIPPER = ''  # No gripper


def launch_setup(context, *args, **kwargs):
    """Configure and launch MoveIt motion planning node with optional RViz visualization."""
    arm = LaunchConfiguration('arm').perform(context)
    base = f'{LaunchConfiguration('base').perform(context)}'
    gripper = LaunchConfiguration('gripper').perform(context)
    octomap = LaunchConfiguration('octomap').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
    publish_robot_description_semantic = LaunchConfiguration('publish_robot_description_semantic')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    warehouse_sqlite_path = LaunchConfiguration('warehouse_sqlite_path')

    robot_name = arm

    if base:
        base += '_'

    # Build MoveIt configuration with planning pipelines and robot-specific configs
    pkg_path = get_package_share_directory(f'{arm}_moveit_config')

    joint_limits_path = ReplaceString(
        source_file=f'{pkg_path}/config/{base}{arm}_{gripper}/joint_limits_template.yaml',
        replacements={'<robot_prefix>': prefix},
    )

    robot_description_semantic_path = ReplaceString(
        source_file=f'{pkg_path}/config/{base}{arm}_{gripper}/{arm}_template.srdf',
        replacements={'<robot_prefix>': prefix, '<robot_name>': robot_name},
    )

    trajectory_execution_path = ReplaceString(
        source_file=f'{pkg_path}/config/{base}{arm}_{gripper}/moveit_controllers_template.yaml',
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
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    warehouse_ros_config = {
        'warehouse_plugin': 'warehouse_ros_sqlite::DatabaseConnection',
        'warehouse_host': warehouse_sqlite_path,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            warehouse_ros_config,
            move_group_capabilities,
            {
                'use_sim_time': use_sim_time,
                'publish_robot_description_semantic': publish_robot_description_semantic,
                'octomap_resolution': 0.005
            },
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [get_package_share_directory(f'{arm}_moveit_config'), 'config', 'moveit.rviz']
    )
    rviz_node = Node(
        package='rviz2',
        condition=IfCondition(rviz),
        executable='rviz2',
        name='rviz2_moveit',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            warehouse_ros_config,
            {
                'use_sim_time': use_sim_time,
            },
        ]
    )

    return [move_group_node, rviz_node]


def generate_launch_description():

    args = [
        DeclareLaunchArgument(
            'arm',
            default_value=DEFAULT_ARM,
            choices=AVAILABLE_ARMS,
            description='Arm model'
        ),
        DeclareLaunchArgument(
            'base',
            default_value='',
            choices=[
                '',
                'jackal',
            ],
            description='Mobile base platform to mount the arm on.'
        ),
        DeclareLaunchArgument(
            'gripper',
            default_value=DEFAULT_GRIPPER,
            choices=AVAILABLE_GRIPPERS,
            description='Gripper model. No gripper if unspecified.'
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
            'publish_robot_description_semantic',
            default_value='true',
            description='Whether to publish robot description semantic.',
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
        DeclareLaunchArgument(
            'warehouse_sqlite_path',
            default_value=os.path.expanduser('~/.ros/warehouse_ros.sqlite'),
            description='Path where the warehouse database should be stored.',
        ),
    ]

    return LaunchDescription([
        *args,
        OpaqueFunction(function=launch_setup),
    ])
