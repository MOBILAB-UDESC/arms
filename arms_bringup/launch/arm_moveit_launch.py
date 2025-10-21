import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from arms_utils.replace_string import replace_string


def launch_setup(context, *args, **kwargs):
    """Configure and launch MoveIt motion planning node with optional RViz visualization."""
    arm = LaunchConfiguration('arm').perform(context)
    gripper = LaunchConfiguration('gripper').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
    publish_robot_description_semantic = LaunchConfiguration('publish_robot_description_semantic')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    warehouse_sqlite_path = LaunchConfiguration('warehouse_sqlite_path')

    robot_name = arm

    # Build MoveIt configuration with planning pipelines and robot-specific configs
    pkg_path = get_package_share_directory(f'{arm}_moveit_config')
    moveit_config = (
        MoveItConfigsBuilder(robot_name=arm)
        .joint_limits(replace_string(
            f'{pkg_path}/config/{arm}_{gripper}/joint_limits_template.yaml',
            ['<robot_prefix>'],
            [prefix]
        ))
        .planning_scene_monitor(
            publish_robot_description=False,
            publish_robot_description_semantic=True,
            publish_planning_scene=True,
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner", "stomp"],
            default_planning_pipeline="ompl"
        )
        .pilz_cartesian_limits(f'{pkg_path}/config/pilz_cartesian_limits.yaml')
        .robot_description_kinematics(f'{pkg_path}/config/kinematics.yaml')
        .robot_description_semantic(replace_string(
            f'{pkg_path}/config/{arm}_{gripper}/{arm}_template.srdf',
            ['<robot_prefix>', '<robot_name>'],
            [prefix, robot_name]
        ))
        .trajectory_execution(replace_string(
            f'{pkg_path}/config/{arm}_{gripper}/moveit_controllers_template.yaml',
            ['<robot_prefix>'],
            [prefix]
        ))
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
            default_value='unitree_z1',
            choices=[
                'gen3_lite',
                'unitree_d1',
                'unitree_z1',
            ],
            description='Arm model.'
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
            description='Gripper model.'
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
            default_value='false',
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
