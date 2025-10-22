from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    z1_controller_script_path = PathJoinSubstitution([
        get_package_share_directory('z1_hardware_interface'),
        'scripts',
        'z1_controller_process.py'
    ])

    z1_controller_process = ExecuteProcess(
        cmd=["python3", z1_controller_script_path],
        output="screen",
    )

    return LaunchDescription([
        z1_controller_process
    ])