# ARMS BRINGUP
Common launch files.

## Overview
There are 3 main launch files:

### 1. arms_world_launch.py
Start gazebo with a selected world.
- Args:
  - **world_name**: world file name from the [worlds](https://github.com/MOBILAB-UDESC/arms/tree/main/arms_bringup/worlds) directory. Default value is empty.sdf and other choices are mobilab.sdf (requires [mobilab_models](https://github.com/NiltonSV/gazebo_models.git)) and playground.sdf.

### 2. arms.launch.py
Initialize the necessary nodes to start up the robot description, ROS 2 controllers, robot spawning in Gazebo, and RViz2 visualization. It automatically selects the correct packages depending on the provided arguments.

|Arg|Description|Default|Choices|
|:---:|:---:|:---:|:---:|
|**arm**|Specifies the arm to be used. The corresponding packages must start with the arm name (e.g., ARMNAME_description, ARMNAME_moveit_config).|unitree_z1|unitree_z1, unitree_d1, gen3_lite|
|**gripper**|Specifies the gripper to be used. The corresponding packages must start with the gripper name (e.g., GRIPPERNAME_description). Grippers are separated from arms to allow easy interchange between compatible ones (Mobilab grippers).|''|'', z1_1f, d1_2f, kinova_2f_lite|
|**prefix**|Adds a prefix to every link and joint of the robot|''|any string|
|**test_urdf**|Launches a simple visualization of the robot in RViz if true.|false|true, false|
|**use_sim_time**|Uses simulation time and spawns the robot in Gazebo if true.|false|true, false|
|**use_camera**|Enables or disables the robot’s camera.|false|true, false|
|**rviz**|Launches RViz2 for visualization if true.|false|true, false|
|**x**, **y**, **z**, **Y**|Defines the robot’s initial position (x, y, z) and yaw orientation (Y) in Gazebo.|0.0|any float|

### 3. arm_moveit_launch.py
Start MoveIt 2 for motion planning and control of the selected manipulator.
It shares most arguments with arms.launch.py.

|Arg|Default|
|:---:|:---:|
|**arm**|unitree_z1|
|**gripper**|''|
|**prefix**|''|
|**use_sim_time**|false|
|**use_camera**|false|
|**rviz**|true|

## USAGE

### Launch MoveIt 2 for a Gen3 Lite arm + Kinova 2F Lite gripper using simulation time:
```bash
ros2 launch arms_bringup arm_world_launch.py world_name:=playground.sdf
```
```bash
ros2 launch arms_bringup arms.launch.py arm:=gen3_lite gripper:=kinova_2f_lite prefix:=kinova_ use_sim_time:=true
```
```bash
ros2 launch arms_bringup arm_moveit_launch.py arm:=gen3_lite gripper:=kinova_2f_lite prefix:=kinova_ use_sim_time:=true
```