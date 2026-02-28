# ARMS MOVEIT DEMOS
Moveit2 Demo tasks.

## Overview
This package provides two demonstration nodes using Moveit2:

### 1. plan_demo.launch.py
Move the end effectot to a specified target pose. Configure parameters in [demo.yaml](https://github.com/MOBILAB-UDESC/arms/tree/main/arms_moveit_demos/config/demos.yaml):
|Arg|Default|
|:---:|:---:|
|**arm_move_group_name**|'arm'|
|**gripper_move_group_name**|'gripper'|
|**target_position**|[-0.282, -0.010, 0.5]|
|**target_orientation**|[0.0, 0.0, 0.0]|
|**max_vel_scaling_factor**|0.5|
|**max_acc_scaling_factor**|0.5|
|**use_sim_time**|true|

```bash
ros2 launch arms_moveit_demos plan_demo.launch.py use_sim_time:=true
```

### 2. pick_and_place_demo.launch.py
Execute a pick and place demo task.
Configure parameters in [demo.yaml](https://github.com/MOBILAB-UDESC/arms/tree/main/arms_moveit_demos/config/demos.yaml):
|Arg|Default|
|:---:|:---:|
|**arm_move_group_name**|'arm'|
|**gripper_move_group_name**|'gripper'|
|**base_link**|'base_link'|
|**init_pose**|'zero'|
|**target_position**|[-0.282, -0.010, 0.5]|
|**target_orientation**|[0.0, 0.0, 0.0]|
|**place_pose**|'place'|
|**max_vel_scaling_factor**|0.5|
|**max_acc_scaling_factor**|0.5|
|**sim_attach**|true|
|**last_arm_link**|'end_effector_link'|
|**target_name**|'Apple'|
|**target_link**|'apple_link'|
|**use_sim_time**|true|

**Note**: Set **sim_attach** to **false** to deactivate attachable gazebo plugin.

```bash
ros2 launch arms_moveit_demos pick_and_place_demo.launch.py use_sim_time:=true
```