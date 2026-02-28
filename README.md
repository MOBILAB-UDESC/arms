# Arms
## Description
Packages for manipulators available in MOBILAB.

## ROS 2 info
|Ubuntu|ROS 2 Distro|Gazebo Version|
|:----:|:---------------:|:------------:|
|[24.04](https://ubuntu.com/blog/tag/ubuntu-24-04-lts)|[Jazzy](https://docs.ros.org/en/jazzy/index.html)|[Harmonic](https://gazebosim.org/docs/harmonic/getstarted/)|

## Repository Structure
```tree
arms/
├── arms/
├── arms_bringup/
│   ├── arm/
│   └── grippers/
├── arms_utils/
├── gen3_lite/
│   ├── description/
│   └── moveit_config/
├── grippers/
│   ├── d1_2f/
│   ├── kinova_2f_lite/
│   └── z1_1f/
├── unitree_d1/
│   ├── description/
│   ├── hardware_interface/
│   └── moveit_config/
├── unitree_z1/
│   ├── description/
│   ├── hardware_interface/
│   └── moveit_config/
└── README.md
```

**Note**: D1 hardware interface package requires [`cyclonedds`](https://github.com/eclipse-cyclonedds/cyclonedds) and [`cyclonedds-cxx`](https://github.com/eclipse-cyclonedds/cyclonedds-cxx) to be installed on your PC. See [`simple_unitree_d1_sdk`](https://github.com/MOBILAB-UDESC/simple_unitree_D1_sdk.git) for details.

## Cloning and building
``` bash
mkdir ~/arms_ws && cd ~/arms_ws
git clone --recursive https://github.com/MOBILAB-UDESC/arms.git src
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:~/arms_ws/build/arms_moveit_demos/_deps/attachable_objects_plugin-build/' >> ~/.bashrc
source  ~/.bashrc
```

### Gazebo models (Optional, for some gazebo worlds)
``` bash
mkdir ~/mobi_gz_models && cd ~/mobi_gz_models
git clone --recursive https://github.com/NiltonSV/gazebo_models.git .
echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/mobi_gz_models' >> ~/.bashrc
source  ~/.bashrc
```

## USAGE

### 1. Visualize URDF in RViz
``` bash
ros2 launch arms_bringup arm.launch.py arm:=unitree_d1 gripper:=d1_2f test_urdf:=true rviz:=true
```

<img src="https://raw.githubusercontent.com/MOBILAB-UDESC/arms/main/doc/resources/First_example.png" alt="d1_urdf" width="1500"/>

### 2. GAZEBO SIMULATION

#### Launch gazebo world
``` bash
ros2 launch arms_bringup arm_world.launch.py
```

#### Spawn manipulator
``` bash
ros2 launch arms_bringup arm.launch.py arm:=unitree_d1 gripper:=d1_2f use_sim_time:=true
```

<img src="https://raw.githubusercontent.com/MOBILAB-UDESC/arms/main/doc/resources/Second_example_1.png" alt="d1_gazebo" width="1500"/>

#### Launch moveit (planning in simulation):
``` bash
ros2 launch arms_bringup arm_moveit.launch.py arm:=unitree_d1 gripper:=d1_2f use_sim_time:=true
```

<img src="https://raw.githubusercontent.com/MOBILAB-UDESC/arms/main/doc/resources/Second_example_2.png" alt="d1_gazebo_moveit" width="1500"/>

### 3. REAL ROBOT
**Note**: The Unitree D1 is supported and runs without a gripper at the moment.
#### Start bringup and controllers:
``` bash
ros2 launch arms_bringup arm.launch.py arm:=unitree_d1
```

<img src="https://raw.githubusercontent.com/MOBILAB-UDESC/arms/main/doc/resources/Third_example_1.jpeg" alt="d1" width="1500"/>

#### Launch moveit
``` bash
ros2 launch arms_bringup arm_moveit.launch.py arm:=unitree_d1
```

<img src="https://raw.githubusercontent.com/MOBILAB-UDESC/arms/main/doc/resources/Thrid_example_2.png" alt="d1_moveit" width="1500"/>