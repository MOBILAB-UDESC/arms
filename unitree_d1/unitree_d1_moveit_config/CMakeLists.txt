cmake_minimum_required(VERSION 3.22)
project(unitree_d1_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})