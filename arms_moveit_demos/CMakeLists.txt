cmake_minimum_required(VERSION 3.8)
project(arms_moveit_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(plan_demo_executable plan_demo_node)
set(pick_and_place_demo_executable pick_and_place_demo_node)

#####################################################
#           Downloading Attachable Plugin           #
#####################################################
include(FetchContent)
FetchContent_Declare(
  attachable_objects_plugin
  GIT_REPOSITORY https://github.com/akinami3/AttachablePlugin.git
  GIT_TAG main
)
FetchContent_GetProperties(attachable_objects_plugin)
if(NOT attachable_objects_plugin_POPULATED)
  FetchContent_Populate(attachable_objects_plugin)
  add_subdirectory(${attachable_objects_plugin_SOURCE_DIR}/attachable_joint_plugin ${attachable_objects_plugin_BINARY_DIR})
endif()

#################################
#           Plan Demo           #
#################################
add_executable(${plan_demo_executable}
  src/plan_demo.cpp)

target_include_directories(${plan_demo_executable}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(${plan_demo_executable} PRIVATE
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  ${moveit_ros_planning_interface_TARGETS}
  ${moveit_visual_tools_TARGETS}
  ${tf2_geometry_msgs_TARGETS})

install(TARGETS
    ${plan_demo_executable}
  DESTINATION lib/${PROJECT_NAME})

###########################################
#           Pick and Place Demo           #
###########################################
add_executable(${pick_and_place_demo_executable}
  src/pick_and_place_demo.cpp)

target_include_directories(${pick_and_place_demo_executable}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

target_link_libraries(${pick_and_place_demo_executable} PRIVATE
  rclcpp::rclcpp
  gz-transport13::gz-transport13
  ${geometry_msgs_TARGETS}
  ${moveit_msgs_TARGETS}
  ${moveit_ros_planning_interface_TARGETS}
  ${moveit_visual_tools_TARGETS}
  ${tf2_geometry_msgs_TARGETS})

install(TARGETS
    ${pick_and_place_demo_executable}
  DESTINATION lib/${PROJECT_NAME})

###############################
#           Install           #
###############################

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
