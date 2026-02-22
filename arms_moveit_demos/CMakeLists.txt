cmake_minimum_required(VERSION 3.8)
project(arms_moveit_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

set(plan_demo_executable plan_demo_node)

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
  # ${moveit_core_TARGETS}
  ${moveit_ros_planning_interface_TARGETS})

install(TARGETS
    ${plan_demo_executable}
  DESTINATION lib/${PROJECT_NAME})

####################################
#           Gripper Demo           #
####################################

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
