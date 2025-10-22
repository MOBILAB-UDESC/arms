cmake_minimum_required(VERSION 3.8)
project(d1_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Get D1-SDK and UNITREE SDK2 from github

include(FetchContent)
FetchContent_Declare(
  simple_d1_sdk
  GIT_REPOSITORY https://github.com/MOBILAB-UDESC/simple_unitree_D1_sdk.git
  GIT_TAG main
)
FetchContent_GetProperties(simple_d1_sdk)
if(NOT simple_d1_sdk_POPULATED)
  FetchContent_Populate(simple_d1_sdk)
endif()

FetchContent_Declare(
  unitree_sdk2
  GIT_REPOSITORY https://github.com/unitreerobotics/unitree_sdk2.git
  GIT_TAG main
)
FetchContent_GetProperties(unitree_sdk2)
if(NOT unitree_sdk2_POPULATED)
  FetchContent_Populate(unitree_sdk2)
endif()

set(servoanglemsg ${simple_d1_sdk_SOURCE_DIR}/src/msg/ServoAngleData.cpp)
set(servopowermsg ${simple_d1_sdk_SOURCE_DIR}/src/msg/ServoPower.cpp)

set(unitree_sdk2_LIB ${unitree_sdk2_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}/libunitree_sdk2.a)
add_library(unitree_sdk2_imported STATIC IMPORTED)
set_target_properties(unitree_sdk2_imported PROPERTIES
  IMPORTED_LOCATION ${unitree_sdk2_LIB}
  INTERFACE_INCLUDE_DIRECTORIES ${unitree_sdk2_SOURCE_DIR}/include
)

# Hardware interface target

add_library(
  ${PROJECT_NAME}
  SHARED
  src/d1_hardware_interface.cpp
  ${servoanglemsg}
  ${servopowermsg}
)

target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE
  CycloneDDS-CXX::ddscxx
  unitree_sdk2_imported
)

ament_target_dependencies(
  ${PROJECT_NAME}
  PUBLIC
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)


target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${simple_d1_sdk_SOURCE_DIR}/src/msg
)

pluginlib_export_plugin_description_file(hardware_interface d1_hardware_interface.xml)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
