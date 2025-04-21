################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(mini_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    geometry_msgs
    std_msgs
    message_generation
    dynamixel_sdk
    dynamixel_workbench_toolbox
    hardware_interface
    controller_manager
)

catkin_python_setup()

## YAML-CPP
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    geometry_msgs
    std_msgs
    dynamixel_sdk
    controller_manager
    dynamixel_workbench_toolbox
    hardware_interface
#  DEPENDS system_lib
)

################################################################################
# Build
################################################################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(mini_hw_control src/mini_hw_control.cpp src/hardware_interface.cpp)
add_dependencies(mini_hw_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mini_hw_control ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(walking_node src/zmp_walking.cpp src/gait_planning.cpp src/foot_planning.cpp src/mod.cpp src/rt_nonfinite.cpp src/rtGetInf.cpp src/rtGetNaN.cpp)
add_dependencies(walking_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} sensor_msgs_generate_messages_cpp)
target_link_libraries(walking_node ${catkin_LIBRARIES})