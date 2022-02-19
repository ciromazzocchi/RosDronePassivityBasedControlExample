cmake_minimum_required(VERSION 2.8.3)
project(quad_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
    pkg_check_modules(GAZEBO gazebo)
endif()

find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  gazebo_msgs
  geometry_msgs
  message_generation
  message_runtime
  nav_msgs
  roscpp
  roslib
  std_msgs
  tf
  tf_conversions
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  UavState.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#     Service1.srv
#     Service2.srv
# )

## Generate actions in the 'action' folder
#add_action_files(
#   FILES
#	 nav.action
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   nav_msgs
#   actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES surveillance_task
#  CATKIN_DEPENDS gazebo_msgs geometry_msgs roscpp roslib sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})


## Declare a C++ library
#add_library(${PROJECT_NAME}
#      FILES
#)

add_library(Differentiator  src/Utils/Differentiator.cpp)
add_library(Utility  src/Utils/utility.cpp)

## Declare a C++ executable
add_executable(odometry_node src/Nodes/odometry_node.cpp)
target_link_libraries(odometry_node ${Eigen_INCLUDE_DIRS} ${catkin_LIBRARIES})

add_executable(trajectory_node src/Nodes/trajectory_node.cpp)
target_link_libraries(trajectory_node ${Eigen_INCLUDE_DIRS} ${catkin_LIBRARIES})

add_executable(outer_loop_node src/Nodes/outer_loop_node.cpp)
target_link_libraries(outer_loop_node ${Eigen_INCLUDE_DIRS} ${catkin_LIBRARIES})

add_executable(eta_reference_node src/Nodes/eta_reference_node.cpp)
target_link_libraries(eta_reference_node Differentiator ${Eigen_INCLUDE_DIRS} ${catkin_LIBRARIES})

#add_executable(inner_loop_node src/Nodes/inner_loop_node.cpp)
#target_link_libraries(inner_loop_node Utility Differentiator ${Eigen_INCLUDE_DIRS} ${catkin_LIBRARIES})