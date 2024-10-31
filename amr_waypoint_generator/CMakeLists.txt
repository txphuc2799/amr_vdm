cmake_minimum_required(VERSION 3.0.2)
project(amr_waypoint_generator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  move_base_msgs
  roslaunch
  tf
  smach
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES amr_waypoint_generator
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/waypoints_generator.py
  scripts/get_fixed_position.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  cfg
  scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)