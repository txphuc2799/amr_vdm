cmake_minimum_required(VERSION 3.0.2)
project(amr_gazebo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  amr_description
  controller_manager
  gazebo_ros
  joint_state_publisher
  robot_localization
  robot_state_publisher
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES amr_gazebo
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

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY config launch rviz sdf world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
