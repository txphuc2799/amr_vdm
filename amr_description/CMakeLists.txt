cmake_minimum_required(VERSION 3.0.2)
project(amr_description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES amr_description
#  CATKIN_DEPENDS std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch meshes rviz urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)