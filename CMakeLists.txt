cmake_minimum_required(VERSION 3.0.2)
project(fs_path_planning)
 
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  message_runtime
  geometry_msgs
  std_msgs
)
 
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Sphere.msg
  SphereCloud.msg
  PointList.msg
)

add_service_files(
  FILES
  Astar.srv
)
 
## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
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
#  LIBRARIES quadrotor_receive
CATKIN_DEPENDS message_generation message_runtime rospy roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  src/classic_path_planner/astar
  ${catkin_INCLUDE_DIRS}
)

add_executable(astar src/classic_path_planner/astar/astar.cpp)
target_link_libraries(astar ${catkin_LIBRARIES})
