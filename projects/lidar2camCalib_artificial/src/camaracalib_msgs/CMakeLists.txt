cmake_minimum_required(VERSION 2.8.3)
project(cameracalib_msgs)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  # roscpp
  # rospy
  std_msgs
  sensor_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ivlidarcameracalib.msg
  capture2calib.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

#include(/opt/ros/kinetic/share/catkin/cmake/env/CMakeLists.txt)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS std_msgs sensor_msgs
   message_runtime
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)


                                                                                                                                                                                                                                                                                                                                       
