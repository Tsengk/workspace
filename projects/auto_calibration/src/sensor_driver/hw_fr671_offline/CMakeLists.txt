cmake_minimum_required(VERSION 3.0.2)

project(hw_fr671_offline)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gassd_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  devkit_shm_param
)


# include(${CMAKE_SOURCE_DIR}/common/cmake/env_board.cmake)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   LidarPackets.msg
#   UdpPackets.msg
#   LidarInfos.msg
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

# link_directories(
#   ${CMAKE_SOURCE_DIR}/../devel/lib/
#   ${CMAKE_SOURCE_DIR}/common/common_lib
#   common/common_lib/glog/lib
# )

catkin_package(
# devkit_shm_param
# gassd_msgs
#  INCLUDE_DIRS include
#  LIBRARIES hw_fr671_offline
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  protocol/
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_executable(hw_fr671_offline
  runnable/entry/main.cpp
  runnable/entry/huawei_decode.cpp
  runnable/entry/radar_driver.cpp
)

add_dependencies(hw_fr671_offline ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(hw_fr671_offline
  ${catkin_LIBRARIES}
)

install(TARGETS hw_fr671_offline
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


# add_executable(test_shm_cloud
#   test/test_shm_cloud.cpp
# )

# add_dependencies(test_shm_cloud ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# target_link_libraries(test_shm_cloud
#   ${catkin_LIBRARIES}
#    ${common_base_LIBRARIES}
# )
