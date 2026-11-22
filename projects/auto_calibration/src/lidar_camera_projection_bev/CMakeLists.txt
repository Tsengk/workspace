cmake_minimum_required(VERSION 3.5)
project(lidar_camera_projection_bev)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  cv_bridge
  image_transport
  message_filters
  pcl_ros
  pcl_conversions
)
find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs std_srvs sensor_msgs cv_bridge image_transport message_filters pcl_ros pcl_conversions
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_executable(lidar_camera_projection_bev_node src/lidar_camera_projection_node.cpp)
target_link_libraries(lidar_camera_projection_bev_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

install(TARGETS lidar_camera_projection_bev_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
