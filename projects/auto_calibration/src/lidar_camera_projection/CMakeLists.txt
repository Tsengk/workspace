cmake_minimum_required(VERSION 3.5)
project(lidar_camera_projection)

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
  camera_model
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs std_srvs sensor_msgs cv_bridge image_transport camera_model
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(lidar_camera_projection_node src/lidar_camera_projection_node.cpp)
target_link_libraries(lidar_camera_projection_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS lidar_camera_projection_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
