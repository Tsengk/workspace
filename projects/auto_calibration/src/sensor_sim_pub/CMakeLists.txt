cmake_minimum_required(VERSION 3.5)
project(sensor_sim_pub)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  geometry_msgs
  pcl_ros
  pcl_conversions
  image_transport
  camera_info_manager
  cv_bridge
  tf2_ros
  vehicle_config
)

find_package(PCL 1.8 REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp sensor_msgs nav_msgs std_msgs std_srvs geometry_msgs
    pcl_ros pcl_conversions image_transport camera_info_manager
    cv_bridge tf2_ros vehicle_config
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(sensor_sim_pub_node
  src/sensor_sim_pub_node.cpp
)
target_link_libraries(sensor_sim_pub_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# Mock 标定节点（模拟所有 8 个标定后端，供 GUI 可视化测试）
add_executable(mock_cali_nodes
  src/mock_cali_nodes.cpp
)
target_link_libraries(mock_cali_nodes
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS sensor_sim_pub_node mock_cali_nodes
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
