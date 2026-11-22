cmake_minimum_required(VERSION 3.5)
project(lidar_gnss_yaw)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  pcl_ros
  pcl_conversions
)

find_package(PCL 1.8 REQUIRED COMPONENTS
  common
  filters
  segmentation
  search
  kdtree
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp sensor_msgs std_msgs std_srvs geometry_msgs
    nav_msgs tf2 tf2_ros pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# ============================================================
# 公共库（算法层，供主节点和测试节点共用）
# ============================================================
add_library(lidar_gnss_yaw_lib
  src/cone_detector.cpp
  src/yaw_calibrator.cpp
)
target_link_libraries(lidar_gnss_yaw_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ============================================================
# 主节点
# ============================================================
add_executable(lidar_gnss_yaw_node
  src/lidar_gnss_yaw_node.cpp
)
target_link_libraries(lidar_gnss_yaw_node
  lidar_gnss_yaw_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ============================================================
# 测试节点（合成快照 + RViz 可视化）
# ============================================================
add_executable(gnss_yaw_test_node
  src/gnss_yaw_test_node.cpp
)
target_link_libraries(gnss_yaw_test_node
  lidar_gnss_yaw_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS lidar_gnss_yaw_lib lidar_gnss_yaw_node gnss_yaw_test_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
