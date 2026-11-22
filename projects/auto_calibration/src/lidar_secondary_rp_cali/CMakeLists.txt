cmake_minimum_required(VERSION 3.5)
project(lidar_secondary_rp_cali)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  tf2_ros
  pcl_ros
  pcl_conversions
  lidar_rp_cali
  devkit_shm_param
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
  CATKIN_DEPENDS
    roscpp sensor_msgs std_msgs std_srvs geometry_msgs
    visualization_msgs tf2_ros pcl_ros pcl_conversions
    lidar_rp_cali
    devkit_shm_param
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ============================================================
# 主节点
# ============================================================
add_executable(lidar_secondary_rp_cali_node
  src/lidar_secondary_rp_cali_node.cpp
)
target_link_libraries(lidar_secondary_rp_cali_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ============================================================
# 安装
# ============================================================
install(TARGETS lidar_secondary_rp_cali_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
