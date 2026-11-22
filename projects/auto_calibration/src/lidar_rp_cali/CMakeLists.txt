cmake_minimum_required(VERSION 3.5)
project(lidar_rp_cali)

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
  INCLUDE_DIRS include
  LIBRARIES lidar_rp_cali_lib
  CATKIN_DEPENDS
    roscpp sensor_msgs std_msgs std_srvs geometry_msgs
    visualization_msgs tf2_ros pcl_ros pcl_conversions
    devkit_shm_param
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# ============================================================
# 公共库
# ============================================================
add_library(lidar_rp_cali_lib
  src/ground_detector.cpp
  src/plane_fitter.cpp
  src/calibration_calculator.cpp
  src/lidar_simulator.cpp
)
target_link_libraries(lidar_rp_cali_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ============================================================
# 主节点（订阅真实 LiDAR 话题）
# ============================================================
add_executable(lidar_rp_cali_node
  src/lidar_rp_cali_node.cpp
)
target_link_libraries(lidar_rp_cali_node
  lidar_rp_cali_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ============================================================
# 手动点击地面点标定节点（订阅 /clicked_point）
# ============================================================
add_executable(lidar_rp_cali_clicked_node
  src/lidar_rp_cali_clicked_node.cpp
)
target_link_libraries(lidar_rp_cali_clicked_node
  lidar_rp_cali_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ============================================================
# 测试节点（仿真 + 场景验证）
# ============================================================
add_executable(calibration_test_node
  src/calibration_test_node.cpp
)
target_link_libraries(calibration_test_node
  lidar_rp_cali_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ============================================================
# 安装
# ============================================================
install(TARGETS lidar_rp_cali_lib lidar_rp_cali_node lidar_rp_cali_clicked_node calibration_test_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
