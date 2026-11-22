cmake_minimum_required(VERSION 3.0.2)
project(lidar_lidar_cali)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2 -Wall)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  visualization_msgs
  pcl_ros
  pcl_conversions
  image_lidar_calibration
  devkit_shm_param
)

find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lidar_lidar_cali_lib
  CATKIN_DEPENDS
    roscpp sensor_msgs std_msgs std_srvs
    geometry_msgs visualization_msgs
    pcl_ros pcl_conversions
    image_lidar_calibration
    devkit_shm_param
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# 核心库（SVD 3D-3D 标定算法）
# add_library(lidar_lidar_cali_lib
#   src/lidar_cali.cpp
# )
# target_link_libraries(lidar_lidar_cali_lib
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )

# 主节点
# add_executable(lidar_lidar_cali_node
#   src/lidar_lidar_cali_node.cpp
#   src/lidar_cali.cpp
# )
# target_link_libraries(lidar_lidar_cali_node
#   lidar_lidar_cali_lib
#   ${catkin_LIBRARIES}
#   ${PCL_LIBRARIES}
# )

# 手动点击锥桶标定节点（订阅 /clicked_point，2D SVD）
add_executable(lidar_lidar_cali_clicked_node
  src/lidar_lidar_cali_clicked_node.cpp
  src/lidar_cali.cpp
)
target_link_libraries(lidar_lidar_cali_clicked_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


install(TARGETS   lidar_lidar_cali_clicked_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
