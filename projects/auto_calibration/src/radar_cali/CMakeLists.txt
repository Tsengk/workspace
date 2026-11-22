cmake_minimum_required(VERSION 3.5)
project(radar_cali)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  std_srvs
  pcl_ros
  pcl_conversions
)

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters sample_consensus)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES radar_cali_lib
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs std_srvs pcl_ros pcl_conversions
  DEPENDS Eigen3 PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

# ---- 算法库 ----
add_library(radar_cali_lib
  src/types.cpp
  src/radar_calibrator.cpp
)

target_link_libraries(radar_cali_lib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ---- 主节点 ----
add_executable(radar_cali_node src/radar_cali_node.cpp)
target_link_libraries(radar_cali_node
  radar_cali_lib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ---- 自测节点 ----
add_executable(radar_cali_test_node src/radar_cali_test_node.cpp)
target_link_libraries(radar_cali_test_node
  radar_cali_lib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS radar_cali_lib radar_cali_node radar_cali_test_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)