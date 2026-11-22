cmake_minimum_required(VERSION 3.5)
project(image_lidar_calibration)

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
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  camera_model
)

find_package(PCL 1.8 REQUIRED COMPONENTS
  common filters segmentation search kdtree features registration)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
foreach(_ceres_dir
        /usr/lib/cmake/Ceres
        /usr/lib/x86_64-linux-gnu/cmake/Ceres
        /usr/local/lib/cmake/Ceres)
  if(EXISTS "${_ceres_dir}/CeresConfig.cmake")
    set(Ceres_DIR "${_ceres_dir}" CACHE PATH "" FORCE)
    break()
  endif()
endforeach()
find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES image_lidar_calibration_lib
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs std_srvs geometry_msgs
    visualization_msgs cv_bridge image_transport pcl_ros pcl_conversions
    camera_model
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(image_lidar_calibration_lib
  src/fastlivo2_detector.cpp
  src/lidar_processor.cpp
  src/calibration_solver.cpp
)
target_link_libraries(image_lidar_calibration_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
)

add_executable(image_lidar_calib_node src/image_lidar_calib_node.cpp)
target_link_libraries(image_lidar_calib_node
  image_lidar_calibration_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CERES_LIBRARIES}
)

install(TARGETS image_lidar_calibration_lib image_lidar_calib_node
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
