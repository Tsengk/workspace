cmake_minimum_required(VERSION 3.5)
project(bev_surround_view)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  camera_model
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
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
find_package(yaml-cpp REQUIRED)

if(OpenCV_VERSION VERSION_GREATER_EQUAL "4.0")
  set(AUTO_CALIB_OPENCV4_OR_NEWER 1)
else()
  set(AUTO_CALIB_OPENCV4_OR_NEWER 0)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bev_surround_view_lib
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport camera_model
  DEPENDS OpenCV Eigen3 Ceres yaml-cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/extract_chessboards
)

add_library(bev_surround_view_lib
  src/calibration_utils.cpp
  src/chessboard_detector.cpp
  src/pose_utils.cpp
  src/joint_optimization.cpp
  src/fisheye_auto_calibrator.cpp
  third_party/extract_chessboards/ChessboradStruct.cpp
  third_party/extract_chessboards/CornerDetAC.cpp
  third_party/extract_chessboards/corealgmatlab.cpp
)
target_compile_definitions(bev_surround_view_lib PRIVATE
  AUTO_CALIB_OPENCV4_OR_NEWER=${AUTO_CALIB_OPENCV4_OR_NEWER}
)

target_link_libraries(bev_surround_view_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${CERES_LIBRARIES}
  yaml-cpp
)

add_executable(bev_surround_view_node src/bev_surround_view_node.cpp)
target_link_libraries(bev_surround_view_node
  bev_surround_view_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS bev_surround_view_lib bev_surround_view_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
