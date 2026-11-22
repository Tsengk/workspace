cmake_minimum_required(VERSION 3.5)
project(bev_calibration)

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
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED)

if(Boost_VERSION VERSION_GREATER_EQUAL 107300)
  set(AUTO_CALIB_BOOST_NEW_PLACEHOLDERS 1)
else()
  set(AUTO_CALIB_BOOST_NEW_PLACEHOLDERS 0)
endif()

if(OpenCV_VERSION VERSION_GREATER_EQUAL "4.0")
  set(AUTO_CALIB_OPENCV4_OR_NEWER 1)
else()
  set(AUTO_CALIB_OPENCV4_OR_NEWER 0)
endif()

set(EXTRACT_CB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../bev_surround_view/third_party/extract_chessboards)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport camera_model
  DEPENDS OpenCV Eigen3 yaml-cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${EXTRACT_CB_DIR}
)

add_executable(bev_calibration_node
  src/bev_calibration_node.cpp
  ${EXTRACT_CB_DIR}/CornerDetAC.cpp
  ${EXTRACT_CB_DIR}/ChessboradStruct.cpp
  ${EXTRACT_CB_DIR}/corealgmatlab.cpp
)
target_compile_definitions(bev_calibration_node PRIVATE
  AUTO_CALIB_BOOST_NEW_PLACEHOLDERS=${AUTO_CALIB_BOOST_NEW_PLACEHOLDERS}
)

target_link_libraries(bev_calibration_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

install(TARGETS bev_calibration_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
