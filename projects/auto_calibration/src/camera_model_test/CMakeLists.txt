cmake_minimum_required(VERSION 3.5)
project(camera_model_test)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

catkin_package(
  INCLUDE_DIRS 
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs cv_bridge image_transport camera_model
  DEPENDS OpenCV
)

include_directories(
  
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(camera_model_test_node src/camera_model_test_node.cpp)
target_link_libraries(camera_model_test_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
