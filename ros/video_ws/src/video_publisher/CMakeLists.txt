cmake_minimum_required(VERSION 3.0.2)
project(video_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(video_publisher src/video_publisher.cpp)

target_link_libraries(video_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)