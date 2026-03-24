cmake_minimum_required(VERSION 3.0.2)
project(face_detector)

# 1. 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
)

# 2. 查找 OpenCV
find_package(OpenCV REQUIRED)

catkin_package()

# 3. 包含头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 4. 编译可执行文件
add_executable(face_detector src/face_detector.cpp)

# 5. 链接库文件（千万别忘了 ${OpenCV_LIBRARIES}）
target_link_libraries(face_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)