cmake_minimum_required(VERSION 3.0.2) # 指定 CMake 编译的最低版本要求
project(gmsl_camera_driver)           # 定义 ROS 功能包的名称

# 1. 查找依赖项
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
)
find_package(OpenCV REQUIRED)

catkin_package()

# 2. 指定头文件路径
include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS} 
)

# 3. 编译可执行文件并链接库
add_executable(gmsl_camera_node src/gmsl_camera_node.cpp)
target_link_libraries(gmsl_camera_node
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
)