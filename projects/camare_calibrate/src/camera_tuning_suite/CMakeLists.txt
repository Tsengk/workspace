cmake_minimum_required(VERSION 3.0.2)
project(camera_tuning_suite)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport sensor_msgs)
find_package(OpenCV REQUIRED)

catkin_package()
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# 编译模拟器
add_executable(gmsl_simulator src/gmsl_simulator.cpp)
target_link_libraries(gmsl_simulator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# 编译调参器
add_executable(tuner_h60 src/distortion_tuner_h60.cpp)
target_link_libraries(tuner_h60 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})