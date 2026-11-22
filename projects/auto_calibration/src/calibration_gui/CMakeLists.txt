cmake_minimum_required(VERSION 3.5)
project(calibration_gui)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-O2 -Wall)

# Qt5
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  pcl_ros
  pcl_conversions
  vehicle_config
  devkit_shm_param
)

find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets OpenGL)
find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED COMPONENTS common io visualization)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# VTK (for 3D point cloud display via QVTKWidget)
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

if(VTK_VERSION VERSION_GREATER_EQUAL "9.0")
  set(AUTO_CALIB_VTK9_OR_NEWER 1)
else()
  set(AUTO_CALIB_VTK9_OR_NEWER 0)
endif()

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs std_srvs sensor_msgs geometry_msgs
                 cv_bridge image_transport pcl_ros pcl_conversions vehicle_config
                 devkit_shm_param
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

set(SOURCES
  src/main.cpp
  src/main_window.cpp
  src/config_manager.cpp
  src/ros_worker.cpp
  src/tab_lidar_rp.cpp
  src/tab_lidar_yaw.cpp
  src/tab_secondary_rp.cpp
  src/tab_lidar_gnss_yaw.cpp
  src/tab_lidar_lidar.cpp
  src/tab_image_lidar.cpp
  src/tab_image_lidar_bev.cpp
  src/tab_camera_cali.cpp
  src/tab_bev.cpp
  src/tab_front_stitch.cpp
  src/tab_radar_lidar.cpp
  src/point_cloud_viewer.cpp
  src/image_viewer.cpp
)

set(HEADERS
  include/calibration_gui/main_window.h
  include/calibration_gui/config_manager.h
  include/calibration_gui/ros_worker.h
  include/calibration_gui/tab_lidar_rp.h
  include/calibration_gui/tab_lidar_yaw.h
  include/calibration_gui/tab_secondary_rp.h
  include/calibration_gui/tab_lidar_gnss_yaw.h
  include/calibration_gui/tab_lidar_lidar.h
  include/calibration_gui/tab_image_lidar.h
  include/calibration_gui/tab_image_lidar_bev.h
  include/calibration_gui/tab_camera_cali.h
  include/calibration_gui/tab_bev.h
  include/calibration_gui/tab_front_stitch.h
  include/calibration_gui/tab_radar_lidar.h
  include/calibration_gui/point_cloud_viewer.h
  include/calibration_gui/image_viewer.h
  include/calibration_gui/wheel_guard.h
)

add_executable(calibration_gui ${SOURCES} ${HEADERS})

target_compile_definitions(calibration_gui PRIVATE
  AUTO_CALIB_VTK9_OR_NEWER=${AUTO_CALIB_VTK9_OR_NEWER}
)

target_link_libraries(calibration_gui
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${VTK_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
  Qt5::OpenGL
)

install(TARGETS calibration_gui
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ---- 防复发回归测试：WheelGuard 滚轮拦截逻辑 ----
add_executable(wheel_guard_test
  test/wheel_guard_test.cpp
  include/calibration_gui/wheel_guard.h
)
target_include_directories(wheel_guard_test PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(wheel_guard_test
  Qt5::Core
  Qt5::Gui
  Qt5::Widgets
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
