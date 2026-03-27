cmake_minimum_required(VERSION 2.8.3)
project(perception_calibration)

## Compile as C++14, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O2 -g -Wall")

find_package(catkin REQUIRED
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  eigen_conversions
  pcl_ros
  pcl_conversions
  cameracalib_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML REQUIRED yaml-cpp)
message(STATUS "YAML=${YAML_INCLUDE_DIRS} ${YAML_LIBRARIES}")

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}_util
 CATKIN_DEPENDS roscpp std_msgs sensor_msgs image_transport pcl_ros pcl_conversions cv_bridge
 DEPENDS system_lib YAML
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_INCLUDE_DIRS}
)
# include_directories(/usr/include/yaml-cpp)
# link_directories(/usr/lib)

###########
## Build ##
###########
add_executable(${PROJECT_NAME}_node
  src/perception_calibration.cpp
  src/util.cpp
  src/dense_cloud_generator.cpp
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  yaml-cpp
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_library(${PROJECT_NAME}_util
  src/util.cpp)
target_link_libraries(${PROJECT_NAME}_util
  yaml-cpp
  ${catkin_LIBRARIES}
)
#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############

