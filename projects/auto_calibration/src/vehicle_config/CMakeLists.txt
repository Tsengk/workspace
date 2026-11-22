cmake_minimum_required(VERSION 3.0.2)
project(vehicle_config)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES vehicle_config_lib
  DEPENDS Eigen3 yaml-cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${YAML_CPP_LIBRARY_DIRS}
)

add_library(vehicle_config_lib
  src/types.cpp
  src/config_loader.cpp
)

target_link_libraries(vehicle_config_lib
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS vehicle_config_lib
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)