cmake_minimum_required(VERSION 3.10)

project(RoadBEV)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(
    road_bev
    main.cpp
)

target_include_directories(
    road_bev PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
    road_bev PRIVATE
    ${OpenCV_LIBS}
)
