cmake_minimum_required(VERSION 3.10)
project(VideoPusher LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)

add_executable(stream_pusher
    src/main.cpp
    src/VideoStreamer.cpp
)

target_include_directories(stream_pusher
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(stream_pusher
    PRIVATE
        ${OpenCV_LIBS}
)
