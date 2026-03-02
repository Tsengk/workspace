cmake_minimum_required(VERSION 3.10)
project(MyBackendServer)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

add_executable(server src/main.cpp)