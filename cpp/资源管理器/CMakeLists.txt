cmake_minimum_required(VERSION 3.10)
project(RAII_Manager_Project)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

add_executable(raii_manager src/main.cpp src/FileHandler.cpp)