cmake_minimum_required(VERSION 3.10)
project(LightweightWebServer)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

add_executable(WebServer main.cpp HttpHandler.cpp)
target_link_libraries(WebServer Threads::Threads)