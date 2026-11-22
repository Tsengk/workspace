cmake_minimum_required(VERSION 3.5)
project(ackermann_canopen_hardware)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(canopen_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED src/ackermann_canopen_system.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface pluginlib rclcpp std_msgs std_srvs canopen_interfaces)
pluginlib_export_plugin_description_file(hardware_interface hardware_plugin.xml)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/mock_canopen_drivers.py scripts/prepare_canopen_config.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ackermann_canopen_system test/test_ackermann_canopen_system.cpp)
  target_link_libraries(test_ackermann_canopen_system ${PROJECT_NAME})
  ament_target_dependencies(test_ackermann_canopen_system
    hardware_interface rclcpp std_msgs std_srvs canopen_interfaces)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface)
ament_package()
