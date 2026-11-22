cmake_minimum_required(VERSION 3.0.2)
project(gassd_msgs_gsds)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder

# 递归获取所有.msg文件（带路径）
file(GLOB_RECURSE MSG_FILES_WITH_PATH "msg/*.msg")

# 提取纯文件名
set(MSG_FILES "")
foreach(msg_file ${MSG_FILES_WITH_PATH})
  get_filename_component(FILENAME_ONLY ${msg_file} NAME)
  list(APPEND MSG_FILES ${FILENAME_ONLY})
endforeach()

 add_message_files(
   FILES
   ${MSG_FILES}
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)


