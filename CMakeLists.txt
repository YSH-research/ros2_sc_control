cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++11)

project(SC_Control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  serial
  tf2
  nav_msgs
  geometry_msgs
  visualization_msgs
)

add_message_files(
  FILES
  selfcar_cmd.msg

)

generate_messages(
  DEPENDENCIES
  std_msgs
    # Or other packages containing msgs
)

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES targetless
#    CATKIN_DEPENDS
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE project_HEADERS
    "*.h"
    "*.hpp"
)

add_executable(
${PROJECT_NAME}
${project_HEADERS}
src/main.cpp 
src/sc_serial.cpp
)

target_link_libraries(SC_Control  ${catkin_LIBRARIES})
