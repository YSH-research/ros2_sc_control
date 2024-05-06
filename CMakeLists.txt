cmake_minimum_required(VERSION 3.5)
project(SC_Control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
#add_compile_options(-std=c++11)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

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

ament_target_dependencies(target serial)
ament_package()