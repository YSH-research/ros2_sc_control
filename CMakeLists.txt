cmake_minimum_required(VERSION 3.5)
project(SC_Control)


add_compile_options(-std=c++11)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()


find_package(catkin REQUIRED COMPONENTS
  rclcpp
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

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()