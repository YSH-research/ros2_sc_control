cmake_minimum_required(VERSION 3.5)
project(sc_control)

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
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SelfcarCmd.msg"
  DEPENDENCIES
  std_msgs
)
# add_message_files(FILESselfcar_cmd.msg)


# catkin_package(
#     INCLUDE_DIRS include
# #  LIBRARIES targetless
# #    CATKIN_DEPENDS
# #  DEPENDS system_lib
# )



include_directories(include)
#include_directories( ${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE project_HEADERS
    "*.h"
    "*.hpp"
)

add_executable(
${PROJECT_NAME} ${project_HEADERS}
src/main.cpp 
src/sc_serial.cpp
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
#target_link_libraries(${PROJECT_NAME}_node ${cpp_typesupport_target})

# rosidl_target_interfaces(
#   ${PROJECT_NAME} 
#   ${PROJECT_NAME} 
#   "rosidl_typesupport_cpp"
#   )

 target_link_libraries(sc_control  ${catkin_LIBRARIES})

#ament_target_dependencies(${PROJECT_NAME})

ament_package() 
 # rclcpp
  # sensor_msgs
  # std_msgs
  # serial
  # tf2
  # nav_msgs
  # geometry_msgs
  # visualization_msgs