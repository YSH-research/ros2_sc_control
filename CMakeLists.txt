cmake_minimum_required(VERSION 3.5)
project(sc_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)   
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SelfcarCmd.msg"
  DEPENDENCIES std_msgs
)


# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

# ament_package(
#   INCLUDE_DIRS include
# )

# include_directories(include ${ament_INCLUDE_DIRS})

# file(GLOB_RECURSE project_HEADERS
#     "*.h"
#     "*.hpp"
# )

# add_executable(${PROJECT_NAME}_exe ${project_HEADERS} src/main.cpp src/sc_serial.cpp)

# ament_target_dependencies(${PROJECT_NAME}_exe
#   rclcpp
#   std_msgs
#   sensor_msgs
#   serial
#   tf2
#   nav_msgs
#   geometry_msgs
#   visualization_msgs
# )


# target_link_libraries(${PROJECT_NAME}_exe ${ament_LIBRARIES})


# ament_export_dependencies(rosidl_default_runtime)
# ament_export_include_directories(include)

ament_package()
