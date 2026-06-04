cmake_minimum_required(VERSION 3.8)
project(path_marker_visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(full_path_publisher src/full_path_publisher.cpp)
ament_target_dependencies(full_path_publisher
  rclcpp visualization_msgs tf2 tf2_ros geometry_msgs
)

add_executable(timed_path_publisher src/timed_path_publisher.cpp)
ament_target_dependencies(timed_path_publisher
  rclcpp visualization_msgs tf2 tf2_ros geometry_msgs
)

install(TARGETS full_path_publisher timed_path_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
