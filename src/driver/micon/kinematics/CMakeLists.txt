cmake_minimum_required(VERSION 3.8)
project(kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

# Component Library
add_library(kinematics_component SHARED src/node.cpp)
target_include_directories(kinematics_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(kinematics_component rclcpp rclcpp_components geometry_msgs std_msgs)

# Joy Converter Component
add_library(joy_converter_component SHARED src/joy_converter.cpp)
target_compile_features(joy_converter_component PUBLIC cxx_std_17)
ament_target_dependencies(joy_converter_component rclcpp sensor_msgs geometry_msgs std_msgs rclcpp_components)
rclcpp_components_register_node(joy_converter_component PLUGIN "manual_with_odom::JoyConverter" EXECUTABLE joy_converter_component_node)

# PID Force Controller Component
add_library(pid_force_controller_component SHARED src/pid_force_controller.cpp)
target_compile_features(pid_force_controller_component PUBLIC cxx_std_17)
ament_target_dependencies(pid_force_controller_component rclcpp geometry_msgs nav_msgs std_msgs rclcpp_components)
rclcpp_components_register_node(pid_force_controller_component PLUGIN "manual_with_odom::PIDController" EXECUTABLE pid_force_controller_component_node)

# Executable
add_executable(kinematics_node src/main.cpp)
target_link_libraries(kinematics_node kinematics_component)
ament_target_dependencies(kinematics_node rclcpp geometry_msgs std_msgs)

# Install
install(TARGETS
  kinematics_component
  kinematics_node
  joy_converter_component
  pid_force_controller_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()