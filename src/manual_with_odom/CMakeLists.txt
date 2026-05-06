cmake_minimum_required(VERSION 3.8)
project(manual_with_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rclcpp_components REQUIRED)

add_library(joy_converter_component SHARED src/joy_converter.cpp)
target_compile_features(joy_converter_component PUBLIC cxx_std_17)
ament_target_dependencies(joy_converter_component rclcpp sensor_msgs geometry_msgs std_msgs rclcpp_components)

rclcpp_components_register_node(joy_converter_component PLUGIN "manual_with_odom::JoyConverter" EXECUTABLE joy_converter_component_node)

add_library(pid_force_controller_component SHARED src/pid_force_controller.cpp)
target_compile_features(pid_force_controller_component PUBLIC cxx_std_17)
ament_target_dependencies(pid_force_controller_component rclcpp geometry_msgs nav_msgs std_msgs rclcpp_components)

rclcpp_components_register_node(pid_force_controller_component PLUGIN "manual_with_odom::PIDController" EXECUTABLE pid_force_controller_component_node)

install(TARGETS
  joy_converter_component
  pid_force_controller_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
