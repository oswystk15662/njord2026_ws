cmake_minimum_required(VERSION 3.8)
project(simple_manual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

add_library(joy2force_component SHARED
  src/force_distributer.cpp
)

target_compile_features(joy2force_component PUBLIC cxx_std_17)

ament_target_dependencies(joy2force_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
)

rclcpp_components_register_node(joy2force_component
  PLUGIN "simple_manual::Joy2Force"
  EXECUTABLE joy2force_component_node
)

install(TARGETS joy2force_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# additional nodes
add_executable(joy_converter_node src/joy_converter.cpp)
target_compile_features(joy_converter_node PUBLIC cxx_std_17)
ament_target_dependencies(joy_converter_node rclcpp sensor_msgs geometry_msgs std_msgs)

add_executable(serial_writer_node src/main.cpp)
target_compile_features(serial_writer_node PUBLIC cxx_std_17)
ament_target_dependencies(serial_writer_node rclcpp std_msgs std_msgs sensor_msgs)

install(TARGETS
  joy_converter_node
  serial_writer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
