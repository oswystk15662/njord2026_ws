cmake_minimum_required(VERSION 3.8)
project(simple_manual)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(joy_converter_lib src/joy_converter.cpp)
target_compile_features(joy_converter_lib PUBLIC cxx_std_17)
target_include_directories(joy_converter_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(joy_converter_lib rclcpp sensor_msgs geometry_msgs std_msgs)

add_executable(joy_converter_node src/main.cpp)
target_compile_features(joy_converter_node PUBLIC cxx_std_17)
target_link_libraries(joy_converter_node joy_converter_lib)
ament_target_dependencies(joy_converter_node rclcpp)

add_executable(command_arbiter_node src/command_arbiter.cpp)
target_compile_features(command_arbiter_node PUBLIC cxx_std_17)
ament_target_dependencies(command_arbiter_node rclcpp geometry_msgs std_msgs)

add_executable(thruster_id_test src/thruster_id_test.cpp)
target_compile_features(thruster_id_test PUBLIC cxx_std_17)
ament_target_dependencies(thruster_id_test rclcpp std_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_joy_converter test/test_joy_converter.cpp)
  target_link_libraries(test_joy_converter joy_converter_lib)
  ament_target_dependencies(test_joy_converter rclcpp sensor_msgs geometry_msgs std_msgs)
endif()

install(TARGETS
  joy_converter_lib
  joy_converter_node
  command_arbiter_node
  thruster_id_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
