cmake_minimum_required(VERSION 3.8)
project(micon_driver_fd)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(serial_writer_lib src/serial_writer.cpp)
target_compile_features(serial_writer_lib PUBLIC cxx_std_17)
target_include_directories(serial_writer_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(serial_writer_lib rclcpp std_msgs)

add_executable(serial_writer src/main.cpp)
target_compile_features(serial_writer PUBLIC cxx_std_17)
target_link_libraries(serial_writer serial_writer_lib)
ament_target_dependencies(serial_writer rclcpp)

install(TARGETS serial_writer_lib serial_writer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

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

  ament_add_gtest(test_serial_writer test/test_serial_writer.cpp)
  target_link_libraries(test_serial_writer serial_writer_lib util)
  ament_target_dependencies(test_serial_writer rclcpp std_msgs)
endif()

ament_package()
