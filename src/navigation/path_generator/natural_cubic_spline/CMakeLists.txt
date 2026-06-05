cmake_minimum_required(VERSION 3.8)
project(natural_cubic_spline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)

# Add the library
add_library(natural_cubic_spline SHARED
  src/natural_cubic_spline_smoother.cpp
)

# IMPORTANT: Disable LTO for pluginlib plugins.
# LTO (Link Time Optimization) breaks RTTI across shared library boundaries
# when used with pluginlib::ClassLoader, causing SIGSEGV at plugin load time.
# The colcon defaults.yaml may globally enable LTO via
# -DCMAKE_INTERPROCEDURAL_OPTIMIZATION=ON, so we override it per-target.
set_target_properties(natural_cubic_spline PROPERTIES
  INTERPROCEDURAL_OPTIMIZATION OFF
)

target_link_libraries(natural_cubic_spline Eigen3::Eigen)

ament_target_dependencies(natural_cubic_spline
  rclcpp
  nav2_core
  nav_msgs
  geometry_msgs
  tf2_ros
  nav2_costmap_2d
  pluginlib
)

target_include_directories(natural_cubic_spline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(
  TARGETS natural_cubic_spline
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugins.xml
  DESTINATION share/natural_cubic_spline
)

install(
  DIRECTORY launch config test
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS test/test_smoother_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
