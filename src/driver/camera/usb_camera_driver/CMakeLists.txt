cmake_minimum_required(VERSION 3.8)
project(usb_camera_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED) # OpenCV for C++
find_package(rclcpp_components REQUIRED) # Add rclcpp_components

find_package(PkgConfig REQUIRED) # Gstreamer
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

# -----------------
# GStreamer component
# -----------------

add_library(usb_camera_driver_component SHARED src/composable_node.cpp)

target_include_directories(usb_camera_driver_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GSTREAMER_INCLUDE_DIRS}
)

# Link dependencies
ament_target_dependencies(usb_camera_driver_component
  "rclcpp_components"  # Add rclcpp_components as a dependency
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# GStreamerの依存関係を追加
target_link_libraries(usb_camera_driver_component
  ${GSTREAMER_LIBRARIES} # <-- GStreamerのライブラリをリンク
)

# Register as a composable node
rclcpp_components_register_nodes(
  usb_camera_driver_component
  PLUGIN "RPiCameraDriverNode"
  EXECUTABLE usb_camera_driver
)

# add for composable
ament_export_targets(export_usb_camera_driver_component)
install(TARGETS usb_camera_driver_component
        EXPORT export_usb_camera_driver_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# -----------------
# MJPG Streamer component
# -----------------

add_library(usb_camera_driver_mjpg_component SHARED src/via_mjpg_streamer.cpp)

target_include_directories(usb_camera_driver_mjpg_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(usb_camera_driver_mjpg_component
  "rclcpp_components"  # Add rclcpp_components as a dependency
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

rclcpp_components_register_nodes(
  usb_camera_driver_mjpg_component
  PLUGIN "RPiCameraDriverMJPGNode"
  EXECUTABLE usb_camera_driver_mjpg
)

ament_export_targets(export_usb_camera_driver_mjpg_component)
install(TARGETS usb_camera_driver_mjpg_component
        EXPORT export_usb_camera_driver_mjpg_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

# -----------------
# Common install and package logic
# -----------------

include_directories(
  include
  ${GSTREAMER_INCLUDE_DIRS}
)

# Install the launch directory
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
