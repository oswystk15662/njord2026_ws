cmake_minimum_required(VERSION 3.8)
project(drogger_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(include)

# Component Library
add_library(drogger_driver_component SHARED
  src/node.cpp
)

target_include_directories(drogger_driver_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(drogger_driver_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
)

target_link_libraries(drogger_driver_component
  Boost::system
)

# Executable
add_executable(drogger_driver_node src/main.cpp)
target_link_libraries(drogger_driver_node drogger_driver_component)

# Install rules
install(TARGETS drogger_driver_component drogger_driver_node
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