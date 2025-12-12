cmake_minimum_required(VERSION 3.8)
project(kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# Component Library
add_library(kinematics_component SHARED src/node.cpp)
target_include_directories(kinematics_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(kinematics_component rclcpp rclcpp_components geometry_msgs std_msgs)

# Executable
add_executable(kinematics_node src/main.cpp)
target_link_libraries(kinematics_node kinematics_component)
ament_target_dependencies(kinematics_node rclcpp geometry_msgs std_msgs)

# Install
install(TARGETS kinematics_component kinematics_node
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