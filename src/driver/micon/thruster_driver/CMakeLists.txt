cmake_minimum_required(VERSION 3.8)
project(thruster_driver)

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
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD QUIET libgpiodcxx)

include_directories(include)

add_library(thruster_driver_component SHARED src/node.cpp)
target_include_directories(thruster_driver_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(thruster_driver_component
  rclcpp rclcpp_components geometry_msgs std_msgs can_msgs)

add_executable(thruster_driver_node src/main.cpp)
target_link_libraries(thruster_driver_node thruster_driver_component)
ament_target_dependencies(thruster_driver_node rclcpp)

if(GPIOD_FOUND)
  add_executable(thruster_pid_node
    src/main_pid.cpp
    src/node_pid.cpp
    src/hardware_pwm.cpp
    src/velocity_controller.cpp
    src/mmg_dynamics.cpp
    src/thrust_allocator.cpp
  )
  ament_target_dependencies(thruster_pid_node
    rclcpp
    geometry_msgs
    nav_msgs
    std_msgs
  )
  target_link_libraries(thruster_pid_node
    ${GPIOD_LIBRARIES}
  )

  add_executable(thruster_4xomni_node
    src/main_4xomni.cpp
    src/node_4xomni.cpp
    src/mmg_model_4xomni.cpp
    src/software_pwm.cpp
  )
  ament_target_dependencies(thruster_4xomni_node
    rclcpp
    geometry_msgs
    nav_msgs
    std_msgs
  )
  target_link_libraries(thruster_4xomni_node
    ${GPIOD_LIBRARIES}
  )
else()
  message(WARNING "libgpiodcxx not found: skipping thruster_pid_node and thruster_4xomni_node")
endif()

add_executable(force_distributer_node
  src/force_distributer.cpp
)
ament_target_dependencies(force_distributer_node
  rclcpp
  sensor_msgs
  std_msgs
)

set(thruster_driver_targets
  thruster_driver_component
  thruster_driver_node
  force_distributer_node)

if(GPIOD_FOUND)
  list(APPEND thruster_driver_targets
    thruster_pid_node
    thruster_4xomni_node)
endif()

install(TARGETS
  ${thruster_driver_targets}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
