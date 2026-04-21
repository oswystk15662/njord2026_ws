cmake_minimum_required(VERSION 3.8)
project(dutyed_tf_pub_with_disturbance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(${PROJECT_NAME}_models SHARED
  src/t200_model.cpp
  src/mmg_doyle_model.cpp
  src/disturbance_model.cpp
)
target_include_directories(${PROJECT_NAME}_models PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(${PROJECT_NAME}_node_component SHARED
  src/sim_node.cpp
)
target_include_directories(${PROJECT_NAME}_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_node_component ${PROJECT_NAME}_models)
ament_target_dependencies(${PROJECT_NAME}_node_component
  rclcpp
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  geometry_msgs
)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_node_component)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)

install(
  TARGETS
    ${PROJECT_NAME}_models
    ${PROJECT_NAME}_node_component
    ${PROJECT_NAME}_node
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

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_models)

ament_package()
