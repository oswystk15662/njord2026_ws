cmake_minimum_required(VERSION 3.8)
project(um982_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. パッケージの検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Boost (Asioなどのため)
find_package(Boost REQUIRED COMPONENTS system)

if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO is not set. Please build inside a ROS 2 environment.")
endif()

string(TOLOWER "$ENV{ROS_DISTRO}" UM982_ROS_DISTRO)
if(UM982_ROS_DISTRO STREQUAL "foxy")
  set(UM982_TF2_GEOMETRY_MSGS_HEADER "tf2_geometry_msgs.h")
elseif(UM982_ROS_DISTRO STREQUAL "humble" OR UM982_ROS_DISTRO STREQUAL "jazzy")
  set(UM982_TF2_GEOMETRY_MSGS_HEADER "tf2_geometry_msgs.hpp")
else()
  message(FATAL_ERROR "Unsupported ROS_DISTRO: $ENV{ROS_DISTRO}. Expected one of: foxy, humble, jazzy.")
endif()

message(STATUS "um982_driver: ROS_DISTRO=${UM982_ROS_DISTRO}, tf2_geometry_msgs header=${UM982_TF2_GEOMETRY_MSGS_HEADER}")

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/tf2_geometry_msgs_include.hpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/include/um982_driver/tf2_geometry_msgs_include.hpp
  @ONLY
)

include_directories(include)

# 2. ライブラリ（コンポーネント）の作成
add_library(um982_driver_component SHARED
  src/node.cpp
  src/utils.cpp
)

target_include_directories(um982_driver_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 3. 依存関係のリンク (ここを修正)
# ROS関連のパッケージは ament_target_dependencies でリンクします
ament_target_dependencies(um982_driver_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
)

# ROS以外のライブラリ（Boostなど）は target_link_libraries を使います
target_link_libraries(um982_driver_component
  Boost::system
)

# 4. 実行ファイルの作成
add_executable(um982_driver_node src/main.cpp)
target_include_directories(um982_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 実行ファイルにコンポーネントをリンク
target_link_libraries(um982_driver_node
  um982_driver_component
)

# 5. インストール設定
install(TARGETS um982_driver_component um982_driver_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_um982_utils test/test_utils.cpp)
  target_include_directories(test_um982_utils PRIVATE include)
  target_link_libraries(test_um982_utils um982_driver_component)
endif()

ament_package()
