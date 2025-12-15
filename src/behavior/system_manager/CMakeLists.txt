cmake_minimum_required(VERSION 3.8)
project(system_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(njord_interfaces REQUIRED)

# ライブラリの作成 (Component)
add_library(system_manager_component SHARED src/node.cpp)
target_include_directories(system_manager_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(system_manager_component
  rclcpp
  rclcpp_components
  rclcpp_action
  rcl_interfaces
  njord_interfaces
)

# 実行ファイルの作成
add_executable(system_manager src/main.cpp) # main関数だけ別ファイルにする場合
# または下記のようにNode登録マクロを使っているのであれば、専用のmainを書かずにRCLCPP_COMPONENTS_REGISTER_NODEのみで運用し、
# 汎用コンテナ(rclcpp_components)でロードすることも可能です。
# ここではわかりやすく、シンプルなmain.cppを用意してリンクする方式を書きます。

target_link_libraries(system_manager system_manager_component)
ament_target_dependencies(system_manager rclcpp rclcpp_components rclcpp_action rcl_interfaces njord_interfaces)

# インストール設定
install(TARGETS system_manager_component system_manager
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
