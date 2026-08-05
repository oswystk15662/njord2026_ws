cmake_minimum_required(VERSION 3.8)
project(robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY
  launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

# GLIM only accepts a single config directory containing a "config.json"
# that it reads at startup, so the CPU-backend variant needs its own
# directory (glim_config_cpu) rather than a config-file switch. Install a
# second copy of glim_config/ with config.json / config_ros.json swapped
# for their CPU-backend + headless-viewer counterparts.
install(DIRECTORY config/glim_config/
  DESTINATION share/${PROJECT_NAME}/config/glim_config_cpu
  PATTERN "config.json" EXCLUDE
  PATTERN "config_ros.json" EXCLUDE
)

install(FILES config/glim_config/config_cpu.json
  DESTINATION share/${PROJECT_NAME}/config/glim_config_cpu
  RENAME config.json
)

install(FILES config/glim_config/config_ros_cpu.json
  DESTINATION share/${PROJECT_NAME}/config/glim_config_cpu
  RENAME config_ros.json
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_livox_config test/test_livox_config.py)
  ament_add_pytest_test(test_task2_real_launch_static test/test_task2_real_launch_static.py)
  ament_add_pytest_test(test_task2_autonomy_launch_static test/test_task2_autonomy_launch_static.py)
  ament_add_pytest_test(
    test_task2_bag_perception_launch_static
    test/test_task2_bag_perception_launch_static.py
  )
  ament_add_pytest_test(test_launch_roles test/test_launch_roles.py)
  ament_add_pytest_test(test_platform_manifest test/test_platform_manifest.py)
endif()

ament_package()
