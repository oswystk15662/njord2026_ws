cmake_minimum_required(VERSION 3.8)
project(zed2i_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDAToolkit QUIET)

set(ZED2I_DRIVER_CUDA_VARIANT "none")
if(CUDAToolkit_FOUND)
  if(CUDAToolkit_VERSION VERSION_GREATER_EQUAL "13.0")
    set(ZED2I_DRIVER_CUDA_VARIANT "cuda13")
  elseif(CUDAToolkit_VERSION VERSION_GREATER_EQUAL "12.0")
    set(ZED2I_DRIVER_CUDA_VARIANT "cuda12")
  else()
    set(ZED2I_DRIVER_CUDA_VARIANT "legacy")
  endif()
  message(STATUS "${PROJECT_NAME}: detected CUDA Toolkit ${CUDAToolkit_VERSION} (${ZED2I_DRIVER_CUDA_VARIANT})")
else()
  message(STATUS "${PROJECT_NAME}: CUDA Toolkit not found; SDK node will be disabled unless ZED SDK resolves without CUDA")
endif()

set(DEPENDENCIES
  rclcpp
  sensor_msgs
)

add_library(${PROJECT_NAME}_common
  src/message_utils.cpp
)
target_include_directories(${PROJECT_NAME}_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_common ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_common ${OpenCV_LIBRARIES})

add_executable(zed2i_cpu_node
  src/cpu_stereo_node.cpp
)
target_include_directories(zed2i_cpu_node PRIVATE include)
ament_target_dependencies(zed2i_cpu_node ${DEPENDENCIES})
target_link_libraries(zed2i_cpu_node
  ${PROJECT_NAME}_common
  ${OpenCV_LIBRARIES}
)

option(ZED2I_DRIVER_BUILD_SDK_NODE "Build the ZED SDK backed node when the Stereolabs SDK is available" ON)

find_package(ZED QUIET)
set(ZED2I_DRIVER_USE_ZED_SDK OFF)
if(ZED2I_DRIVER_BUILD_SDK_NODE AND ZED_FOUND)
  set(ZED2I_DRIVER_USE_ZED_SDK ON)
endif()

if(ZED2I_DRIVER_USE_ZED_SDK)
  message(STATUS "${PROJECT_NAME}: ZED SDK found; building SDK node")
  if(CUDAToolkit_FOUND AND CUDAToolkit_VERSION VERSION_GREATER_EQUAL "13.0")
    message(STATUS "${PROJECT_NAME}: using CUDA 13.x compatible ZED SDK build path")
  elseif(CUDAToolkit_FOUND AND CUDAToolkit_VERSION VERSION_GREATER_EQUAL "12.0")
    message(STATUS "${PROJECT_NAME}: using CUDA 12.x compatible ZED SDK build path")
  endif()
  add_executable(zed2i_sdk_node
    src/sdk_node_zed.cpp
  )
  target_include_directories(zed2i_sdk_node PRIVATE
    include
    ${ZED_INCLUDE_DIRS}
    ${CUDAToolkit_INCLUDE_DIRS}
  )
  target_compile_definitions(zed2i_sdk_node PRIVATE
    ZED2I_DRIVER_HAS_ZED_SDK=1
    ZED2I_DRIVER_CUDA_VARIANT="${ZED2I_DRIVER_CUDA_VARIANT}"
  )
  ament_target_dependencies(zed2i_sdk_node ${DEPENDENCIES})
  target_link_libraries(zed2i_sdk_node
    ${PROJECT_NAME}_common
    ${OpenCV_LIBRARIES}
    ${ZED_LIBRARIES}
  )
else()
  if(NOT ZED2I_DRIVER_BUILD_SDK_NODE)
    message(STATUS "${PROJECT_NAME}: SDK node disabled by ZED2I_DRIVER_BUILD_SDK_NODE=OFF; building SDK-mode stub")
  else()
    message(STATUS "${PROJECT_NAME}: ZED SDK not found; building SDK-mode stub")
  endif()
  add_executable(zed2i_sdk_node
    src/sdk_node_stub.cpp
  )
  target_include_directories(zed2i_sdk_node PRIVATE include)
  ament_target_dependencies(zed2i_sdk_node ${DEPENDENCIES})
endif()

install(TARGETS
  ${PROJECT_NAME}_common
  zed2i_cpu_node
  zed2i_sdk_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
