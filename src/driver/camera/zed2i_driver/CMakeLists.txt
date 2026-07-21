cmake_minimum_required(VERSION 3.8)
project(zed2i_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(njord_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree segmentation)
find_package(OpenCV REQUIRED)
find_package(CUDAToolkit QUIET)
include(CheckLanguage)

option(ZED2I_DRIVER_BUILD_GPU_PERCEPTION
  "Build CUDA/TensorRT buoy perception when all GPU dependencies are available" ON)
find_path(ZED2I_DRIVER_NVINFER_INCLUDE_DIR NvInfer.h)
find_library(ZED2I_DRIVER_NVINFER_LIBRARY nvinfer)

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
  geometry_msgs
  njord_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
)

add_library(${PROJECT_NAME}_common
  src/message_utils.cpp
  src/perception_logic.cpp
  src/lidar_fusion.cpp
  src/detection_message_utils.cpp
)
set_target_properties(${PROJECT_NAME}_common PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(${PROJECT_NAME}_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_common SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_common ${DEPENDENCIES})
target_link_libraries(${PROJECT_NAME}_common ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_library(zed2i_cpu_component SHARED
  src/cpu_stereo_node.cpp
)
target_include_directories(zed2i_cpu_component PRIVATE include)
ament_target_dependencies(zed2i_cpu_component ${DEPENDENCIES} rclcpp_components)
target_link_libraries(zed2i_cpu_component
  ${PROJECT_NAME}_common
  ${OpenCV_LIBRARIES}
)
rclcpp_components_register_node(zed2i_cpu_component
  PLUGIN "zed2i_driver::CpuStereoNode"
  EXECUTABLE zed2i_cpu_node
)

option(ZED2I_DRIVER_BUILD_SDK_NODE "Build the ZED SDK backed node when the Stereolabs SDK is available" ON)

# Stereolabs installs a lower-case package configuration file
# (``/usr/local/zed/zed-config.cmake``), not ``ZEDConfig.cmake``.  Do not try
# to load it with an incompatible CUDA toolkit: the SDK config calls
# ``find_package(CUDA <SDK-major> REQUIRED)`` internally and would otherwise
# make the CPU/stub fallback impossible to configure.
set(ZED2I_DRIVER_USE_ZED_SDK OFF)
if(ZED2I_DRIVER_BUILD_SDK_NODE)
  set(_zed_config "/usr/local/zed/zed-config.cmake")
  if(DEFINED ZED_DIR AND EXISTS "${ZED_DIR}/zed-config.cmake")
    set(_zed_config "${ZED_DIR}/zed-config.cmake")
  endif()
  if(EXISTS "${_zed_config}")
    file(STRINGS "${_zed_config}" _zed_cuda_version
      REGEX "^[ \\t]*SET\\(ZED_CUDA_VERSION[ \\t]+[0-9]+\\)")
    string(REGEX MATCH "[0-9]+" _zed_cuda_major "${_zed_cuda_version}")
  endif()

  if(_zed_cuda_major AND CUDAToolkit_FOUND AND
     CUDAToolkit_VERSION_MAJOR EQUAL _zed_cuda_major)
    get_filename_component(ZED_DIR "${_zed_config}" DIRECTORY)
    find_package(zed QUIET CONFIG PATHS "${ZED_DIR}" NO_DEFAULT_PATH)
    if(ZED_FOUND)
      set(ZED2I_DRIVER_USE_ZED_SDK ON)
    endif()
  elseif(_zed_cuda_major)
    message(STATUS
      "${PROJECT_NAME}: ZED SDK requires CUDA ${_zed_cuda_major}.x but the "
      "detected CUDA Toolkit is ${CUDAToolkit_VERSION}; building SDK stub")
  endif()
endif()

set(ZED2I_DRIVER_HAS_GPU_PERCEPTION OFF)
if(ZED2I_DRIVER_BUILD_GPU_PERCEPTION AND ZED2I_DRIVER_USE_ZED_SDK AND
   CUDAToolkit_FOUND AND ZED2I_DRIVER_NVINFER_INCLUDE_DIR AND
   ZED2I_DRIVER_NVINFER_LIBRARY)
  check_language(CUDA)
  if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    set(ZED2I_DRIVER_HAS_GPU_PERCEPTION ON)
  endif()
endif()
if(ZED2I_DRIVER_HAS_GPU_PERCEPTION)
  message(STATUS "${PROJECT_NAME}: GPU perception enabled (ZED + CUDA compiler/toolkit + TensorRT)")
else()
  message(STATUS "${PROJECT_NAME}: GPU perception unavailable; requires ZED, CUDA compiler/toolkit, NvInfer.h, and libnvinfer")
endif()

if(ZED2I_DRIVER_USE_ZED_SDK)
  message(STATUS "${PROJECT_NAME}: ZED SDK found; building SDK node")
  if(CUDAToolkit_FOUND AND CUDAToolkit_VERSION VERSION_GREATER_EQUAL "13.0")
    message(STATUS "${PROJECT_NAME}: using CUDA 13.x compatible ZED SDK build path")
  elseif(CUDAToolkit_FOUND AND CUDAToolkit_VERSION VERSION_GREATER_EQUAL "12.0")
    message(STATUS "${PROJECT_NAME}: using CUDA 12.x compatible ZED SDK build path")
  endif()
  add_library(zed2i_sdk_component SHARED
    src/sdk_node_zed.cpp
  )
  target_include_directories(zed2i_sdk_component PRIVATE
    include
    ${ZED_INCLUDE_DIRS}
    ${CUDAToolkit_INCLUDE_DIRS}
  )
  target_compile_definitions(zed2i_sdk_component PRIVATE
    ZED2I_DRIVER_HAS_ZED_SDK=1
    ZED2I_DRIVER_CUDA_VARIANT="${ZED2I_DRIVER_CUDA_VARIANT}"
  )
  ament_target_dependencies(zed2i_sdk_component ${DEPENDENCIES} rclcpp_components)
  target_link_libraries(zed2i_sdk_component
    ${PROJECT_NAME}_common
    ${OpenCV_LIBRARIES}
    ${ZED_LIBRARIES}
  )
  if(ZED2I_DRIVER_HAS_GPU_PERCEPTION)
    target_sources(zed2i_sdk_component PRIVATE
      src/tensor_rt_detector.cpp
      src/cuda_preprocess.cu
      src/cuda_depth_median.cu
    )
    target_compile_definitions(zed2i_sdk_component PRIVATE ZED2I_DRIVER_HAS_GPU_PERCEPTION=1)
    target_include_directories(zed2i_sdk_component PRIVATE ${ZED2I_DRIVER_NVINFER_INCLUDE_DIR})
    target_link_libraries(zed2i_sdk_component ${ZED2I_DRIVER_NVINFER_LIBRARY} CUDA::cudart)
  endif()
else()
  if(NOT ZED2I_DRIVER_BUILD_SDK_NODE)
    message(STATUS "${PROJECT_NAME}: SDK node disabled by ZED2I_DRIVER_BUILD_SDK_NODE=OFF; building SDK-mode stub")
  else()
    message(STATUS "${PROJECT_NAME}: ZED SDK not found; building SDK-mode stub")
  endif()
  add_library(zed2i_sdk_component SHARED
    src/sdk_node_stub.cpp
  )
  target_include_directories(zed2i_sdk_component PRIVATE include)
  ament_target_dependencies(zed2i_sdk_component ${DEPENDENCIES} rclcpp_components)
endif()

rclcpp_components_register_node(zed2i_sdk_component
  PLUGIN "zed2i_driver::SdkNode"
  EXECUTABLE zed2i_sdk_node
)

install(TARGETS
  ${PROJECT_NAME}_common
  zed2i_cpu_component
  zed2i_sdk_component
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
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_perception_logic test/test_perception_logic.cpp)
  if(TARGET test_perception_logic)
    target_link_libraries(test_perception_logic ${PROJECT_NAME}_common)
    target_include_directories(test_perception_logic PRIVATE include)
  endif()
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
