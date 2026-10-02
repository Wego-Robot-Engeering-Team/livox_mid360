# SPDX-License-Identifier: MIT
# ROS 2 build integration based on Livox's upstream CMakeLists.txt.
# Upstream C++ and message sources are compiled without modification.
cmake_minimum_required(VERSION 3.16)
project(livox_ros_driver2)

if(NOT DEFINED Python3_EXECUTABLE)
  set(Python3_EXECUTABLE /usr/bin/python3 CACHE FILEPATH "Python interpreter for ROS")
endif()
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(driver_source "${CMAKE_CURRENT_SOURCE_DIR}/livox_ros_driver2")
set(sdk_source "${CMAKE_CURRENT_SOURCE_DIR}/Livox-SDK2")
foreach(required_file
    "${driver_source}/src/driver_node.cpp"
    "${driver_source}/msg/CustomMsg.msg"
    "${sdk_source}/sdk_core/CMakeLists.txt")
  if(NOT EXISTS "${required_file}")
    message(FATAL_ERROR
      "Missing official sources: ${required_file}. Run vcs import . < dependencies.repos from the repository root first.")
  endif()
endforeach()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcutils REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(PCL REQUIRED)
find_package(Threads REQUIRED)

# Build and install SDK libraries in the same colcon package prefix as the
# driver. Referencing their CMake targets gives the correct build ordering and
# header/library paths without /usr/local or a separate SDK installation step.
# Only SDK core is needed; upstream example executables are not built.
add_subdirectory("${sdk_source}/sdk_core" "${CMAKE_CURRENT_BINARY_DIR}/sdk_core")
target_link_libraries(livox_lidar_sdk_shared Threads::Threads)
target_link_libraries(livox_lidar_sdk_static Threads::Threads)

set(interface_target livox_interfaces2)
rosidl_generate_interfaces(${interface_target}
  "${driver_source}:msg/CustomPoint.msg"
  "${driver_source}:msg/CustomMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs
  LIBRARY_NAME ${PROJECT_NAME}
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${interface_target} rosidl_typesupport_cpp)

set(driver_sources
  src/livox_ros_driver2.cpp
  src/lddc.cpp
  src/driver_node.cpp
  src/lds.cpp
  src/lds_lidar.cpp
  src/comm/comm.cpp
  src/comm/ldq.cpp
  src/comm/semaphore.cpp
  src/comm/lidar_imu_data_queue.cpp
  src/comm/cache_index.cpp
  src/comm/pub_handler.cpp
  src/parse_cfg_file/parse_cfg_file.cpp
  src/parse_cfg_file/parse_livox_lidar_cfg.cpp
  src/call_back/lidar_common_callback.cpp
  src/call_back/livox_lidar_callback.cpp
)
list(TRANSFORM driver_sources PREPEND "${driver_source}/")
add_library(${PROJECT_NAME} SHARED ${driver_sources})
target_compile_definitions(${PROJECT_NAME} PRIVATE BUILDING_ROS2)
target_compile_options(${PROJECT_NAME} PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wno-unused-parameter>)
target_include_directories(${PROJECT_NAME} PRIVATE
  "${driver_source}/src"
  "${driver_source}/3rdparty"
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_components std_msgs builtin_interfaces sensor_msgs
  rcutils pcl_conversions rcl_interfaces)
target_link_libraries(${PROJECT_NAME}
  livox_lidar_sdk_shared "${cpp_typesupport_target}"
  ${PCL_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "$ORIGIN")

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "livox_ros::DriverNode"
  EXECUTABLE livox_ros_driver2_node)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(FILES "${driver_source}/LICENSE.txt"
  DESTINATION share/${PROJECT_NAME}/licenses RENAME livox_ros_driver2.txt)
install(FILES "${sdk_source}/LICENSE.txt"
  DESTINATION share/${PROJECT_NAME}/licenses RENAME Livox-SDK2.txt)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
