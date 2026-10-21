cmake_minimum_required(VERSION 3.8)
project(block_plugin_engine)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(bpe_blackboard REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

#
# Shared BPE core state
#
# The process-wide blackboard must live in one shared library so every plugin
# loaded through pluginlib observes the same instance.
#

add_library(block_plugin_engine_core SHARED
  src/blackboard.cpp
  src/ros_type_registry.cpp
)

target_compile_features(block_plugin_engine_core PUBLIC cxx_std_20)

target_include_directories(block_plugin_engine_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(block_plugin_engine_core
  rclcpp
  rosidl_runtime_cpp
)

target_link_libraries(block_plugin_engine_core
  bpe_blackboard::bpe_blackboard
)

#
# Default scheduler implementation
#
# IBlockScheduler is public API, but BPE_SimpleScheduler is an internal
# implementation.  It is built as a shared library so the scheduler can
# remain physically separated from the engine executable.
#

add_library(bpe_simple_scheduler SHARED
  src/bpe_simple_scheduler.cpp
)

target_compile_features(bpe_simple_scheduler PUBLIC cxx_std_20)

target_include_directories(bpe_simple_scheduler
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(bpe_simple_scheduler
  Threads::Threads
)

#
# BlockPluginEngine executable
#
# BPECompositionRoot, BPE_LifecycleNode and BlockPluginManager are internal
# implementation details and are compiled directly into the executable.
#

add_executable(block_plugin_engine
  src/main.cpp
  src/bpe_composition_root.cpp
  src/bpe_lifecycle_node.cpp
  src/block_plugin_manager.cpp
)

target_compile_features(block_plugin_engine PUBLIC cxx_std_20)

target_include_directories(block_plugin_engine
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(block_plugin_engine
  rclcpp
  rclcpp_lifecycle
  pluginlib
)

target_link_libraries(block_plugin_engine
  block_plugin_engine_core
  bpe_simple_scheduler
  yaml-cpp
)

#
# Install only the public API headers.
#

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    block_plugin_engine_core
    bpe_simple_scheduler
    block_plugin_engine
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#
# Public dependencies for packages implementing Blocks.
#
# yaml-cpp is exported because BlockConfig is part of the public API and is
# currently implemented as an alias of YAML::Node.
#

ament_export_include_directories(include)
ament_export_libraries(block_plugin_engine_core)

ament_export_dependencies(
  bpe_blackboard
  rclcpp
  rclcpp_lifecycle
  rosidl_runtime_cpp
  yaml-cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(geometry_msgs REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
