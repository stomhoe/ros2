cmake_minimum_required(VERSION 3.8)
project(bpe_ros_boundary_plugins)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(block_plugin_engine REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(bpe_ros_boundary_plugins SHARED
  src/ros_input_block.cpp
  src/ros_output_block.cpp
)

target_compile_features(bpe_ros_boundary_plugins PUBLIC cxx_std_20)

target_include_directories(bpe_ros_boundary_plugins
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

ament_target_dependencies(bpe_ros_boundary_plugins
  block_plugin_engine
  pluginlib
  rclcpp
)

pluginlib_export_plugin_description_file(
  block_plugin_engine
  plugins.xml
)

install(
  TARGETS bpe_ros_boundary_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
