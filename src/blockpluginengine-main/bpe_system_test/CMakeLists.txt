cmake_minimum_required(VERSION 3.11)
project(bpe_system_test)

find_package(ament_cmake REQUIRED)
find_package(block_plugin_engine REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(bpe_system_test_plugins SHARED
  src/twist_producer_block.cpp
  src/twist_transform_block.cpp
  src/twist_checker_block.cpp
)

target_compile_features(
  bpe_system_test_plugins
  PUBLIC cxx_std_20
)

target_include_directories(
  bpe_system_test_plugins
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(
  bpe_system_test_plugins
  block_plugin_engine
  pluginlib
  geometry_msgs
)

pluginlib_export_plugin_description_file(
  block_plugin_engine
  plugins.xml
)

install(
  TARGETS bpe_system_test_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(yaml-cpp REQUIRED)
  add_subdirectory(test)
endif()

ament_package()
