cmake_minimum_required(VERSION 3.8)
project(my_cpp_package)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(block_plugin_engine REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(my_cpp_bpe_plugins SHARED
  src/bug2_controller_block.cpp
  src/pp_bug2.cpp
  src/pure_pursuit_block.cpp
  src/timed_elastic_band_block.cpp
  src/twist_scale_block.cpp
)
target_compile_features(my_cpp_bpe_plugins PUBLIC cxx_std_20)
ament_target_dependencies(my_cpp_bpe_plugins
  block_plugin_engine
  geometry_msgs
  pluginlib
  sensor_msgs
  visualization_msgs
)

pluginlib_export_plugin_description_file(block_plugin_engine plugins.xml)

install(TARGETS my_cpp_bpe_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/bug2_example.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
