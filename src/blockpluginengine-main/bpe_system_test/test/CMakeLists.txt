ament_add_gtest(test_twist_pipeline
  test_twist_pipeline.cpp
)

target_compile_features(test_twist_pipeline PUBLIC cxx_std_20)

add_dependencies(
  test_twist_pipeline
  bpe_system_test_plugins
)

target_include_directories(test_twist_pipeline
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include
)

ament_target_dependencies(test_twist_pipeline
  block_plugin_engine
  pluginlib
)

target_link_libraries(test_twist_pipeline
  yaml-cpp
)
