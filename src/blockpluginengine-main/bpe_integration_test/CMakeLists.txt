cmake_minimum_required(VERSION 3.8)
project(bpe_integration_test)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(
    test/test_configure_shutdown.py
    TARGET test_configure_shutdown_00_empty
    ARGS "config_name:=00_empty.yaml"
    TIMEOUT 30
  )

  add_launch_test(
    test/test_configure_shutdown.py
    TARGET test_configure_shutdown_10_internal_transform
    ARGS "config_name:=10_internal_transform.yaml"
    TIMEOUT 30
  )

  add_launch_test(
    test/test_configure_shutdown.py
    TARGET test_configure_shutdown_20_ros_input
    ARGS "config_name:=20_ros_input.yaml"
    TIMEOUT 30
  )

  add_launch_test(
    test/test_configure_shutdown.py
    TARGET test_configure_shutdown_30_ros_output
    ARGS "config_name:=30_ros_output.yaml"
    TIMEOUT 30
  )

  add_launch_test(
    test/test_twist_pipeline.py
    TARGET test_90_full_pipeline
    TIMEOUT 30
  )
endif()

ament_package()
