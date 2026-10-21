from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    package_share = FindPackageShare("my_cpp_package")
    config_file = PathJoinSubstitution([package_share, "config", "bug2.yaml"])
    rviz_config = PathJoinSubstitution([package_share, "config", "bug2.rviz"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the Bug2 visualization.",
            ),
            DeclareLaunchArgument(
                "example",
                default_value="true",
                description="Run the Python obstacle and sensor simulator.",
            ),
            DeclareLaunchArgument(
                "wall_count",
                default_value="6",
                description="Number of randomized walls in the example simulator.",
            ),
            DeclareLaunchArgument(
                "wall_seed",
                default_value="23",
                description="Seed for repeatable wall placement.",
            ),
            Node(
                package="block_plugin_engine",
                executable="block_plugin_engine",
                name="block_plugin_engine",
                parameters=[
                    {"config_file": config_file, "autostart": True},
                ],
                output="screen",
            ),
            Node(
                package="my_cpp_package",
                executable="bug2_example.py",
                condition=IfCondition(LaunchConfiguration("example")),
                parameters=[
                    {
                        "wall_count": ParameterValue(
                            LaunchConfiguration("wall_count"), value_type=int
                        ),
                        "wall_seed": ParameterValue(
                            LaunchConfiguration("wall_seed"), value_type=int
                        ),
                    }
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
