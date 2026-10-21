from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    package_share = FindPackageShare("my_cpp_package")
    config_file = PathJoinSubstitution(
        [package_share, "config", "pure_pursuit_bpe.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "config", "rviz.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the pure-pursuit visualization.",
            ),
            Node(
                package="block_plugin_engine",
                executable="block_plugin_engine",
                name="block_plugin_engine",
                parameters=[
                    {
                        "config_file": config_file,
                        "autostart": True,
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
