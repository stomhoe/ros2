import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, LifecycleTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('bpe_integration_test'),
        'config',
        '90_full_pipeline.yaml',
    )

    engine = LifecycleNode(
        package='block_plugin_engine',
        executable='block_plugin_engine',
        name='block_plugin_engine',
        namespace='',
        output='screen',
        parameters=[{
            'config_file': config_file,
            'autostart': False,
        }],
    )

    bringup = LifecycleTransition(
        lifecycle_node_names=['block_plugin_engine'],
        transition_ids=[
            Transition.TRANSITION_CONFIGURE,
            Transition.TRANSITION_ACTIVATE,
        ],
    )

    return LaunchDescription([
        engine,
        bringup,
    ])
