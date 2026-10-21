import time
import unittest

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, LifecycleTransition
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import GetState
import rclpy


def generate_test_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('bpe_integration_test'),
        'config',
        LaunchConfiguration('config_name'),
    ])

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

    configure = LifecycleTransition(
        lifecycle_node_names=['block_plugin_engine'],
        transition_ids=[Transition.TRANSITION_CONFIGURE],
    )

    return (
        LaunchDescription([
            DeclareLaunchArgument('config_name'),
            engine,
            configure,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'engine': engine},
    )


class TestConfigureShutdown(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('bpe_configure_shutdown_test')

    def tearDown(self):
        self.node.destroy_node()

    def _wait_for_inactive(self, timeout_sec=10.0):
        client = self.node.create_client(GetState, '/block_plugin_engine/get_state')
        try:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                return False

            deadline = time.monotonic() + timeout_sec
            while time.monotonic() < deadline:
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)

                if future.done() and future.result() is not None:
                    if future.result().current_state.id == State.PRIMARY_STATE_INACTIVE:
                        return True

                time.sleep(0.05)

            return False
        finally:
            self.node.destroy_client(client)

    def test_configure_reaches_inactive(self):
        self.assertTrue(
            self._wait_for_inactive(),
            'BPE lifecycle node did not reach inactive state after configure',
        )


@launch_testing.post_shutdown_test()
class TestConfigureShutdownExit(unittest.TestCase):
    def test_exit_code(self, proc_info, engine):
        launch_testing.asserts.assertExitCodes(proc_info, process=engine)
