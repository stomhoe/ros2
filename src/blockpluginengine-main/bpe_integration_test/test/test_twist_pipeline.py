import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, LifecycleTransition
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import GetState
import rclpy


def generate_test_description():
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

    return (
        LaunchDescription([
            engine,
            bringup,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'engine': engine},
    )


class TestTwistPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('bpe_twist_pipeline_test')
        self.received = []

    def tearDown(self):
        self.node.destroy_node()

    def _on_output(self, message):
        self.received.append(message)

    def _spin_until(self, predicate, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return predicate()

    def _wait_for_active(self, timeout_sec=10.0):
        client = self.node.create_client(GetState, '/block_plugin_engine/get_state')
        try:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                return False

            deadline = time.monotonic() + timeout_sec
            while time.monotonic() < deadline:
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)

                if future.done() and future.result() is not None:
                    if future.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
                        return True

                time.sleep(0.05)

            return False
        finally:
            self.node.destroy_client(client)

    def test_ros_input_transform_output(self):
        self.assertTrue(self._wait_for_active(), 'BPE lifecycle node did not reach active state')

        subscription = self.node.create_subscription(
            TwistStamped,
            '/bpe_test/output',
            self._on_output,
            10,
        )
        publisher = self.node.create_publisher(TwistStamped, '/bpe_test/input', 10)

        try:
            self.assertTrue(
                self._spin_until(
                    lambda: publisher.get_subscription_count() > 0
                    and self.node.count_publishers('/bpe_test/output') > 0,
                    5.0,
                ),
                'ROS boundary endpoints were not discovered',
            )

            message = TwistStamped()
            message.twist.linear.x = 1.5
            message.twist.linear.y = -2.0
            message.twist.linear.z = 3.0
            message.twist.angular.x = -1.0
            message.twist.angular.y = 2.0
            message.twist.angular.z = 0.5

            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and not self.received:
                publisher.publish(message)
                rclpy.spin_once(self.node, timeout_sec=0.05)

            self.assertTrue(self.received, 'No transformed TwistStamped was received')

            result = self.received[-1]
            self.assertEqual(result.twist.linear.x, 3.0)
            self.assertEqual(result.twist.linear.y, 2.0)
            self.assertEqual(result.twist.linear.z, 3.0)
            self.assertEqual(result.twist.angular.x, -1.0)
            self.assertEqual(result.twist.angular.y, 2.0)
            self.assertEqual(result.twist.angular.z, 0.75)
        finally:
            self.node.destroy_publisher(publisher)
            self.node.destroy_subscription(subscription)


@launch_testing.post_shutdown_test()
class TestTwistPipelineShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info, engine):
        launch_testing.asserts.assertExitCodes(proc_info, process=engine)
