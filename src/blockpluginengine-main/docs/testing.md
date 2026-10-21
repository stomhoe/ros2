# Block Plugin Engine testing

## 1. Test strategy

BPE tests are layered so failures can be localized without requiring every test to start ROS communication.

```text
bpe_blackboard
    ↓ concurrent primitives and blackboard semantics
block_plugin_engine
    ↓ manager, scheduler and typed BPE adapters
bpe_system_test
    ↓ real pluginlib-loaded internal typed pipeline
bpe_integration_test
    ↓ real lifecycle executable and ROS boundaries
```

The integration scenarios intentionally retain several incomplete configurations. They are valuable regression tests because they exercise configuration and destruction boundaries independently from message-flow correctness.

## 2. `bpe_blackboard`

This package tests the generic concurrent layer directly:

- `ConcurrentSlot` writer/reader ownership;
- `ConcurrentRing` registration, capacity and latest-value behavior;
- producer/consumer grants and RAII release;
- blackboard label registration;
- sequence and missed-update behavior;
- concurrent producer/consumer stress cases.

These tests are the primary verification of the steady-state blackboard concurrency contract.

Run with:

```bash
colcon test --packages-select bpe_blackboard --event-handlers console_direct+
```

## 3. `block_plugin_engine`

Core tests cover three main areas:

### 3.1 Plugin manager

The manager test verifies real `pluginlib` loading and lifecycle behavior, including:

- valid configuration and execution registration;
- duplicate block-name rejection;
- rejection of execution settings for non-`ITimerBlock` plugins;
- rollback after configuration failure;
- rollback of already started blocks after start failure.

### 3.2 Scheduler

Scheduler tests characterize:

- periodic phase maintenance;
- variable execution load;
- recovery after overrun;
- fairness under overload;
- runtime schedule immutability;
- invalid configuration rejection.

Timing tests should be run in an environment where host/container scheduling noise is understood; they test timing behavior rather than only functional return values.

### 3.3 Typed blackboard integration

Typed adapter tests verify that typed producers/consumers share the native representation correctly and can register in different configuration orders.

Run with:

```bash
colcon test --packages-select block_plugin_engine --event-handlers console_direct+
```

## 4. `bpe_system_test`

`bpe_system_test` provides test-only dynamically loaded plugins using `geometry_msgs::msg::TwistStamped`:

```text
TwistProducerBlock
    ↓ raw_twist
TwistTransformBlock
    ↓ processed_twist
TwistCheckerBlock
```

The pipeline exercises the public plugin API and typed blackboard without ROS topic boundaries. It verifies deterministic payload transformation plus basic sequence/missed-data behavior.

The package is useful as an intermediate layer: failures here implicate plugin loading, scheduler integration, or typed blackboard behavior before generic ROS I/O is involved.

Run with:

```bash
colcon test --packages-select bpe_system_test --event-handlers console_direct+
```

## 5. `bpe_integration_test`

This package starts the real `block_plugin_engine` executable with `launch_testing` and uses ordered YAML scenarios.

### 5.1 Scenario numbering

The numeric prefixes indicate increasing integration scope:

| Configuration | Purpose |
| --- | --- |
| `00_empty.yaml` | Lifecycle/configuration baseline with no blocks. |
| `10_internal_transform.yaml` | One dynamically loaded internal typed plugin. |
| `20_ros_input.yaml` | Generic ROS subscription only; regression for lifecycle-node / plugin-DSO lifetime. |
| `30_ros_output.yaml` | Generic ROS publisher only; differential boundary case. |
| `90_full_pipeline.yaml` | Complete ROS input → transform → ROS output pipeline. |

The gaps are deliberate. The earlier diagnostic matrix contained additional combinations, but only the scenarios with lasting regression value are retained.

### 5.2 Configure/shutdown regression tests

`test_configure_shutdown.py` is instantiated for:

```text
00_empty
10_internal_transform
20_ros_input
30_ros_output
```

Each case:

1. launches the real lifecycle executable;
2. requests `configure`;
3. waits until the node reaches `INACTIVE`;
4. lets `launch_testing` terminate the process;
5. requires clean exit code zero.

These pipelines are intentionally incomplete. For example, standalone `20_ros_input.yaml` has no typed endpoint providing serialization operations, so the test does not activate it. Its purpose is to verify that configuring and destroying a dynamically loaded generic subscription is safe.

### 5.3 Full end-to-end test

`test_twist_pipeline.py` uses `90_full_pipeline.yaml` and drives:

```text
configure → activate
```

The Python test node then:

1. waits for `ACTIVE`;
2. creates a typed `TwistStamped` publisher/subscriber;
3. waits for ROS endpoint discovery;
4. repeatedly publishes a deterministic sample until output arrives or timeout expires;
5. verifies the exact transformed values;
6. verifies clean process shutdown.

The path under test is:

```text
rclpy publisher
    ↓ ROS/DDS
GenericSubscription / RosInputBlock
    ↓ SerializedMessage
serialized → native conversion
    ↓ blackboard
TypedConsumer / TwistTransformBlock / TypedProducer
    ↓ conditional native → serialized conversion
RosOutputBlock / GenericPublisher
    ↓ ROS/DDS
rclpy subscriber
```

This test therefore covers the complete current architecture rather than only message transformation.

Run the integration package with:

```bash
colcon test --packages-select bpe_integration_test --event-handlers console_direct+
```

The full scenario can also be launched manually:

```bash
ros2 launch bpe_integration_test twist_pipeline.launch.py
```

## 6. Lifetime regression and archived diagnostic fixtures

A shutdown `SIGSEGV` discovered during integration testing was reduced systematically:

```text
empty engine                         → clean
internal typed plugin                → clean
ROS publisher plugin                 → clean
ROS subscription plugin              → crash on lifecycle-node destruction
```

Further diagnostic plugins separated blackboard binding from generic-subscription construction. Both member-callback and free-function callback subscription variants reproduced the fault, while binding alone did not.

The decisive experiment kept `BlockPluginManager` alive until after `BPE_LifecycleNode` destruction. The crash disappeared, establishing the lifetime contract now implemented by `BPECompositionRoot`.

The diagnostic source files remain under:

```text
bpe_ros_boundary_plugins/src/debug/
```

They are intentionally not compiled and not registered in `plugins.xml`. Their local README records their purpose. They should only be re-enabled for targeted lifetime diagnostics.

`20_ros_input.yaml` is the permanent automated regression for the failure mode that motivated this contract.

## 7. Recommended test sequence during development

For changes to generic concurrency or storage:

```bash
colcon test --packages-select bpe_blackboard block_plugin_engine \
  --event-handlers console_direct+
```

For changes to plugin API, scheduling integration, ROS type handling, boundaries, lifecycle, or ownership:

```bash
colcon test --packages-select \
  bpe_blackboard block_plugin_engine bpe_system_test bpe_integration_test \
  --event-handlers console_direct+
```

For a complete workspace validation:

```bash
colcon test --event-handlers console_direct+
```

## 8. Diagnostic principle

When a regression is not localized by existing tests, prefer differential reduction over repeated speculative fixes:

1. establish the smallest passing configuration;
2. add one capability at a time;
3. keep lifecycle and shutdown behavior constant;
4. promote high-value reduced cases into regression tests;
5. only change production architecture after the experiment identifies the violated contract.

The current ordered integration YAML set is intentionally the retained result of that method.
