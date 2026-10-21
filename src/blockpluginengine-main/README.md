# Block Plugin Engine

Block Plugin Engine (BPE) is a ROS 2 execution framework for dynamically loaded processing blocks.

BPE provides a single ROS 2 lifecycle node, a plugin manager, a periodic scheduler, and a concurrent blackboard used to exchange data between blocks. ROS 2 is primarily treated as an input/output boundary: processing blocks can remain independent from ROS unless they explicitly implement a ROS boundary.

The system is designed around a fixed configuration established before execution and an execution path with predictable concurrency and memory behaviour.

## Key features and guarantees

BPE currently provides the following architectural properties:

- A single `rclcpp_lifecycle::LifecycleNode` acts as the ROS 2 integration point.
- Processing components are dynamically loaded through `pluginlib`.
- Multiple instances of the same plugin type may be configured.
- The block topology is fixed after configuration; plugins are not added or removed while the system is running.
- Data exchange between blocks uses the `bpe_blackboard` concurrent blackboard.
- Blackboard data access during execution does not use mutex-based blocking.
- Under its defined single-producer/multiple-consumer topology and slot-capacity contract, blackboard producer and consumer access provides wait-free progress.
- Consumers read the latest committed value and do not interfere with each other.
- BPE has two main execution domains:
  - the main ROS executor thread;
  - the dedicated scheduler worker thread.
- The scheduler is designed for periodic real-time-oriented execution:
  - fixed cyclic schedule;
  - absolute deadlines using `clock_nanosleep(..., TIMER_ABSTIME)`;
  - configurable wake-early interval;
  - final busy-wait until the deadline;
  - no allocations in the cyclic execution path after startup.
- ROS input and output are implemented as generic boundaries and do not require compile-time knowledge of the ROS message type.
- Internal plugins use normal typed C++ messages.
- Serialization is performed only when a ROS output requires it.
- The runtime composition and destruction order are explicit design contracts rather than incidental C++ member lifetime.

These properties are described in detail in [`docs/architecture.md`](docs/architecture.md).

## Main architecture

The main runtime components are:

```text
BPECompositionRoot
├── BPE_LifecycleNode
├── BlockPluginManager
└── Scheduler
```

`BPECompositionRoot` is the composition root and owner of the runtime components. It constructs the system, connects its dependencies, and enforces the required destruction order.

`BPE_LifecycleNode` maps the ROS 2 lifecycle to BPE operations.

`BlockPluginManager` loads, configures, starts, stops, and destroys block plugins.

The scheduler executes blocks implementing `ITimerBlock`.

The blackboard provides concurrent communication between producers and consumers.

### Lifetime contract

The destruction order is deliberately explicit:

```text
BPE_LifecycleNode
        ↓
BlockPluginManager / pluginlib ClassLoader
        ↓
Scheduler
```

This order is part of the correctness contract.

Plugins may create ROS entities such as generic subscriptions. ROS can retain internal references associated with those entities until the lifecycle node itself is destroyed. Therefore plugin shared libraries must remain loaded until destruction of the `BPE_LifecycleNode` has completed.

Changing this order can introduce invalid references to code from already unloaded plugin libraries.

## Block interfaces

Every block implements:

```cpp
bpe::IBlockPlugin
```

which provides the block lifecycle:

```text
configure
start
stop
cleanup
```

Periodic blocks additionally implement:

```cpp
bpe::ITimerBlock
```

ROS boundary blocks implement:

```cpp
bpe::IRosBoundaryBlock
```

which allows BPE to inject the ROS `NodeTopicsInterface`.

The current boundary convention is:

```text
IRosBoundaryBlock + !ITimerBlock
    → ROS input

IRosBoundaryBlock + ITimerBlock
    → ROS output
```

An input is callback-driven by ROS.

An output is driven by the BPE scheduler.

## Blackboard access

Internal plugins normally use the typed BPE API:

```cpp
bpe::TypedConsumer<Message> consumer_;
bpe::TypedProducer<Message> producer_;
```

During configuration:

```cpp
consumer_.bind("input_label");
producer_.bind("output_label");
```

During execution:

```cpp
auto input = consumer_.read_access();

if (input.has_new_value()) {
  auto output = producer_.write_access();
  output.value() = input.value();
}
```

The typed API hides the internal `RosProtectedData`, grant factories, serialized representation, and type-erasure mechanisms.

## ROS boundaries

The package `bpe_ros_boundary_plugins` provides two generic ROS plugins:

```text
bpe_ros_boundary_plugins/RosInputBlock
bpe_ros_boundary_plugins/RosOutputBlock
```

`RosInputBlock` receives a serialized ROS message, stores the serialized representation in the blackboard, and deserializes the corresponding native representation for typed consumers.

`RosOutputBlock` reads the serialized representation from the blackboard and publishes it through a generic ROS publisher.

The associated `RosTypeRegistry` validates type compatibility and provides the type-erased serialization/deserialization operations required by the boundary.

## Configuration

BPE is configured using YAML.

A typical pipeline is:

```yaml
scheduler:
  cycle: 10ms
  wake_early: 0.5ms

blocks:
  - name: ros_input
    type: bpe_ros_boundary_plugins/RosInputBlock
    config:
      topic: /bpe_test/input
      ros_type: geometry_msgs/msg/TwistStamped
      blackboard_output: raw_twist
      qos_depth: 10

  - name: transform
    type: bpe_system_test/TwistTransformBlock
    execution:
      period: 10ms
    config:
      blackboard_input: raw_twist
      blackboard_output: processed_twist

  - name: ros_output
    type: bpe_ros_boundary_plugins/RosOutputBlock
    execution:
      period: 10ms
    config:
      topic: /bpe_test/output
      ros_type: geometry_msgs/msg/TwistStamped
      blackboard_input: processed_twist
      qos_depth: 10
```

The complete configuration format and constraints are documented in [`docs/configuration.md`](docs/configuration.md).

## Lifecycle

BPE uses the normal ROS 2 lifecycle:

```text
UNCONFIGURED
     ↓ configure
INACTIVE
     ↓ activate
ACTIVE
     ↓ deactivate
INACTIVE
     ↓ cleanup
UNCONFIGURED
```

Shutdown may transition the node to `FINALIZED`.

Configuration establishes the plugin topology, blackboard registrations, type information, ROS boundaries, and scheduler configuration.

Activation starts the plugins and the scheduler.

Stopping and cleanup are performed in explicit order so that the scheduler cannot continue executing block instances while they are being destroyed.

## Running BPE

Run the node with a configuration file:

```bash
ros2 run block_plugin_engine block_plugin_engine --ros-args \
  -p config_file:=/path/to/config.yaml
```

Lifecycle transitions can then be controlled normally:

```bash
ros2 lifecycle set /block_plugin_engine configure
ros2 lifecycle set /block_plugin_engine activate
ros2 lifecycle set /block_plugin_engine deactivate
ros2 lifecycle set /block_plugin_engine cleanup
```

For applications where the node should immediately configure and activate itself:

```bash
ros2 run block_plugin_engine block_plugin_engine --ros-args \
  -p config_file:=/path/to/config.yaml \
  -p autostart:=true
```

`autostart` is disabled by default.

## Process shutdown

The executable handles `SIGINT` and `SIGTERM` explicitly instead of delegating process shutdown entirely to the default `rclcpp` signal handler.

The signal handler itself only sets a signal-safe flag.

The main executor loop detects the request and performs the orderly lifecycle shutdown while the ROS context is still valid. Runtime objects are then destroyed in the order enforced by `BPECompositionRoot`, and `rclcpp::shutdown()` is called afterwards.

This explicit sequence is intentional and forms part of the runtime lifecycle design.

## Packages

The current repository contains the following main packages:

### `bpe_blackboard`

Header-only concurrent blackboard implementation.

It provides the generic concurrent ring, producer/consumer grants, factories, and the public blackboard facade.

### `block_plugin_engine`

Core BPE runtime.

Contains:

- `BPECompositionRoot`;
- `BPE_LifecycleNode`;
- `BlockPluginManager`;
- scheduler interfaces and implementation;
- typed blackboard adapters;
- `RosProtectedData`;
- `RosTypeRegistry`;
- the BPE executable.

### `bpe_ros_boundary_plugins`

Generic ROS input and output plugins.

### `bpe_system_test`

Auxiliary plugins used for system-level tests, including the deterministic `TwistStamped` transformation pipeline.

### `bpe_integration_test`

End-to-end integration tests using a real BPE node, lifecycle transitions, dynamically loaded plugins, ROS publishers/subscribers, serialization, blackboard communication, and clean process shutdown.

## Testing

The project uses several test levels:

```text
bpe_blackboard
    concurrent data structure behaviour

block_plugin_engine
    plugin manager
    scheduler
    typed blackboard integration

bpe_system_test
    block-level processing pipeline

bpe_integration_test
    complete ROS 2 end-to-end system
```

The integration package contains progressively more complete configurations:

```text
00_empty.yaml
10_internal_transform.yaml
20_ros_input.yaml
30_ros_output.yaml
90_full_pipeline.yaml
```

These configurations are intentionally retained as regression scenarios.

In particular, the ROS-input-only case exercises an important lifetime contract between dynamically loaded subscription plugins and the lifecycle node.

See [`docs/testing.md`](docs/testing.md) for the testing strategy and commands.

## Further documentation

- [`docs/architecture.md`](docs/architecture.md) — architecture, guarantees, concurrency model, lifecycle, ownership, and design contracts.
- [`docs/configuration.md`](docs/configuration.md) — YAML configuration format and plugin configuration.
- [`docs/testing.md`](docs/testing.md) — test architecture, integration scenarios, and regression strategy.