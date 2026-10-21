# Block Plugin Engine architecture

## 1. Goals and scope

Block Plugin Engine (BPE) executes dynamically loaded C++ processing blocks inside one ROS 2 process. The architecture deliberately separates three concerns:

1. **ROS 2 lifecycle and external I/O**;
2. **periodic block execution**;
3. **concurrent in-process data exchange**.

ROS 2 is intended primarily as an I/O boundary. Internal processing blocks can remain independent of publishers, subscriptions, executors, and concrete ROS node classes.

The runtime topology is assembled during lifecycle configuration and remains fixed while active.

## 2. System guarantees and design contracts

The table below distinguishes externally useful properties from the internal contracts that preserve them.

| Guarantee / feature | Required contract | Current mechanism |
| --- | --- | --- |
| Steady-state blackboard access uses no mutexes or blocking waits | One producer per label; fixed registered consumer set during execution; sufficient ring capacity | Atomic slot state, CAS acquisition, latest-committed index, structural slot margin |
| Wait-free progress for the configured producer/consumer topology | A writer-accessible slot must always exist; readers share the latest committed slot | Capacity based on consumer count plus a two-slot wait-free margin, rounded to a power of two |
| Consumers observe latest-value semantics | Readers do not reserve historical slots | Every read starts from the latest committed index |
| Runtime topology is immutable | Registration/loading happens before execution | `pluginlib` loading and scheduler registration occur in `configure`; scheduler add/remove/clear reject the running state |
| Periodic execution preserves nominal phase | Deadlines advance from previous deadlines, not from completion time | `next_execution += period` |
| Scheduler wake-up is suitable for low-jitter cyclic work | Absolute clock, optional real-time policy, bounded final spin window | `CLOCK_MONOTONIC`, `TIMER_ABSTIME`, optional `SCHED_FIFO`, wake-early + busy-wait |
| ROS callbacks and cyclic execution are separated | Subscription callbacks must not execute in the cyclic scheduler | Main-thread `SingleThreadedExecutor` plus dedicated scheduler worker |
| Internal blocks need not depend on ROS node types | ROS node access is injected only when requested | `IRosBoundaryBlock::set_ros_interface(NodeTopicsInterface::SharedPtr)` |
| Type-erased ROS boundaries remain type-safe at configuration time | Every blackboard label has one consistent ROS type identity | `RosTypeRegistry` checks `package/msg/Type` identity and provides typed serialization operations |
| Internal-only pipelines do not pay serialization cost | Serialization is required only if a ROS output exists for a label | `RosTypeInfo::requires_serialization()` checked by `TypedProducerAdapter` |
| Failed representation conversion must not publish a partial sample | Producer commit must be cancellable | `ProducerGrant::cancel()` releases the claimed slot without advancing sequence or last-committed index |
| Dynamic plugin code remains loaded while ROS destroys entities created by those plugins | Plugin manager/class loader must outlive the lifecycle node | Explicit `BPECompositionRoot` destruction order: node → plugin manager → scheduler |

### 2.1 Scope of the blackboard real-time guarantee

The blackboard has two phases with different requirements:

- **configuration/registration** may use mutexes and dynamic allocation;
- **steady-state producer/consumer data access** uses the concurrent ring and does not use mutexes or blocking waits.

The wait-free contract applies to the fixed runtime topology. The generic `Blackboard` facade itself uses a mutex when creating labels and registering endpoints; this is intentionally outside the execution path.

### 2.2 Scope of the scheduler real-time guarantee

`BPE_SimpleScheduler` is **real-time-oriented**, not a declaration that the complete process is hard real-time. Its worker loop avoids dynamic scheduler reconfiguration and container growth after start, but the code executed by a plugin's `execute()` remains plugin responsibility. ROS middleware and the operating system also remain outside BPE's deterministic control.

## 3. Top-level composition

One BPE process is assembled by `BPECompositionRoot`:

```text
BPECompositionRoot
├── BPE_SimpleScheduler       (IBlockScheduler)
├── BPE_LifecycleNode         (rclcpp_lifecycle::LifecycleNode)
└── BlockPluginManager        (pluginlib::ClassLoader<IBlockPlugin>)
```

Construction is explicit:

```text
create scheduler
    ↓
create lifecycle node(scheduler)
    ↓
create plugin manager(scheduler, NodeTopicsInterface, logger)
    ↓
inject plugin manager into lifecycle node
```

The root owns all three objects with `shared_ptr` because references are shared across the trio, but ownership direction remains acyclic:

- the lifecycle node holds scheduler and plugin-manager references;
- the plugin manager holds scheduler and ROS `NodeTopicsInterface` references;
- scheduler and manager do not own the lifecycle node;
- `BPECompositionRoot` is the ultimate owner.

## 4. Physical lifetime contract

Destruction order is deliberately coded in `BPECompositionRoot::~BPECompositionRoot()`:

```text
BPE_LifecycleNode
    ↓
BlockPluginManager / pluginlib::ClassLoader
    ↓
BPE_SimpleScheduler
```

This order is a correctness requirement.

During integration testing, a shutdown fault was reduced to a generic ROS subscription created by a dynamically loaded plugin. Destroying `BlockPluginManager` before `BPE_LifecycleNode` allowed the plugin DSO to be unloaded while ROS node teardown still retained subscription-related state. Keeping the manager/class loader alive through lifecycle-node destruction removed the fault.

Therefore:

> **Do not replace the explicit composition-root destruction order with incidental member destruction or move the class loader into an object that can die before the lifecycle node.**

The archived diagnostic fixtures that isolated this requirement remain in `bpe_ros_boundary_plugins/src/debug/` but are not compiled or registered.

## 5. Lifecycle model

`BPE_LifecycleNode` translates ROS lifecycle transitions into BPE operations.

### 5.1 Configure

```text
load YAML file
    ↓
configure scheduler
    ↓
BlockPluginManager validates block list
    ↓
load each plugin with pluginlib
    ↓
inject ROS interface into IRosBoundaryBlock
    ↓
plugin.configure(config subtree)
    ↓
register ITimerBlock entries with scheduler
```

Configuration failures roll back already configured plugin instances. Shared blackboard/type-registry state is reset when engine-level plugin configuration fails.

### 5.2 Activate

```text
start plugins in configuration order
    ↓
start scheduler
```

If a plugin start fails, previously started plugins are stopped. If scheduler start fails, plugins are stopped before returning failure.

### 5.3 Deactivate

```text
stop scheduler and join worker
    ↓
stop plugins in reverse order
```

The scheduler stops first so no `ITimerBlock::execute()` call can race with plugin stop.

### 5.4 Cleanup

```text
ensure scheduler stopped
    ↓
clear scheduler block references
    ↓
cleanup plugins in reverse order
    ↓
destroy plugin instances
    ↓
reset process-wide blackboard
    ↓
reset ROS type registry
```

After cleanup the engine can be configured again from an empty runtime state.

### 5.5 Shutdown

Shutdown performs the same operational safety sequence needed to terminate from any configured/active state: stop scheduler, stop plugins, clean up plugin instances, then reset shared runtime state.

Operational lifecycle teardown and physical C++ object lifetime are intentionally separate concepts. Lifecycle callbacks make the system inactive/clean; `BPECompositionRoot` then guarantees safe object destruction order.

## 6. Process shutdown model

The executable does not install rclcpp's default signal handler. It initializes ROS with `SignalHandlerOptions::None` and installs minimal SIGINT/SIGTERM handlers that only set a `sig_atomic_t` flag.

The main thread then follows an explicit sequence:

```text
SingleThreadedExecutor::spin_once() loop
    ↓
signal flag observed
    ↓
remove lifecycle node from executor
    ↓
request lifecycle shutdown while ROS context is still valid
    ↓
destroy executor
    ↓
leave BPECompositionRoot scope
    ↓
root destroys node → manager → scheduler
    ↓
rclcpp::shutdown()
```

This makes process-level shutdown ordering visible and testable.

## 7. Execution and threading model

BPE itself manages two execution domains when periodic blocks are present:

```text
Main thread
  └── rclcpp::SingleThreadedExecutor
      └── ROS callbacks, including RosInputBlock subscriptions

Scheduler worker thread
  └── BPE_SimpleScheduler
      └── ITimerBlock::execute()
```

A configuration with no scheduled blocks does not create the scheduler worker thread.

ROS/DDS may use additional internal threads; those are middleware implementation details and are not BPE scheduling domains.

This split has an important semantic consequence:

- a ROS input boundary is callback-driven and does **not** implement `ITimerBlock`;
- a ROS output boundary is scheduler-driven and **does** implement `ITimerBlock`.

The same pattern can be applied to future non-ROS boundaries: reception may be externally driven, while deterministic emission can be scheduled.

## 8. Scheduler design

`BPE_SimpleScheduler` is a single-worker cyclic scheduler.

### 8.1 Timing mechanism

For the earliest pending deadline it:

1. computes `deadline - spin_before_deadline`;
2. sleeps using `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)`;
3. busy-waits until the absolute deadline;
4. executes each block whose deadline has been reached;
5. advances that block's deadline with `next_execution += period`.

Using absolute deadlines and incrementing from the previous deadline avoids systematic phase drift caused by execution time.

### 8.2 Real-time policy

The worker can request `SCHED_FIFO` with configurable priority. Real-time setup may be optional or required:

- `enabled: false`: do not request `SCHED_FIFO`;
- `enabled: true, required: false`: attempt real-time scheduling but continue if the OS rejects it;
- `enabled: true, required: true`: scheduler start fails if `SCHED_FIFO` cannot be established.

### 8.3 Runtime immutability

`add`, `remove`, and `clear` are rejected while running. The scheduled block vector is constructed before start. `stop()` is idempotent and joins the worker thread before returning.

The scheduler worker contains no logging and does not grow its containers in the cyclic loop. This does not constrain allocations performed inside plugin `execute()` implementations.

## 9. Block model

### 9.1 `IBlockPlugin`

Every plugin implements:

```cpp
configure(const BlockConfig&)
start()
stop()
cleanup()
```

`BlockConfig` is currently a semantic alias for `YAML::Node`.

### 9.2 `ITimerBlock`

A plugin implementing `ITimerBlock` may have an `execution:` section in YAML and is registered with the scheduler. A block that declares execution settings without implementing `ITimerBlock` is rejected during configuration.

### 9.3 `IRosBoundaryBlock`

A plugin implementing `IRosBoundaryBlock` receives a shared `NodeTopicsInterface` from the plugin manager before `configure()` is called. The interface is passed by const reference to a `shared_ptr`, expressing shared dependency rather than transfer of ownership.

## 10. Blackboard architecture

### 10.1 Generic layer

`bpe_blackboard` is a header-only C++20 library in namespace `bpe::lfbb`. Its core layers are:

```text
Blackboard<ProtectedData>
    ↓ labels / registration
ConcurrentRing<ProtectedData>
    ↓
ConcurrentSlot<ProtectedData>

ProducerGrantFactory → ProducerGrant
ConsumerGrantFactory → ConsumerGrant
```

`ConcurrentSlot` separates concurrency state from the protected payload. Slot state represents free, writer-owned, or reader-count ownership using atomics and CAS.

### 10.2 Runtime ring contract

Each label supports:

- at most one registered producer;
- multiple registered consumers;
- one active grant per endpoint factory;
- latest-committed read semantics;
- monotonically increasing commit sequence;
- timestamp captured at commit;
- consumer status: `no_data_yet`, `no_update`, `ok`, or `missed`.

A producer grant normally commits on destruction. `cancel()` releases the write claim without incrementing sequence, changing the latest committed index, or publishing the modified slot.

### 10.3 Capacity rule

The ring reserves a structural margin of two slots beyond the consumer count:

```text
base_capacity = max(4, consumer_count + 2)
capacity = next_power_of_two(base_capacity + uninitialized_registration_margin)
```

The two structural slots are independent of the initialization margin:

- one accounts for the latest committed/readable position;
- one guarantees writer progress under the fixed consumer topology.

The public `Blackboard` default initialization margin is `2`. It allows a limited number of registrations without a new initializer after storage has been initialized.

### 10.4 BPE process-wide instance

BPE specializes the generic blackboard as:

```cpp
using BpeBlackboard = lfbb::Blackboard<RosProtectedData>;
BpeBlackboard& the_blackboard();
```

The instance lives in the shared `block_plugin_engine_core` library rather than in a header. This guarantees that the engine executable and every plugin DSO observe the same process-wide blackboard.

## 11. Native and serialized representations

BPE stores two representations of one logical sample in each slot:

```cpp
struct RosProtectedData {
  std::shared_ptr<void> native;
  rclcpp::SerializedMessage serialized;
};
```

Plugins should not manipulate these members directly. They use typed or serialized endpoint adapters.

### 11.1 Typed internal endpoints

`TypedProducer<T>` and `TypedConsumer<T>` bind a blackboard label during plugin configuration. Binding:

- registers the endpoint in the blackboard;
- initializes native slot storage when required;
- registers the ROS type name from `rosidl_generator_traits::name<T>()`;
- installs type-erased serialize/deserialize functions in `RosTypeRegistry`.

Runtime grants expose `T&` or `const T&` while retaining the underlying slot reservation through RAII.

### 11.2 ROS type registry

`RosTypeRegistry` is configuration-time mutable and runtime stable. For each label it records:

- canonical ROS type string;
- type-erased serialize function;
- type-erased deserialize function;
- whether one ROS output has been registered.

A different ROS type for an existing label is rejected. A second ROS output for the same label is also rejected.

Registration order is intentionally flexible: a ROS boundary may establish label/type identity before a typed plugin supplies conversion functions, or vice versa.

### 11.3 Conditional serialization

A typed producer always writes the native representation. Before commit:

```text
if ROS output registered for label:
    serialize native → serialized
else:
    no serialization
```

If serialization throws, the producer grant is cancelled and no new sample becomes visible.

### 11.4 ROS input

`RosInputBlock` receives a serialized sample on the main executor thread. Its serialized producer writes the slot's `SerializedMessage` and deserializes into the native object before commit. Deserialization failure cancels the grant.

The input boundary can be configured before a typed endpoint has supplied conversion functions, but `start()` requires those functions to exist. This permits configuration-order independence while preventing an active input from committing data that cannot be represented natively.

### 11.5 ROS output

`RosOutputBlock` is an `ITimerBlock`. On each scheduled execution it reads the latest serialized representation and publishes only when the consumer grant reports new data.

## 12. Configuration-time versus runtime invariants

The following operations belong to configuration and are allowed to lock or allocate:

- YAML parsing;
- plugin loading;
- endpoint registration;
- blackboard growth and slot initialization;
- ROS publisher/subscription construction;
- scheduler-vector construction;
- ROS type-registry completion.

After activation, the structural model is fixed. Runtime code should treat plugin instances, labels, endpoint factories, scheduler entries, and type-registry entries as immutable infrastructure.

## 13. Public versus internal API

Installed public headers under `include/block_plugin_engine/` contain the interfaces required by plugin packages:

- `IBlockPlugin`;
- `ITimerBlock`;
- `IRosBoundaryBlock`;
- `IBlockScheduler` and scheduler configuration types;
- `BlockConfig`;
- `TypedProducer<T>` / `TypedConsumer<T>` and adapters;
- `RosProtectedData`, `BpeBlackboard`, and `RosTypeRegistry` support.

Engine composition classes live under `src/detail/` and are not installed:

- `BPECompositionRoot`;
- `BPE_LifecycleNode`;
- `BlockPluginManager`;
- `BPE_SimpleScheduler` implementation details.

The separation is intentional: plugin authors depend on behavioral interfaces, not on the engine's composition root or node implementation.
