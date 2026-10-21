# Archived ROS input diagnostic fixtures

These source files were created while diagnosing a shutdown/lifetime fault triggered after configuring `RosInputBlock`.

They split the input boundary into independent operations:

- `DebugRosInputBindBlock`: serialized blackboard/type-registry producer binding only;
- `DebugRosSubscriptionMemberBlock`: generic ROS subscription with a member callback, without blackboard binding;
- `DebugRosSubscriptionFunctionBlock`: the same generic ROS subscription with a callback independent of the plugin instance.

The experiments showed that producer binding alone shut down correctly, while either generic-subscription variant reproduced the fault. A later lifetime experiment showed that keeping `BlockPluginManager` and its `pluginlib::ClassLoader` alive until after destruction of `BPE_LifecycleNode` removed the fault.

The permanent fix is the explicit ownership/destruction contract implemented by `BPECompositionRoot`:

```text
BPE_LifecycleNode
    ↓
BlockPluginManager / ClassLoader
    ↓
Scheduler
```

These files are intentionally **not compiled and not registered in `plugins.xml`**. They are retained only as diagnostic fixtures in case this lifetime issue must be investigated again. Do not add them back to the production plugin library unless they are explicitly needed for diagnostics.

The full rationale and retained regression test are documented in:

- `docs/architecture.md` — physical lifetime contract;
- `docs/testing.md` — lifetime regression and diagnostic reduction.
