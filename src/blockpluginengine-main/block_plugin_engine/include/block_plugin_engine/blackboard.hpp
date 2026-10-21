#pragma once

#include "bpe/lfbb/blackboard.hpp"
#include "block_plugin_engine/ros_protected_data.hpp"

namespace bpe {

using BpeBlackboard = lfbb::Blackboard<RosProtectedData>;

// Process-wide BPE blackboard shared by the engine and all loaded plugins.
BpeBlackboard& the_blackboard();

} // namespace bpe
