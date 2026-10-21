#pragma once

#include <yaml-cpp/yaml.h>

namespace bpe {

// Semantic name used by the BlockPluginEngine API.
// The current implementation uses YAML::Node, but Blocks depend on
// BlockConfig rather than on that implementation detail in their signatures.
using BlockConfig = YAML::Node;

} // namespace bpe
