#pragma once

#include <chrono>

namespace bpe {

enum class ExecutionPolicy { TIMER };

struct ExecutionConfig {
  ExecutionPolicy policy{ExecutionPolicy::TIMER};
  std::chrono::nanoseconds period{0};
};

} // namespace bpe
