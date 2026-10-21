#pragma once

namespace bpe {

enum class ExecutionResult { SUCCESS, FAILURE };

class ITimerBlock {
public:
  virtual ~ITimerBlock() = default;

  virtual ExecutionResult execute() = 0;
};

} // namespace bpe
