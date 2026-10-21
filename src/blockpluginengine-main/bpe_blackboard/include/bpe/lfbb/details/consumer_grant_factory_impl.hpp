#pragma once

#include <stdexcept>

namespace bpe::lfbb {

template <typename ProtectedData, std::size_t InitializationMargin>
ConsumerGrantFactory<ProtectedData, InitializationMargin>::ConsumerGrantFactory(Ring* ring) noexcept
    : ring_(ring) {}

template <typename ProtectedData, std::size_t InitializationMargin>
ConsumerGrant<ProtectedData, InitializationMargin>
ConsumerGrantFactory<ProtectedData, InitializationMargin>::read_access() {
  if (active_) {
    throw std::runtime_error("Consumer already active");
  } else {
    active_ = true;
  }

  return ConsumerGrant<ProtectedData, InitializationMargin>(ring_, active_, last_seen_sequence_);
}

} // namespace bpe::lfbb
