#pragma once

#include "bpe/lfbb/details/concurrent_ring.hpp"

namespace bpe::lfbb {

template <typename ProtectedData, std::size_t InitializationMargin>
ProducerGrant<ProtectedData, InitializationMargin>::ProducerGrant(Ring* ring,
                                                                   bool& active) noexcept
    : ring_(ring), slot_(ring_->claim_next_write_slot()), active_(active) {}

template <typename ProtectedData, std::size_t InitializationMargin>
ProducerGrant<ProtectedData, InitializationMargin>::~ProducerGrant() {
  if (cancelled_) {
    ring_->cancel_written_slot();
  } else {
    ring_->commit_written_slot();
  }

  active_ = false;
}

template <typename ProtectedData, std::size_t InitializationMargin>
ProtectedData& ProducerGrant<ProtectedData, InitializationMargin>::data() noexcept {
  return slot_->data();
}

template <typename ProtectedData, std::size_t InitializationMargin>
void ProducerGrant<ProtectedData, InitializationMargin>::cancel() noexcept {
  cancelled_ = true;
}

} // namespace bpe::lfbb
