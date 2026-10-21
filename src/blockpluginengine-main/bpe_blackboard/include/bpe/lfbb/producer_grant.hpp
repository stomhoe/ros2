#pragma once

#include <cstddef>

namespace bpe::lfbb {

template <typename ProtectedData, std::size_t InitializationMargin> class ProducerGrantFactory;

namespace details {
template <typename ProtectedData, std::size_t InitializationMargin> class ConcurrentRing;
template <typename ProtectedData> class ConcurrentSlot;
} // namespace details

template <typename ProtectedData, std::size_t InitializationMargin> class ProducerGrant {
public:
  ~ProducerGrant();

  ProtectedData& data() noexcept;
  void cancel() noexcept;

  ProducerGrant(const ProducerGrant&) = delete;
  ProducerGrant& operator=(const ProducerGrant&) = delete;
  ProducerGrant(ProducerGrant&&) = delete;
  ProducerGrant& operator=(ProducerGrant&&) = delete;

private:
  using Ring = details::ConcurrentRing<ProtectedData, InitializationMargin>;
  using Slot = details::ConcurrentSlot<ProtectedData>;

  ProducerGrant(Ring* ring, bool& active) noexcept;

  Ring* ring_{nullptr};
  Slot* slot_{nullptr};
  bool& active_;
  bool cancelled_{false};

  friend class ProducerGrantFactory<ProtectedData, InitializationMargin>;
};

} // namespace bpe::lfbb

#include "bpe/lfbb/details/producer_grant_impl.hpp"
