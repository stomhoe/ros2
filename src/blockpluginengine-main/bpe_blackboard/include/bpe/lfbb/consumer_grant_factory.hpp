#pragma once

#include <cstddef>
#include <cstdint>

#include "bpe/lfbb/consumer_grant.hpp"

namespace bpe::lfbb {

template <typename ProtectedData, std::size_t InitializationMargin> class Blackboard;

namespace details {
template <typename ProtectedData, std::size_t InitializationMargin> class ConcurrentRing;
} // namespace details

template <typename ProtectedData, std::size_t InitializationMargin> class ConsumerGrantFactory {
public:
  ~ConsumerGrantFactory() = default;

  ConsumerGrantFactory(const ConsumerGrantFactory&) = delete;
  ConsumerGrantFactory& operator=(const ConsumerGrantFactory&) = delete;
  ConsumerGrantFactory(ConsumerGrantFactory&&) = delete;
  ConsumerGrantFactory& operator=(ConsumerGrantFactory&&) = delete;

  ConsumerGrant<ProtectedData, InitializationMargin> read_access();

private:
  using Ring = details::ConcurrentRing<ProtectedData, InitializationMargin>;

  explicit ConsumerGrantFactory(Ring* ring) noexcept;

  Ring* ring_{nullptr};
  std::uint64_t last_seen_sequence_{0};
  bool active_{false};

  friend class Blackboard<ProtectedData, InitializationMargin>;
};

} // namespace bpe::lfbb

#include "bpe/lfbb/details/consumer_grant_factory_impl.hpp"
