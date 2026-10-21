#pragma once

#include <cstddef>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "bpe/lfbb/consumer_grant_factory.hpp"
#include "bpe/lfbb/details/concurrent_ring.hpp"
#include "bpe/lfbb/producer_grant_factory.hpp"

namespace bpe::lfbb {

template <typename ProtectedData, std::size_t InitializationMargin = 2u> class Blackboard {
public:
  Blackboard() = default;
  ~Blackboard() = default;

  Blackboard(const Blackboard&) = delete;
  Blackboard& operator=(const Blackboard&) = delete;
  Blackboard(Blackboard&&) = delete;
  Blackboard& operator=(Blackboard&&) = delete;

  ProducerGrantFactory<ProtectedData, InitializationMargin>
  register_producer(const std::string& label);

  template <typename Initializer>
  ProducerGrantFactory<ProtectedData, InitializationMargin>
  register_producer(const std::string& label, const Initializer& initializer);

  ConsumerGrantFactory<ProtectedData, InitializationMargin>
  register_consumer(const std::string& label);

  template <typename Initializer>
  ConsumerGrantFactory<ProtectedData, InitializationMargin>
  register_consumer(const std::string& label, const Initializer& initializer);

  bool all_have_producer() const;
  std::vector<std::string> labels_without_producer() const;

private:
  using Ring = details::ConcurrentRing<ProtectedData, InitializationMargin>;

  struct Entry {
    Ring ring;
  };

  mutable std::mutex mutex_;
  std::map<std::string, Entry> entries_;
};

} // namespace bpe::lfbb

#include "bpe/lfbb/details/blackboard_impl.hpp"
