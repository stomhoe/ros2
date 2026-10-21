#include "test_event_log.hpp"

#include <iostream>
#include <mutex>
#include <vector>

namespace bpe::test {
namespace {

std::mutex event_mutex;
std::vector<std::string> event_log;

} // namespace

void clear_events() {
  std::lock_guard<std::mutex> lock(event_mutex);
  event_log.clear();
}

void record_event(const std::string& event) {
  {
    std::lock_guard<std::mutex> lock(event_mutex);
    event_log.push_back(event);
  }

  std::cout << "[TEST BLOCK] " << event << std::endl;
}

std::vector<std::string> events() {
  std::lock_guard<std::mutex> lock(event_mutex);
  return event_log;
}

} // namespace bpe::test