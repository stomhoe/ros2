#pragma once

#include <string>
#include <vector>

namespace bpe::test {

void clear_events();
void record_event(const std::string& event);
std::vector<std::string> events();

} // namespace bpe::test