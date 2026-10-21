ament_add_gtest(test_concurrent_slot
  test_concurrent_slot.cpp
)
target_link_libraries(test_concurrent_slot
  bpe_blackboard
  Threads::Threads
)

ament_add_gtest(test_concurrent_ring
  test_concurrent_ring.cpp
)
target_link_libraries(test_concurrent_ring
  bpe_blackboard
  Threads::Threads
)

ament_add_gtest(test_grants
  test_grants.cpp
)
target_link_libraries(test_grants
  bpe_blackboard
  Threads::Threads
)

ament_add_gtest(test_blackboard
  test_blackboard.cpp
)
target_link_libraries(test_blackboard
  bpe_blackboard
  Threads::Threads
)
