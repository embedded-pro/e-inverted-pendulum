add_executable(inverted_pendulum_bot.persistence_test)
emil_build_for(inverted_pendulum_bot.persistence_test HOST All BOOL INVERTED_PENDULUM_BOT_BUILD_TESTS)
emil_add_test(inverted_pendulum_bot.persistence_test)

target_link_libraries(inverted_pendulum_bot.persistence_test PUBLIC
    gmock_main
    inverted_pendulum_bot.persistence
    services.util_test_doubles
    hal.interfaces_test_doubles
    infra.timer_test_helper
)

target_sources(inverted_pendulum_bot.persistence_test PRIVATE
    Mocks.hpp
    TestPersistentTuning.cpp
    TestPersistingBalanceControl.cpp
)
