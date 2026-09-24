add_executable(inverted_pendulum_bot.balance_control_test)
emil_build_for(inverted_pendulum_bot.balance_control_test HOST All BOOL INVERTED_PENDULUM_BOT_BUILD_TESTS)
emil_add_test(inverted_pendulum_bot.balance_control_test)

target_link_libraries(inverted_pendulum_bot.balance_control_test PUBLIC
    gmock_main
    inverted_pendulum_bot.balance_control.implementations
    infra.timer_test_helper
)

target_sources(inverted_pendulum_bot.balance_control_test PRIVATE
    TestBalanceControlImpl.cpp
    TestCascadedPidStrategy.cpp
    TestContinuousPid.cpp
    TestLqrStrategy.cpp
)
