add_executable(inverted_pendulum_bot.safety_supervisor_test)
emil_build_for(inverted_pendulum_bot.safety_supervisor_test HOST All BOOL INVERTED_PENDULUM_BOT_BUILD_TESTS)
emil_add_test(inverted_pendulum_bot.safety_supervisor_test)

target_link_libraries(inverted_pendulum_bot.safety_supervisor_test PUBLIC
    gmock_main
    inverted_pendulum_bot.safety_supervisor.implementations
    infra.timer_test_helper
)

target_sources(inverted_pendulum_bot.safety_supervisor_test PRIVATE
    TestSafetySupervisorImpl.cpp
)
