add_executable(inverted_pendulum_bot.cli_test)
emil_build_for(inverted_pendulum_bot.cli_test HOST All BOOL INVERTED_PENDULUM_BOT_BUILD_TESTS)
emil_add_test(inverted_pendulum_bot.cli_test)

target_link_libraries(inverted_pendulum_bot.cli_test PUBLIC
    gmock_main
    inverted_pendulum_bot.cli
    inverted_pendulum_bot.platform_abstraction_mock
    inverted_pendulum_bot.inertial_sensing.interfaces
    hal.interfaces_test_doubles
    infra.event_test_helper
    infra.timer_test_helper
)

target_sources(inverted_pendulum_bot.cli_test PRIVATE
    TestCli.cpp
    TestNumberParser.cpp
)
