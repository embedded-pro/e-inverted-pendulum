add_executable(inverted_pendulum_bot.telemetry_test)
emil_build_for(inverted_pendulum_bot.telemetry_test HOST All BOOL INVERTED_PENDULUM_BOT_BUILD_TESTS)
emil_add_test(inverted_pendulum_bot.telemetry_test)

target_link_libraries(inverted_pendulum_bot.telemetry_test PUBLIC
    gmock_main
    inverted_pendulum_bot.telemetry.implementations
)

target_sources(inverted_pendulum_bot.telemetry_test PRIVATE
    TestTelemetryRecorder.cpp
)
