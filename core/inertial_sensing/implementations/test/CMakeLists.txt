add_executable(inverted_pendulum_bot.inertial_sensing_test)
emil_build_for(inverted_pendulum_bot.inertial_sensing_test HOST All BOOL INVERTED_PENDULUM_BOT_BUILD_TESTS)
emil_add_test(inverted_pendulum_bot.inertial_sensing_test)

target_link_libraries(inverted_pendulum_bot.inertial_sensing_test PUBLIC
    gmock_main
    inverted_pendulum_bot.inertial_sensing.implementations
    inverted_pendulum_bot.platform_abstraction_mock
    infra.timer_test_helper
)

target_sources(inverted_pendulum_bot.inertial_sensing_test PRIVATE
    TestInertialSensingImpl.cpp
)
