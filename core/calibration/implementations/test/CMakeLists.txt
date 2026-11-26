add_executable(inverted_pendulum_bot.calibration_test)
emil_build_for(inverted_pendulum_bot.calibration_test HOST All BOOL INVERTED_PENDULUM_BOT_BUILD_TESTS)
emil_add_test(inverted_pendulum_bot.calibration_test)

target_link_libraries(inverted_pendulum_bot.calibration_test PUBLIC
    gmock_main
    inverted_pendulum_bot.calibration.implementations
    inverted_pendulum_bot.balance_control_mock
    hal.interfaces_test_doubles
    infra.timer_test_helper
)

target_sources(inverted_pendulum_bot.calibration_test PRIVATE
    TestCalibrationImpl.cpp
)
