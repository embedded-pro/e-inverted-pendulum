add_executable(inverted_pendulum_bot.motion_actuation_test)
emil_build_for(inverted_pendulum_bot.motion_actuation_test HOST All BOOL INVERTED_PENDULUM_BOT_BUILD_TESTS)
emil_add_test(inverted_pendulum_bot.motion_actuation_test)

target_link_libraries(inverted_pendulum_bot.motion_actuation_test PUBLIC
    gmock_main
    inverted_pendulum_bot.motion_actuation.implementations
    inverted_pendulum_bot.platform_abstraction_mock
    hal.interfaces_test_doubles
    infra.timer_test_helper
)

target_sources(inverted_pendulum_bot.motion_actuation_test PRIVATE
    TestBridgeMapping.cpp
    TestDrv8711Configuration.cpp
    TestMotionActuationImpl.cpp
)
