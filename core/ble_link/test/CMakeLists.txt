add_executable(inverted_pendulum_bot.ble_link_test)
emil_build_for(inverted_pendulum_bot.ble_link_test HOST All BOOL INVERTED_PENDULUM_BOT_BUILD_TESTS)
emil_add_test(inverted_pendulum_bot.ble_link_test)

target_link_libraries(inverted_pendulum_bot.ble_link_test PUBLIC
    gmock_main
    inverted_pendulum_bot.ble_link
    inverted_pendulum_bot.platform_abstraction_mock
    services.ble_test_doubles
    infra.timer_test_helper
)

target_sources(inverted_pendulum_bot.ble_link_test PRIVATE
    Mocks.hpp
    TestBleLink.cpp
    TestDeviceIdentity.cpp
    TestRobotControlService.cpp
    TestWireFormat.cpp
)
