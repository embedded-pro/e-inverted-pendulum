add_executable(inverted_pendulum_bot.platform_abstraction_test)
emil_build_for(inverted_pendulum_bot.platform_abstraction_test HOST All BOOL INVERTED_PENDULUM_BOT_BUILD_TESTS)
emil_add_test(inverted_pendulum_bot.platform_abstraction_test)

target_link_libraries(inverted_pendulum_bot.platform_abstraction_test PUBLIC
    gmock_main
    inverted_pendulum_bot.platform_abstraction
)

target_sources(inverted_pendulum_bot.platform_abstraction_test PRIVATE
    TestInertialFrame.cpp
    TestSignMagnitude.cpp
)
