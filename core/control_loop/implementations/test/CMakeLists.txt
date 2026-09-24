add_executable(inverted_pendulum_bot.control_loop_test)
emil_build_for(inverted_pendulum_bot.control_loop_test HOST All BOOL INVERTED_PENDULUM_BOT_BUILD_TESTS)
emil_add_test(inverted_pendulum_bot.control_loop_test)

target_link_libraries(inverted_pendulum_bot.control_loop_test PUBLIC
    gmock_main
    inverted_pendulum_bot.control_loop.implementations
)

target_sources(inverted_pendulum_bot.control_loop_test PRIVATE
    TestControlLoopImpl.cpp
)
