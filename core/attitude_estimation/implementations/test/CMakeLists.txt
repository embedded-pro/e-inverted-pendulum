add_executable(inverted_pendulum_bot.attitude_estimation_test)
emil_build_for(inverted_pendulum_bot.attitude_estimation_test HOST All BOOL INVERTED_PENDULUM_BOT_BUILD_TESTS)
emil_add_test(inverted_pendulum_bot.attitude_estimation_test)

target_link_libraries(inverted_pendulum_bot.attitude_estimation_test PUBLIC
    gmock_main
    inverted_pendulum_bot.attitude_estimation.implementations
)

target_sources(inverted_pendulum_bot.attitude_estimation_test PRIVATE
    TestAttitudeEstimationImpl.cpp
    TestPitchFilters.cpp
)
