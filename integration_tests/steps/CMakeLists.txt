add_library(inverted_pendulum_bot.integration_tests.steps OBJECT
    SafetySteps.cpp
)

target_link_libraries(inverted_pendulum_bot.integration_tests.steps PUBLIC
    inverted_pendulum_bot.integration_tests.support
    cucumber_cpp.runner
    GTest::gtest
    GTest::gmock
)
