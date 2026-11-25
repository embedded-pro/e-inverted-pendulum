add_library(inverted_pendulum_bot.integration_tests.support OBJECT
    InertialSensorFake.cpp
    InertialSensorFake.hpp
    MotorDriverFake.cpp
    MotorDriverFake.hpp
    SafetyFixture.cpp
    SafetyFixture.hpp
)

target_include_directories(inverted_pendulum_bot.integration_tests.support PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../../>"
)

target_link_libraries(inverted_pendulum_bot.integration_tests.support PUBLIC
    inverted_pendulum_bot.attitude_estimation.implementations
    inverted_pendulum_bot.balance_control.implementations
    inverted_pendulum_bot.control_loop.implementations
    inverted_pendulum_bot.inertial_sensing.implementations
    inverted_pendulum_bot.motion_actuation.implementations
    inverted_pendulum_bot.safety_supervisor.implementations
    inverted_pendulum_bot.wheel_odometry.implementations
    inverted_pendulum_bot.platform_host
    infra.timer_test_helper
    GTest::gtest
    GTest::gmock
)
