#include "core/balance_control/implementations/CascadedPidStrategy.hpp"
#include "gtest/gtest.h"
#include <string>

namespace
{
    using Pid = balance::CascadedPidStrategy;

    constexpr float balanceInterval{ 0.002f };
    constexpr float outerInterval{ 0.02f };
    constexpr float maximumLean{ 10.0f * 3.14159265f / 180.0f };

    estimation::Estimate Attitude(float pitch, float pitchRate = 0.0f)
    {
        return estimation::Estimate{ pitch, pitchRate, true, estimation::InvalidCause::none };
    }

    class CascadedPidStrategyTest
        : public testing::Test
    {
    public:
        void OnlyGain(Pid::Gain gain, float value)
        {
            for (std::size_t index = 0; index < Pid::parameterCount; ++index)
                strategy.SetParameter(index, 0.0f);

            strategy.SetParameter(gain, value);
        }

        Pid strategy;
    };
}

TEST_F(CascadedPidStrategyTest, is_named_pid)
{
    EXPECT_EQ(std::string{ "pid" }, strategy.Name());
}

TEST_F(CascadedPidStrategyTest, publishes_its_nine_gains_with_ranges)
{
    const auto parameters = strategy.Parameters();

    ASSERT_EQ(Pid::parameterCount, parameters.size());
    EXPECT_EQ(std::string{ "pitch.kp" }, parameters[Pid::pitchKp].name);
    EXPECT_EQ(std::string{ "velocity.ki" }, parameters[Pid::velocityKi].name);
    EXPECT_EQ(std::string{ "yaw.kd" }, parameters[Pid::yawKd].name);
    EXPECT_FLOAT_EQ(50.0f, parameters[Pid::pitchKp].maximum);
    EXPECT_FLOAT_EQ(2.0f, strategy.Parameter(Pid::pitchKp));
}

TEST_F(CascadedPidStrategyTest, a_written_gain_reads_back)
{
    strategy.SetParameter(Pid::yawKi, 1.5f);

    EXPECT_FLOAT_EQ(1.5f, strategy.Parameter(Pid::yawKi));
}

TEST_F(CascadedPidStrategyTest, upright_and_still_needs_no_effort)
{
    EXPECT_EQ((balance::Effort{ 0.0f, 0.0f }), strategy.Balance(Attitude(0.0f), balanceInterval));
}

TEST_F(CascadedPidStrategyTest, leaning_nose_up_drives_both_wheels_backward)
{
    const auto effort = strategy.Balance(Attitude(0.1f), balanceInterval);

    EXPECT_NEAR(-0.2f, effort.left, 1e-6f);
    EXPECT_NEAR(-0.2f, effort.right, 1e-6f);
}

TEST_F(CascadedPidStrategyTest, a_rising_pitch_rate_is_damped_from_the_measured_rate)
{
    const auto effort = strategy.Balance(Attitude(0.0f, 0.5f), balanceInterval);

    EXPECT_NEAR(-0.05f, effort.left, 1e-6f);
    EXPECT_NEAR(-0.05f, effort.right, 1e-6f);
}

TEST_F(CascadedPidStrategyTest, the_pitch_integral_grows_with_the_measured_interval)
{
    OnlyGain(Pid::pitchKi, 10.0f);

    EXPECT_NEAR(-0.002f, strategy.Balance(Attitude(0.1f), 0.002f).left, 1e-6f);
    EXPECT_NEAR(-0.006f, strategy.Balance(Attitude(0.1f), 0.004f).left, 1e-6f);
}

TEST_F(CascadedPidStrategyTest, a_forward_velocity_command_first_drives_back_to_lean_forward)
{
    strategy.Steer(balance::Setpoints{ 0.5f, 0.0f }, odometry::ChassisMotion{}, outerInterval);

    const auto effort = strategy.Balance(Attitude(0.0f), balanceInterval);

    EXPECT_NEAR(-0.05f, effort.left, 1e-6f);
    EXPECT_NEAR(-0.05f, effort.right, 1e-6f);
}

TEST_F(CascadedPidStrategyTest, moving_faster_than_commanded_leans_back_to_slow_down)
{
    strategy.Steer(balance::Setpoints{}, odometry::ChassisMotion{ 0.5f, 0.0f }, outerInterval);

    EXPECT_NEAR(0.05f, strategy.Balance(Attitude(0.0f), balanceInterval).left, 1e-6f);
}

TEST_F(CascadedPidStrategyTest, the_requested_lean_is_limited_to_ten_degrees)
{
    strategy.SetParameter(Pid::velocityKp, 2.0f);
    strategy.Steer(balance::Setpoints{ 1.0f, 0.0f }, odometry::ChassisMotion{}, outerInterval);

    EXPECT_NEAR(-2.0f * maximumLean, strategy.Balance(Attitude(0.0f), balanceInterval).left, 1e-5f);
}

TEST_F(CascadedPidStrategyTest, turning_left_speeds_up_the_right_wheel)
{
    strategy.Steer(balance::Setpoints{ 0.0f, 1.0f }, odometry::ChassisMotion{}, outerInterval);

    const auto effort = strategy.Balance(Attitude(0.0f), balanceInterval);

    EXPECT_NEAR(-0.1f, effort.left, 1e-6f);
    EXPECT_NEAR(0.1f, effort.right, 1e-6f);
}

TEST_F(CascadedPidStrategyTest, the_differential_effort_is_limited)
{
    strategy.SetParameter(Pid::yawKp, 2.0f);
    strategy.Steer(balance::Setpoints{ 0.0f, 1.0f }, odometry::ChassisMotion{}, outerInterval);

    EXPECT_NEAR(0.3f, strategy.Balance(Attitude(0.0f), balanceInterval).right, 1e-6f);
}

TEST_F(CascadedPidStrategyTest, saturation_restarts_accumulation_from_the_applied_effort)
{
    strategy.SetParameter(Pid::pitchKi, 100.0f);
    strategy.SetParameter(Pid::pitchKd, 5.0f);

    const auto first = strategy.Balance(Attitude(0.1f, 0.2f), balanceInterval);
    EXPECT_NEAR(-1.22f, first.left, 1e-5f);

    strategy.Saturated(balance::Effort{ -1.0f, -1.0f });

    EXPECT_NEAR(-1.02f, strategy.Balance(Attitude(0.1f, 0.2f), balanceInterval).left, 1e-5f);
}

TEST_F(CascadedPidStrategyTest, reset_forgets_accumulated_state_and_setpoints)
{
    strategy.SetParameter(Pid::pitchKi, 10.0f);
    strategy.Steer(balance::Setpoints{ 0.5f, 1.0f }, odometry::ChassisMotion{}, outerInterval);
    strategy.Balance(Attitude(0.1f), balanceInterval);

    strategy.Reset();

    EXPECT_EQ((balance::Effort{ 0.0f, 0.0f }), strategy.Balance(Attitude(0.0f), balanceInterval));
}
