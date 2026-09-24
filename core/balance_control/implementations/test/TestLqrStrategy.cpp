#include "core/balance_control/implementations/LqrStrategy.hpp"
#include "gtest/gtest.h"
#include <string>

namespace
{
    using Lqr = balance::LqrStrategy;

    constexpr float balanceInterval{ 0.002f };
    constexpr float outerInterval{ 0.02f };

    estimation::Estimate Attitude(float pitch, float pitchRate = 0.0f)
    {
        return estimation::Estimate{ pitch, pitchRate, true, estimation::InvalidCause::none };
    }

    class LqrStrategyTest
        : public testing::Test
    {
    public:
        void OnlyGain(Lqr::Gain gain, float value)
        {
            for (std::size_t index = 0; index < Lqr::parameterCount; ++index)
                strategy.SetParameter(index, 0.0f);

            strategy.SetParameter(gain, value);
        }

        float Common(const estimation::Estimate& estimate = Attitude(0.0f))
        {
            return strategy.Balance(estimate, balanceInterval).left;
        }

        Lqr strategy;
    };
}

TEST_F(LqrStrategyTest, is_named_lqr)
{
    EXPECT_EQ(std::string{ "lqr" }, strategy.Name());
}

TEST_F(LqrStrategyTest, publishes_its_gain_vector_and_yaw_gains_with_ranges)
{
    const auto parameters = strategy.Parameters();

    ASSERT_EQ(Lqr::parameterCount, parameters.size());
    EXPECT_EQ(std::string{ "k.pitch" }, parameters[Lqr::pitchGain].name);
    EXPECT_EQ(std::string{ "k.velocity" }, parameters[Lqr::velocityGain].name);
    EXPECT_EQ(std::string{ "yaw.kp" }, parameters[Lqr::yawKp].name);
    EXPECT_FLOAT_EQ(-10.0f, parameters[Lqr::positionGain].minimum);
    EXPECT_FLOAT_EQ(-0.1f, strategy.Parameter(Lqr::velocityGain));
}

TEST_F(LqrStrategyTest, upright_and_still_needs_no_effort)
{
    EXPECT_EQ((balance::Effort{ 0.0f, 0.0f }), strategy.Balance(Attitude(0.0f), balanceInterval));
}

TEST_F(LqrStrategyTest, leaning_nose_up_drives_both_wheels_backward)
{
    const auto effort = strategy.Balance(Attitude(0.1f), balanceInterval);

    EXPECT_NEAR(-0.2f, effort.left, 1e-6f);
    EXPECT_NEAR(-0.2f, effort.right, 1e-6f);
}

TEST_F(LqrStrategyTest, a_rising_pitch_rate_is_opposed)
{
    EXPECT_NEAR(-0.05f, Common(Attitude(0.0f, 0.5f)), 1e-6f);
}

TEST_F(LqrStrategyTest, a_written_gain_takes_effect_immediately)
{
    strategy.SetParameter(Lqr::pitchGain, 4.0f);

    EXPECT_NEAR(-0.4f, Common(Attitude(0.1f)), 1e-6f);
}

TEST_F(LqrStrategyTest, moving_faster_than_commanded_first_drives_forward_to_lean_back)
{
    strategy.Steer(balance::Setpoints{}, odometry::ChassisMotion{ 0.5f, 0.0f }, outerInterval);

    EXPECT_NEAR(0.1f * 0.5f + 0.02f * 0.01f, Common(), 1e-6f);
}

TEST_F(LqrStrategyTest, the_velocity_deviation_is_taken_against_the_command)
{
    OnlyGain(Lqr::velocityGain, -1.0f);

    strategy.Steer(balance::Setpoints{ 0.3f, 0.0f }, odometry::ChassisMotion{ 0.5f, 0.0f }, outerInterval);

    EXPECT_NEAR(0.2f, Common(), 1e-6f);
}

TEST_F(LqrStrategyTest, the_position_deviation_accumulates_over_the_measured_interval)
{
    OnlyGain(Lqr::positionGain, -1.0f);

    strategy.Steer(balance::Setpoints{}, odometry::ChassisMotion{ 0.5f, 0.0f }, 0.02f);
    EXPECT_NEAR(0.01f, Common(), 1e-6f);

    strategy.Steer(balance::Setpoints{}, odometry::ChassisMotion{ 0.5f, 0.0f }, 0.04f);
    EXPECT_NEAR(0.03f, Common(), 1e-6f);
}

TEST_F(LqrStrategyTest, the_position_deviation_is_bounded_to_half_a_metre)
{
    OnlyGain(Lqr::positionGain, -1.0f);

    strategy.Steer(balance::Setpoints{}, odometry::ChassisMotion{ 1.0f, 0.0f }, 1.0f);
    EXPECT_NEAR(0.5f, Common(), 1e-6f);

    strategy.Steer(balance::Setpoints{}, odometry::ChassisMotion{ -1.0f, 0.0f }, 2.0f);
    EXPECT_NEAR(-0.5f, Common(), 1e-6f);
}

TEST_F(LqrStrategyTest, saturation_holds_the_position_deviation_for_one_outer_iteration)
{
    OnlyGain(Lqr::positionGain, -1.0f);
    strategy.Steer(balance::Setpoints{}, odometry::ChassisMotion{ 0.5f, 0.0f }, outerInterval);

    strategy.Saturated(balance::Effort{ 1.0f, 1.0f });
    strategy.Steer(balance::Setpoints{}, odometry::ChassisMotion{ 0.5f, 0.0f }, outerInterval);
    EXPECT_NEAR(0.01f, Common(), 1e-6f);

    strategy.Steer(balance::Setpoints{}, odometry::ChassisMotion{ 0.5f, 0.0f }, outerInterval);
    EXPECT_NEAR(0.02f, Common(), 1e-6f);
}

TEST_F(LqrStrategyTest, turning_left_speeds_up_the_right_wheel)
{
    strategy.Steer(balance::Setpoints{ 0.0f, 1.0f }, odometry::ChassisMotion{}, outerInterval);

    const auto effort = strategy.Balance(Attitude(0.0f), balanceInterval);

    EXPECT_NEAR(-0.1f, effort.left, 1e-6f);
    EXPECT_NEAR(0.1f, effort.right, 1e-6f);
}

TEST_F(LqrStrategyTest, reset_forgets_deviations_and_the_yaw_loop)
{
    strategy.SetParameter(Lqr::yawKi, 2.0f);
    strategy.Steer(balance::Setpoints{ 0.2f, 1.0f }, odometry::ChassisMotion{ 0.5f, 0.0f }, outerInterval);

    strategy.Reset();

    EXPECT_EQ((balance::Effort{ 0.0f, 0.0f }), strategy.Balance(Attitude(0.0f), balanceInterval));
}
