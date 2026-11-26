#include "tools/lqr_design/LqrDesign.hpp"
#include "gtest/gtest.h"
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace
{
    class LqrDesignTest
        : public testing::Test
    {
    public:
        static tools::RobotParameters SmallRobot()
        {
            tools::RobotParameters parameters;
            parameters.bodyMass = 0.8f;
            parameters.wheelsMass = 0.1f;
            parameters.height = 0.1f;
            parameters.bodyInertia = 0.004f;
            parameters.wheelInertia = 5e-5f;
            parameters.wheelRadius = 0.034f;
            parameters.wheelTorque = 0.3f;
            return parameters;
        }
    };
}

TEST_F(LqrDesignTest, firmware_gains_negate_pitch_and_scale_by_both_wheels)
{
    const auto gains = tools::ToFirmwareGains({ -1.2f, -0.12f, -0.6f, -0.3f }, 0.3f);

    EXPECT_NEAR(gains[0], 2.0f, 1e-5f);
    EXPECT_NEAR(gains[1], 0.2f, 1e-5f);
    EXPECT_NEAR(gains[2], -1.0f, 1e-5f);
    EXPECT_NEAR(gains[3], -0.5f, 1e-5f);
}

TEST_F(LqrDesignTest, the_small_robot_gets_a_settling_design_with_the_expected_signs)
{
    const auto design = tools::DesignLqr(SmallRobot());

    EXPECT_EQ(tools::DesignFailure::none, design.failure);
    EXPECT_LT(design.decay, 0.01f);
    EXPECT_GT(design.firmwareGains[0], 0.0f);
    EXPECT_GT(design.firmwareGains[1], 0.0f);
    EXPECT_LT(design.firmwareGains[2], 0.0f);
    EXPECT_LT(design.firmwareGains[3], 0.0f);
}

TEST_F(LqrDesignTest, the_discrete_plant_holds_the_continuous_one_over_one_period)
{
    const auto design = tools::DesignLqr(SmallRobot());

    EXPECT_NEAR(design.a.at(0, 1), 0.002f, 1e-5f);
    EXPECT_NEAR(design.a.at(1, 0), 142.3686f * 0.002f, 1e-3f);
    EXPECT_NEAR(design.b.at(1, 0), -614.0876f * 0.002f, 1e-2f);
    EXPECT_NEAR(design.b.at(3, 0), 79.61313f * 0.002f, 1e-3f);
}

TEST_F(LqrDesignTest, without_feedback_the_plant_falls)
{
    const auto design = tools::DesignLqr(SmallRobot());

    EXPECT_GT(tools::ClosedLoopDecay(design.a, design.b, { 0.0f, 0.0f, 0.0f, 0.0f }, 500), 1.0f);
}

TEST_F(LqrDesignTest, a_gain_outside_its_parameter_range_does_not_fit)
{
    EXPECT_TRUE(tools::FitsFirmwareRanges({ 99.0f, 9.0f, -9.0f, -9.0f }));
    EXPECT_FALSE(tools::FitsFirmwareRanges({ 101.0f, 0.0f, 0.0f, 0.0f }));
    EXPECT_FALSE(tools::FitsFirmwareRanges({ 0.0f, 0.0f, 0.0f, -10.5f }));
}

TEST_F(LqrDesignTest, tight_pitch_limits_leave_the_position_too_slow_to_settle)
{
    auto parameters = SmallRobot();
    parameters.stateLimits = { 0.001f, 0.01f, 0.5f, 1.0f };

    EXPECT_EQ(tools::DesignFailure::notSettled, tools::DesignLqr(parameters).failure);
}

TEST_F(LqrDesignTest, the_firmware_gains_balance_the_plant_in_firmware_conventions)
{
    const auto parameters = SmallRobot();
    const auto design = tools::DesignLqr(parameters);
    math::Vector<float, 4> model{ { 0.05f }, { 0.0f }, { 0.0f }, { 0.0f } };

    for (int step = 0; step != 2500; ++step)
    {
        const float firmwarePitch{ -model[0] };
        const float firmwarePitchRate{ -model[1] };
        const float effort{ -(design.firmwareGains[0] * firmwarePitch + design.firmwareGains[1] * firmwarePitchRate + design.firmwareGains[2] * model[2] + design.firmwareGains[3] * model[3]) };
        const float commonTorque{ 2.0f * parameters.wheelTorque * effort };
        model = design.a * model + design.b * math::Vector<float, 1>{ { commonTorque } };
    }

    for (std::size_t i = 0; i != 4; ++i)
        EXPECT_NEAR(model[i], 0.0f, 5e-4f);
}

TEST_F(LqrDesignTest, a_diverging_state_never_counts_as_settled)
{
    const auto design = tools::DesignLqr(SmallRobot());

    EXPECT_TRUE(std::isinf(tools::ClosedLoopDecay(design.a, design.b, { 0.0f, 0.0f, 0.0f, 0.0f }, 200000)));
}

TEST_F(LqrDesignTest, arguments_set_each_parameter)
{
    const auto parameters = tools::ParseArguments({ "--body-mass", "0.8", "--wheels-mass", "0.1", "--height", "0.1", "--body-inertia", "0.004", "--wheel-inertia", "5e-5", "--wheel-torque", "0.3", "--limit-velocity", "2" });

    ASSERT_TRUE(parameters.has_value());
    EXPECT_NEAR(parameters->bodyMass, 0.8f, 1e-6f);
    EXPECT_NEAR(parameters->wheelInertia, 5e-5f, 1e-9f);
    EXPECT_NEAR(parameters->wheelRadius, 0.034f, 1e-6f);
    EXPECT_NEAR(parameters->stateLimits[3], 2.0f, 1e-6f);
}

TEST_F(LqrDesignTest, bad_arguments_are_rejected)
{
    const std::vector<std::string> valid{ "--body-mass", "0.8", "--height", "0.1", "--wheel-torque", "0.3" };
    EXPECT_TRUE(tools::ParseArguments(valid).has_value());

    for (const std::vector<std::string>& arguments : std::vector<std::vector<std::string>>{
             { "--body-mass" },
             { "--body-mass", "0.8", "--height", "0.1" },
             { "--body-mass", "nan", "--height", "0.1", "--wheel-torque", "0.3" },
             { "--body-mass", "inf", "--height", "0.1", "--wheel-torque", "0.3" },
             { "--body-mass", "0.8x", "--height", "0.1", "--wheel-torque", "0.3" },
             { "--body-mass", "0.8", "--height", "0.1", "--wheel-torque", "0.3", "--limit-pitch", "0" },
             { "--body-mass", "0.8", "--height", "0.1", "--wheel-torque", "0.3", "--period", "1e-9" },
             { "--body-mass", "0.8", "--height", "0.1", "--wheel-torque", "0.3", "--colour", "1" } })
        EXPECT_FALSE(tools::ParseArguments(arguments).has_value());
}
