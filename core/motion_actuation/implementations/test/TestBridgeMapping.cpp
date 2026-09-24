#include "core/motion_actuation/implementations/BridgeMapping.hpp"
#include "gtest/gtest.h"
#include <limits>

namespace
{
    platform::BridgeInputs Inputs(uint32_t input1Percent, uint32_t input2Percent)
    {
        return { hal::DutyCycle::FromPercent(input1Percent), hal::DutyCycle::FromPercent(input2Percent) };
    }
}

TEST(BridgeMappingTest, slow_decay_forward_holds_input_one_and_switches_input_two_for_the_remainder)
{
    EXPECT_EQ(Inputs(100, 70), motion::InputsFor(0.3f, motion::Decay::slow));
}

TEST(BridgeMappingTest, slow_decay_reverse_holds_input_two_and_switches_input_one_for_the_remainder)
{
    EXPECT_EQ(Inputs(70, 100), motion::InputsFor(-0.3f, motion::Decay::slow));
}

TEST(BridgeMappingTest, slow_decay_zero_effort_shorts_the_motor)
{
    EXPECT_EQ(Inputs(100, 100), motion::InputsFor(0.0f, motion::Decay::slow));
}

TEST(BridgeMappingTest, slow_decay_full_effort_drives_continuously)
{
    EXPECT_EQ(Inputs(100, 0), motion::InputsFor(1.0f, motion::Decay::slow));
    EXPECT_EQ(Inputs(0, 100), motion::InputsFor(-1.0f, motion::Decay::slow));
}

TEST(BridgeMappingTest, fast_decay_switches_one_input_and_holds_the_other_low)
{
    EXPECT_EQ(Inputs(30, 0), motion::InputsFor(0.3f, motion::Decay::fast));
    EXPECT_EQ(Inputs(0, 30), motion::InputsFor(-0.3f, motion::Decay::fast));
}

TEST(BridgeMappingTest, fast_decay_zero_effort_releases_the_bridge)
{
    EXPECT_EQ(motion::ReleasedInputs(), motion::InputsFor(0.0f, motion::Decay::fast));
}

TEST(BridgeMappingTest, effort_beyond_the_range_is_clamped_not_wrapped)
{
    EXPECT_EQ(Inputs(100, 0), motion::InputsFor(5.0f, motion::Decay::slow));
    EXPECT_EQ(Inputs(0, 100), motion::InputsFor(-5.0f, motion::Decay::slow));
}

TEST(BridgeMappingTest, non_finite_effort_is_treated_as_zero)
{
    EXPECT_EQ(Inputs(100, 100), motion::InputsFor(std::numeric_limits<float>::quiet_NaN(), motion::Decay::slow));
    EXPECT_EQ(motion::ReleasedInputs(), motion::InputsFor(std::numeric_limits<float>::infinity(), motion::Decay::fast));
}

TEST(BridgeMappingTest, duty_keeps_sub_percent_resolution)
{
    const auto inputs = motion::InputsFor(0.305f, motion::Decay::fast);

    EXPECT_GT(inputs.input1.Value(), hal::DutyCycle::FromPercent(30).Value());
    EXPECT_LT(inputs.input1.Value(), hal::DutyCycle::FromPercent(31).Value());
}

TEST(BridgeMappingTest, released_holds_both_inputs_low_and_braked_holds_both_high)
{
    EXPECT_EQ(Inputs(0, 0), motion::ReleasedInputs());
    EXPECT_EQ(Inputs(100, 100), motion::BrakedInputs());
}
