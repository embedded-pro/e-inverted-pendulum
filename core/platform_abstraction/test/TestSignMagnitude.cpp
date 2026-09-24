#include "core/platform_abstraction/SignMagnitude.hpp"
#include "gtest/gtest.h"

TEST(SignMagnitudeTest, both_inputs_low_holds_the_second_input_low_at_zero_duty)
{
    const auto command = platform::AsSignMagnitude(hal::DutyCycle::FromPercent(0), hal::DutyCycle::FromPercent(0));

    EXPECT_FALSE(command.secondInputHigh);
    EXPECT_EQ(hal::DutyCycle::FromPercent(0), command.dutyCycle);
}

TEST(SignMagnitudeTest, both_inputs_high_holds_the_second_input_high_at_full_duty)
{
    const auto command = platform::AsSignMagnitude(hal::DutyCycle::FromPercent(100), hal::DutyCycle::FromPercent(100));

    EXPECT_TRUE(command.secondInputHigh);
    EXPECT_EQ(hal::DutyCycle::FromPercent(100), command.dutyCycle);
}

TEST(SignMagnitudeTest, duty_on_the_first_input_passes_through_unchanged)
{
    const auto command = platform::AsSignMagnitude(hal::DutyCycle::FromPercent(30), hal::DutyCycle::FromPercent(0));

    EXPECT_FALSE(command.secondInputHigh);
    EXPECT_EQ(hal::DutyCycle::FromPercent(30), command.dutyCycle);
}

TEST(SignMagnitudeTest, duty_on_the_second_input_is_complemented)
{
    const auto command = platform::AsSignMagnitude(hal::DutyCycle::FromPercent(0), hal::DutyCycle::FromPercent(30));

    EXPECT_TRUE(command.secondInputHigh);
    EXPECT_EQ(hal::DutyCycle::FromPercent(70), command.dutyCycle);
}

TEST(SignMagnitudeTest, full_duty_on_the_second_input_is_zero_duty_reversed)
{
    const auto command = platform::AsSignMagnitude(hal::DutyCycle::FromPercent(0), hal::DutyCycle::FromPercent(100));

    EXPECT_TRUE(command.secondInputHigh);
    EXPECT_EQ(hal::DutyCycle::FromPercent(0), command.dutyCycle);
}

TEST(SignMagnitudeTest, full_duty_forward_is_distinct_from_brake)
{
    const auto forward = platform::AsSignMagnitude(hal::DutyCycle::FromPercent(100), hal::DutyCycle::FromPercent(0));

    EXPECT_FALSE(forward.secondInputHigh);
    EXPECT_EQ(hal::DutyCycle::FromPercent(100), forward.dutyCycle);
}

TEST(SignMagnitudeTest, full_duty_on_the_second_input_is_distinct_from_brake)
{
    const auto reverse = platform::AsSignMagnitude(hal::DutyCycle::FromPercent(0), hal::DutyCycle::FromPercent(100));
    const auto brake = platform::AsSignMagnitude(hal::DutyCycle::FromPercent(100), hal::DutyCycle::FromPercent(100));

    EXPECT_EQ(reverse.secondInputHigh, brake.secondInputHigh);
    EXPECT_NE(reverse.dutyCycle, brake.dutyCycle);
}
