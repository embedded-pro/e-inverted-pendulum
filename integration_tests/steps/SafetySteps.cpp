#include "cucumber_cpp/Steps.hpp"
#include "integration_tests/support/SafetyFixture.hpp"
#include <chrono>
#include <gtest/gtest.h>

using namespace integration;

namespace
{
    constexpr std::chrono::milliseconds supervisionPeriod{ 2 };
    constexpr std::chrono::milliseconds reactionLimit{ 100 };
    constexpr std::chrono::milliseconds holdPeriod{ 500 };
    constexpr std::chrono::seconds settlePeriod{ 1 };
    constexpr float uprightRate{ 1.0f };
    constexpr float engagedEffort{ 0.5f };

    struct DisableRequests
    {
        bool tristateApplied{ false };
        bool brakeApplied{ false };
    };

    void ExpectCause(const SafetyFixture& fixture)
    {
        ASSERT_TRUE(fixture.expectedCause.has_value());
        EXPECT_EQ(*fixture.expectedCause, fixture.supervisor.LatchedCause());
    }

    bool InFault(SafetyFixture& fixture)
    {
        return fixture.ForwardUntil([&fixture]()
                          {
                              return fixture.supervisor.Current() == safety::Mode::fault;
                          },
                          reactionLimit)
            .has_value();
    }

    void FallIntoFault(SafetyFixture& fixture)
    {
        fixture.TipTo(40.0f);
        ASSERT_TRUE(InFault(fixture));
        ASSERT_EQ(safety::FaultCause::fall, fixture.supervisor.LatchedCause());
    }

    void ReturnUpright(SafetyFixture& fixture)
    {
        fixture.inertial.TiltTo(0.0f, uprightRate);
        fixture.Forward(settlePeriod);
    }
}

GIVEN(R"(the robot is calibrated)")
{
    auto& fixture = *context.Emplace<SafetyFixture>();
    fixture.Calibrate();
    ASSERT_EQ(safety::Mode::idle, fixture.supervisor.Current());
}

GIVEN(R"(the robot is armed and balancing)")
{
    auto& fixture = context.Get<SafetyFixture>();
    fixture.motors.Mark();
    fixture.Arm();
    ASSERT_EQ(safety::Mode::armed, fixture.supervisor.Current());
    ASSERT_TRUE(fixture.motors.EnergisedSinceMark());
    ASSERT_FALSE(fixture.motors.Released());
}

GIVEN(R"(the robot has entered FAULT after a fall)")
{
    FallIntoFault(context.Get<SafetyFixture>());
}

GIVEN(R"(the robot is in FAULT)")
{
    FallIntoFault(context.Get<SafetyFixture>());
}

GIVEN(R"(the body is upright)")
{
    ReturnUpright(context.Get<SafetyFixture>());
}

GIVEN(R"(the actuation component is available)")
{
    ASSERT_EQ(motion::DriverState::ready, context.Get<SafetyFixture>().motionActuation.State());
}

WHEN(R"(the body is tipped {int} degrees from upright)", (std::int32_t degrees))
{
    context.Get<SafetyFixture>().TipTo(static_cast<float>(degrees));
}

WHEN(R"(the body is returned to upright)")
{
    ReturnUpright(context.Get<SafetyFixture>());
}

WHEN(R"(the motor driver asserts its fault output)")
{
    auto& fixture = context.Get<SafetyFixture>();
    fixture.expectedCause = safety::FaultCause::driverFault;
    fixture.motors.RaiseFault();
}

WHEN(R"(the attitude estimate becomes invalid)")
{
    auto& fixture = context.Get<SafetyFixture>();
    fixture.expectedCause = safety::FaultCause::estimateInvalid;
    fixture.inertial.FailTransfers();
}

WHEN(R"(the balance control loop is not serviced for {int} consecutive periods)", (std::int32_t periods))
{
    auto& fixture = context.Get<SafetyFixture>();
    fixture.expectedCause = safety::FaultCause::loopStalled;
    fixture.inertial.Silence();
    fixture.Forward(supervisionPeriod * periods);
}

WHEN(R"(a clear-fault command is issued)")
{
    auto& fixture = context.Get<SafetyFixture>();
    fixture.motors.Mark();
    EXPECT_TRUE(fixture.supervisor.ClearFault());
}

WHEN(R"(a tri-state disable and a braked disable are each requested)")
{
    auto& fixture = context.Get<SafetyFixture>();
    auto& requests = *context.Emplace<DisableRequests>();

    fixture.motionActuation.Apply(engagedEffort, engagedEffort);
    requests.tristateApplied = !fixture.motors.Released();
    fixture.motionActuation.Disable(motion::DisableState::tristate);
    requests.tristateApplied = requests.tristateApplied && fixture.motors.Released();

    fixture.motionActuation.Apply(engagedEffort, engagedEffort);
    requests.brakeApplied = !fixture.motors.Braked();
    fixture.motionActuation.Disable(motion::DisableState::brake);
    requests.brakeApplied = requests.brakeApplied && fixture.motors.Braked();
}

THEN(R"(the mode becomes FAULT within {int} milliseconds)", (std::int32_t milliseconds))
{
    auto& fixture = context.Get<SafetyFixture>();
    ASSERT_TRUE(fixture.thresholdCrossed.has_value());

    const auto deadline = *fixture.thresholdCrossed + std::chrono::milliseconds(milliseconds);
    const auto result = fixture.ForwardUntil([&fixture]()
        {
            return fixture.supervisor.Current() == safety::Mode::fault;
        },
        deadline - infra::Now());

    EXPECT_TRUE(result.has_value());
    EXPECT_TRUE(fixture.motors.Released());
    ExpectCause(fixture);
}

THEN(R"(the mode becomes FAULT)")
{
    auto& fixture = context.Get<SafetyFixture>();
    EXPECT_TRUE(InFault(fixture));
    ExpectCause(fixture);
}

THEN(R"(the mode remains FAULT)")
{
    EXPECT_TRUE(context.Get<SafetyFixture>().ModeHolds(safety::Mode::fault, holdPeriod));
}

THEN(R"(the mode becomes IDLE)")
{
    EXPECT_EQ(safety::Mode::idle, context.Get<SafetyFixture>().supervisor.Current());
}

THEN(R"(both motor bridges are tri-stated)")
{
    EXPECT_TRUE(context.Get<SafetyFixture>().motors.Released());
}

THEN(R"(both motor bridges are disabled)")
{
    EXPECT_TRUE(context.Get<SafetyFixture>().motors.Released());
}

THEN(R"(both motor bridges remain disabled)")
{
    EXPECT_TRUE(context.Get<SafetyFixture>().motors.Released());
}

THEN(R"(the bridges are not actively braked)")
{
    EXPECT_FALSE(context.Get<SafetyFixture>().motors.AnyBraked());
}

THEN(R"(the reported fault cause identifies the motor driver)")
{
    EXPECT_EQ(safety::FaultCause::driverFault, context.Get<SafetyFixture>().supervisor.LatchedCause());
}

THEN(R"(the motors remain de-energised until an arm command is issued)")
{
    auto& fixture = context.Get<SafetyFixture>();
    fixture.Forward(settlePeriod);

    EXPECT_FALSE(fixture.motors.EnergisedSinceMark());
    EXPECT_TRUE(fixture.motors.Released());
}

THEN(R"(both disable states are supported)")
{
    const auto& requests = context.Get<DisableRequests>();
    EXPECT_TRUE(requests.tristateApplied);
    EXPECT_TRUE(requests.brakeApplied);
}

THEN(R"(every safety-initiated disable uses the tri-state)")
{
    auto& fixture = context.Get<SafetyFixture>();

    ASSERT_TRUE(fixture.supervisor.Disarm());
    EXPECT_TRUE(fixture.motors.Released());

    fixture.Arm();
    ASSERT_EQ(safety::Mode::armed, fixture.supervisor.Current());
    FallIntoFault(fixture);
    EXPECT_TRUE(fixture.motors.Released());
}
