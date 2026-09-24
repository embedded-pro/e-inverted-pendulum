#include "core/motion_actuation/implementations/MotionActuationImpl.hpp"
#include "core/platform_abstraction/test_doubles/MotorBridgeMock.hpp"
#include "core/platform_abstraction/test_doubles/MotorDriverMock.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <optional>

namespace
{
    class MotionActuationImplTest
        : public testing::Test
    {
    public:
        MotionActuationImplTest()
        {
            EXPECT_CALL(motors, Left()).WillRepeatedly(testing::ReturnRef(left));
            EXPECT_CALL(motors, Right()).WillRepeatedly(testing::ReturnRef(right));

            EXPECT_CALL(left, SetBaseFrequency(hal::Hertz{ 25000 }));
            EXPECT_CALL(right, SetBaseFrequency(hal::Hertz{ 25000 }));
            EXPECT_CALL(left, Stop());
            EXPECT_CALL(right, Stop());
            EXPECT_CALL(motors, EnableFaultNotification(testing::_))
                .WillOnce(testing::SaveArg<0>(&onFault));

            actuation.emplace(motors);
        }

        ~MotionActuationImplTest() override
        {
            EXPECT_CALL(motors, DisableFaultNotification());
            EXPECT_CALL(left, Stop());
            EXPECT_CALL(right, Stop());
            actuation = std::nullopt;
        }

        testing::StrictMock<platform::MotorBridgeMock> left;
        testing::StrictMock<platform::MotorBridgeMock> right;
        testing::StrictMock<platform::MotorDriverMock> motors;
        infra::Function<void()> onFault;
        std::optional<motion::MotionActuationImpl> actuation;
    };

    class MotionActuationImplConfigTest
        : public testing::Test
    {
    public:
        MotionActuationImplConfigTest()
        {
            EXPECT_CALL(motors, Left()).WillRepeatedly(testing::ReturnRef(left));
            EXPECT_CALL(motors, Right()).WillRepeatedly(testing::ReturnRef(right));
        }

        ~MotionActuationImplConfigTest() override
        {
            if (actuation)
            {
                EXPECT_CALL(motors, DisableFaultNotification());
                EXPECT_CALL(left, Stop());
                EXPECT_CALL(right, Stop());
                actuation = std::nullopt;
            }
        }

        testing::StrictMock<platform::MotorBridgeMock> left;
        testing::StrictMock<platform::MotorBridgeMock> right;
        testing::StrictMock<platform::MotorDriverMock> motors;
        std::optional<motion::MotionActuationImpl> actuation;
    };
}

TEST_F(MotionActuationImplTest, positive_effort_drives_input_one)
{
    EXPECT_CALL(left, Start(hal::DutyCycle::FromPercent(30), hal::DutyCycle::FromPercent(0)));
    EXPECT_CALL(right, Start(hal::DutyCycle::FromPercent(70), hal::DutyCycle::FromPercent(0)));

    actuation->Apply(0.3f, 0.7f);
}

TEST_F(MotionActuationImplTest, negative_effort_drives_input_two)
{
    EXPECT_CALL(left, Start(hal::DutyCycle::FromPercent(0), hal::DutyCycle::FromPercent(30)));
    EXPECT_CALL(right, Start(hal::DutyCycle::FromPercent(0), hal::DutyCycle::FromPercent(70)));

    actuation->Apply(-0.3f, -0.7f);
}

TEST_F(MotionActuationImplTest, opposite_efforts_turn_the_robot)
{
    EXPECT_CALL(left, Start(hal::DutyCycle::FromPercent(30), hal::DutyCycle::FromPercent(0)));
    EXPECT_CALL(right, Start(hal::DutyCycle::FromPercent(0), hal::DutyCycle::FromPercent(30)));

    actuation->Apply(0.3f, -0.3f);
}

TEST_F(MotionActuationImplTest, effort_beyond_the_range_is_clamped_not_wrapped)
{
    EXPECT_CALL(left, Start(hal::DutyCycle::FromPercent(100), hal::DutyCycle::FromPercent(0)));
    EXPECT_CALL(right, Start(hal::DutyCycle::FromPercent(0), hal::DutyCycle::FromPercent(100)));

    actuation->Apply(5.0f, -5.0f);
}

TEST_F(MotionActuationImplTest, zero_effort_holds_both_inputs_low)
{
    EXPECT_CALL(left, Start(hal::DutyCycle::FromPercent(0), hal::DutyCycle::FromPercent(0)));
    EXPECT_CALL(right, Start(hal::DutyCycle::FromPercent(0), hal::DutyCycle::FromPercent(0)));

    actuation->Apply(0.0f, 0.0f);
}

TEST_F(MotionActuationImplTest, tristate_releases_both_bridges)
{
    EXPECT_CALL(left, Stop());
    EXPECT_CALL(right, Stop());

    actuation->Disable(motion::DisableState::tristate);
}

TEST_F(MotionActuationImplTest, brake_drives_both_inputs_high)
{
    EXPECT_CALL(left, Start(hal::DutyCycle::FromPercent(100), hal::DutyCycle::FromPercent(100)));
    EXPECT_CALL(right, Start(hal::DutyCycle::FromPercent(100), hal::DutyCycle::FromPercent(100)));

    actuation->Disable(motion::DisableState::brake);
}

TEST_F(MotionActuationImplTest, driver_fault_tristates_and_latches)
{
    EXPECT_CALL(left, Stop());
    EXPECT_CALL(right, Stop());

    onFault();

    EXPECT_EQ(motion::FaultCause::driverFault, actuation->Fault());
}

TEST_F(MotionActuationImplTest, fault_stays_latched_and_ignores_effort_until_cleared)
{
    EXPECT_CALL(left, Stop());
    EXPECT_CALL(right, Stop());
    onFault();

    actuation->Apply(0.5f, 0.5f);
    EXPECT_EQ(motion::FaultCause::driverFault, actuation->Fault());

    actuation->ClearFault();
    EXPECT_EQ(motion::FaultCause::none, actuation->Fault());

    EXPECT_CALL(left, Start(hal::DutyCycle::FromPercent(50), hal::DutyCycle::FromPercent(0)));
    EXPECT_CALL(right, Start(hal::DutyCycle::FromPercent(50), hal::DutyCycle::FromPercent(0)));
    actuation->Apply(0.5f, 0.5f);
}

TEST_F(MotionActuationImplConfigTest, switching_frequency_is_configurable)
{
    motion::MotionActuationImpl::Config config;
    config.switchingFrequency = hal::Hertz{ 20000 };

    EXPECT_CALL(left, SetBaseFrequency(hal::Hertz{ 20000 }));
    EXPECT_CALL(right, SetBaseFrequency(hal::Hertz{ 20000 }));
    EXPECT_CALL(left, Stop());
    EXPECT_CALL(right, Stop());
    EXPECT_CALL(motors, EnableFaultNotification(testing::_));

    actuation.emplace(motors, config);
}
