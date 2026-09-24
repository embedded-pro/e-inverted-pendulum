#include "core/motion_actuation/implementations/MotionActuationImpl.hpp"
#include "core/platform_abstraction/test_doubles/MotorDriverMock.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <optional>

namespace
{
    class DriverConfigurationMock
        : public motion::DriverConfiguration
    {
    public:
        virtual ~DriverConfigurationMock() = default;

        MOCK_METHOD(void, Configure, (const infra::Function<void(bool verified)>& onDone), (override));
    };

    platform::BridgeInputs Inputs(uint32_t input1Percent, uint32_t input2Percent)
    {
        return { hal::DutyCycle::FromPercent(input1Percent), hal::DutyCycle::FromPercent(input2Percent) };
    }

    const platform::BridgeInputs released{ Inputs(0, 0) };

    class MotionActuationImplTest
        : public testing::Test
    {
    public:
        ~MotionActuationImplTest() override
        {
            if (actuation)
            {
                EXPECT_CALL(motors, DisableFaultNotification());
                EXPECT_CALL(motors, Drive(released, released));
                actuation = std::nullopt;
            }
        }

        void Construct(const motion::MotionActuationImpl::Config& config = motion::MotionActuationImpl::Config())
        {
            EXPECT_CALL(motors, SetBaseFrequency(config.switchingFrequency));
            EXPECT_CALL(motors, Drive(released, released));
            EXPECT_CALL(motors, EnableFaultNotification(testing::_)).WillOnce(testing::SaveArg<0>(&onFault));
            EXPECT_CALL(configuration, Configure(testing::_)).WillOnce(testing::SaveArg<0>(&onConfigured));

            actuation.emplace(motors, configuration, config);
        }

        void ConstructReady(const motion::MotionActuationImpl::Config& config = motion::MotionActuationImpl::Config())
        {
            Construct(config);
            onConfigured(true);
        }

        testing::StrictMock<platform::MotorDriverMock> motors;
        testing::StrictMock<DriverConfigurationMock> configuration;
        infra::Function<void()> onFault;
        infra::Function<void(bool)> onConfigured;
        std::optional<motion::MotionActuationImpl> actuation;
    };
}

TEST_F(MotionActuationImplTest, construction_releases_both_bridges_and_starts_configuring_the_driver)
{
    Construct();

    EXPECT_EQ(motion::DriverState::configuring, actuation->State());
}

TEST_F(MotionActuationImplTest, verified_configuration_makes_the_driver_ready)
{
    ConstructReady();

    EXPECT_EQ(motion::DriverState::ready, actuation->State());
}

TEST_F(MotionActuationImplTest, failed_configuration_is_reported)
{
    Construct();
    onConfigured(false);

    EXPECT_EQ(motion::DriverState::failed, actuation->State());
}

TEST_F(MotionActuationImplTest, effort_is_ignored_while_configuring)
{
    Construct();

    actuation->Apply(0.5f, 0.5f);
}

TEST_F(MotionActuationImplTest, effort_is_ignored_after_configuration_failed)
{
    Construct();
    onConfigured(false);

    actuation->Apply(0.5f, 0.5f);
}

TEST_F(MotionActuationImplTest, positive_effort_drives_forward_in_slow_decay)
{
    ConstructReady();

    EXPECT_CALL(motors, Drive(Inputs(100, 70), Inputs(100, 30)));
    actuation->Apply(0.3f, 0.7f);
}

TEST_F(MotionActuationImplTest, negative_effort_drives_reverse_in_slow_decay)
{
    ConstructReady();

    EXPECT_CALL(motors, Drive(Inputs(70, 100), Inputs(30, 100)));
    actuation->Apply(-0.3f, -0.7f);
}

TEST_F(MotionActuationImplTest, opposite_efforts_turn_the_robot)
{
    ConstructReady();

    EXPECT_CALL(motors, Drive(Inputs(100, 70), Inputs(70, 100)));
    actuation->Apply(0.3f, -0.3f);
}

TEST_F(MotionActuationImplTest, fast_decay_is_selectable)
{
    motion::MotionActuationImpl::Config config;
    config.decay = motion::Decay::fast;
    ConstructReady(config);

    EXPECT_CALL(motors, Drive(Inputs(30, 0), Inputs(0, 70)));
    actuation->Apply(0.3f, -0.7f);
}

TEST_F(MotionActuationImplTest, tristate_releases_both_bridges)
{
    ConstructReady();

    EXPECT_CALL(motors, Drive(released, released));
    actuation->Disable(motion::DisableState::tristate);
}

TEST_F(MotionActuationImplTest, brake_drives_both_inputs_high)
{
    ConstructReady();

    EXPECT_CALL(motors, Drive(Inputs(100, 100), Inputs(100, 100)));
    actuation->Disable(motion::DisableState::brake);
}

TEST_F(MotionActuationImplTest, brake_before_the_driver_is_ready_tristates_instead)
{
    Construct();

    EXPECT_CALL(motors, Drive(released, released));
    actuation->Disable(motion::DisableState::brake);
}

TEST_F(MotionActuationImplTest, driver_fault_tristates_and_latches)
{
    ConstructReady();

    EXPECT_CALL(motors, Drive(released, released));
    onFault();

    EXPECT_EQ(motion::FaultCause::driverFault, actuation->Fault());
}

TEST_F(MotionActuationImplTest, brake_while_a_fault_is_latched_tristates_instead)
{
    ConstructReady();
    EXPECT_CALL(motors, Drive(released, released));
    onFault();

    EXPECT_CALL(motors, Drive(released, released));
    actuation->Disable(motion::DisableState::brake);
}

TEST_F(MotionActuationImplTest, fault_stays_latched_and_ignores_effort_until_cleared)
{
    ConstructReady();
    EXPECT_CALL(motors, Drive(released, released));
    onFault();

    actuation->Apply(0.5f, 0.5f);
    EXPECT_EQ(motion::FaultCause::driverFault, actuation->Fault());

    actuation->ClearFault();
    EXPECT_EQ(motion::FaultCause::none, actuation->Fault());

    EXPECT_CALL(motors, Drive(Inputs(100, 50), Inputs(100, 50)));
    actuation->Apply(0.5f, 0.5f);
}

TEST_F(MotionActuationImplTest, switching_frequency_is_configurable)
{
    motion::MotionActuationImpl::Config config;
    config.switchingFrequency = hal::Hertz{ 20000 };

    Construct(config);
}
