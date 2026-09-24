#include "core/motion_actuation/implementations/Drv8711DriverConfiguration.hpp"
#include "core/platform_abstraction/UnusedAnalogToDigitalPin.hpp"
#include "hal/interfaces/test_doubles/SpiMock.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <chrono>
#include <cstdint>
#include <vector>

namespace
{
    using Drv8711 = drivers::StepperMotorControllerDrv8711;

    std::vector<uint8_t> Frame(uint16_t frame)
    {
        return { static_cast<uint8_t>(frame >> 8), static_cast<uint8_t>(frame & 0xff) };
    }

    class Drv8711DriverConfigurationTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        infra::Function<void(bool)> ReportDone()
        {
            return [this](bool verified)
            {
                done.Call(verified);
            };
        }

        testing::StrictMock<hal::SpiMock> spi;
        platform::UnusedAnalogToDigitalPin backEmf;
        drivers::DirectPwmStepperMotorDrv8711Decorator driver{ spi, hal::dummyPin, hal::dummyPin, backEmf };
        testing::StrictMock<testing::MockFunction<void(bool)>> done;
        motion::Drv8711DriverConfiguration configuration{ driver };
    };
}

TEST(Drv8711CurrentScalingTest, picks_the_highest_sense_gain_that_fits_the_torque_field)
{
    EXPECT_EQ((motion::CurrentScaling{ Drv8711::Isgain::gain40, 186 }), motion::ScaleCurrent(50, 1000));
    EXPECT_EQ((motion::CurrentScaling{ Drv8711::Isgain::gain10, 139 }), motion::ScaleCurrent(50, 3000));
    EXPECT_EQ((motion::CurrentScaling{ Drv8711::Isgain::gain40, 5 }), motion::ScaleCurrent(15, 100));
}

TEST(Drv8711CurrentScalingTest, current_beyond_the_lowest_gain_saturates_the_torque_field)
{
    EXPECT_EQ((motion::CurrentScaling{ Drv8711::Isgain::gain5, 255 }), motion::ScaleCurrent(1000, 10000));
}

TEST(Drv8711DriverConfigurationMappingTest, selects_direct_pwm_with_the_scaled_current_and_chosen_decay)
{
    motion::Drv8711DriverConfiguration::Config config;
    config.senseResistanceMilliOhm = 50;
    config.tripCurrentMilliAmpere = 3000;
    config.deadTime = Drv8711::Dtime::ns400;
    config.currentDecay = Drv8711::DecayMode::slow;

    const auto configuration = motion::Drv8711DriverConfiguration::DriverConfigurationFor(config);

    EXPECT_TRUE(configuration.pwmMode);
    EXPECT_TRUE(configuration.enable);
    EXPECT_EQ(Drv8711::Isgain::gain10, configuration.isgain);
    EXPECT_EQ(139, configuration.torque);
    EXPECT_EQ(Drv8711::Dtime::ns400, configuration.dtime);
    EXPECT_EQ(Drv8711::DecayMode::slow, configuration.decayMode);
}

TEST_F(Drv8711DriverConfigurationTest, nothing_is_sent_before_the_driver_has_woken_up)
{
    configuration.Configure(ReportDone());
    ForwardTime(std::chrono::microseconds{ 999 });
}

TEST_F(Drv8711DriverConfigurationTest, reports_the_driver_verification_after_waking_up)
{
    std::vector<std::vector<uint8_t>> sent;
    EXPECT_CALL(spi, SendDataMock(testing::_, hal::SpiAction::stop)).WillRepeatedly([&sent](std::vector<uint8_t> frame, hal::SpiAction)
        {
            sent.push_back(frame);
        });
    EXPECT_CALL(spi, ReceiveDataMock(hal::SpiAction::stop)).WillOnce(testing::Return(Frame(0x0000)));
    EXPECT_CALL(done, Call(false));

    configuration.Configure(ReportDone());
    ForwardTime(std::chrono::milliseconds{ 1 });
    ExecuteAllActions();

    ASSERT_LE(3u, sent.size());
    EXPECT_EQ(Frame(0x0f00), sent[0]);
    EXPECT_EQ(Frame(0x11ba), sent[1]);
    EXPECT_EQ(Frame(0x2130), sent[2]);
}
