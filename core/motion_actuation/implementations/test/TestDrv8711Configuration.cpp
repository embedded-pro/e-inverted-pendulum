#include "core/motion_actuation/implementations/Drv8711Configuration.hpp"
#include "hal/interfaces/test_doubles/SpiMock.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <chrono>
#include <cstdint>
#include <vector>

namespace
{
    std::vector<uint8_t> Frame(uint16_t frame)
    {
        return { static_cast<uint8_t>(frame >> 8), static_cast<uint8_t>(frame & 0xff) };
    }

    class Drv8711ConfigurationTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        void ExpectWrite(uint16_t frame)
        {
            EXPECT_CALL(spi, SendDataMock(Frame(frame), hal::SpiAction::stop));
        }

        void ExpectReadBack(uint16_t frame, uint16_t data)
        {
            EXPECT_CALL(spi, SendDataMock(Frame(frame), hal::SpiAction::stop));
            EXPECT_CALL(spi, ReceiveDataMock(hal::SpiAction::stop)).WillOnce(testing::Return(Frame(data)));
        }

        void ExpectDefaultConfigurationWritten()
        {
            ExpectWrite(0x0f00);
            ExpectWrite(0x11ba);
            ExpectWrite(0x2130);
            ExpectWrite(0x3080);
            ExpectWrite(0x4110);
            ExpectWrite(0x5040);
            ExpectWrite(0x6a59);
        }

        void ExpectDefaultConfigurationReadBackUpToDrive()
        {
            ExpectReadBack(0x8000, 0x0f00);
            ExpectReadBack(0x9000, 0x01ba);
            ExpectReadBack(0xa000, 0x0130);
            ExpectReadBack(0xb000, 0x0080);
            ExpectReadBack(0xc000, 0x0110);
            ExpectReadBack(0xd000, 0x0040);
        }

        infra::Function<void(bool)> ReportDone()
        {
            return [this](bool verified)
            {
                done.Call(verified);
            };
        }

        void RunConfiguration()
        {
            configuration.Configure(ReportDone());
            ForwardTime(std::chrono::milliseconds{ 1 });
            ExecuteAllActions();
        }

        testing::StrictMock<hal::SpiMock> spi;
        testing::StrictMock<testing::MockFunction<void(bool)>> done;
        motion::Drv8711Configuration configuration{ spi };
    };
}

TEST(Drv8711RegistersTest, frames_carry_the_address_in_the_top_nibble_and_twelve_data_bits)
{
    EXPECT_EQ(0x6a59, motion::drv8711::WriteFrame(motion::drv8711::Register::drive, 0xfa59));
    EXPECT_EQ(0x9000, motion::drv8711::ReadFrame(motion::drv8711::Register::torque));
}

TEST(Drv8711RegistersTest, current_scaling_picks_the_highest_gain_that_fits_the_torque_field)
{
    EXPECT_EQ((motion::drv8711::CurrentScaling{ 3, 186 }), motion::drv8711::ScaleCurrent(50, 1000));
    EXPECT_EQ((motion::drv8711::CurrentScaling{ 1, 139 }), motion::drv8711::ScaleCurrent(50, 3000));
    EXPECT_EQ((motion::drv8711::CurrentScaling{ 3, 5 }), motion::drv8711::ScaleCurrent(15, 100));
}

TEST(Drv8711RegistersTest, current_beyond_the_lowest_gain_saturates_the_torque_field)
{
    EXPECT_EQ((motion::drv8711::CurrentScaling{ 0, 255 }), motion::drv8711::ScaleCurrent(1000, 10000));
}

TEST_F(Drv8711ConfigurationTest, nothing_is_sent_before_the_driver_has_woken_up)
{
    configuration.Configure(ReportDone());
    ForwardTime(std::chrono::microseconds{ 999 });
}

TEST_F(Drv8711ConfigurationTest, writes_reads_back_then_clears_status_and_enables_last)
{
    testing::InSequence sequence;

    ExpectDefaultConfigurationWritten();
    ExpectDefaultConfigurationReadBackUpToDrive();
    ExpectReadBack(0xe000, 0x0a59);
    ExpectWrite(0x7000);
    ExpectWrite(0x0f01);
    EXPECT_CALL(done, Call(true));

    RunConfiguration();
}

TEST_F(Drv8711ConfigurationTest, write_only_torque_bit_is_ignored_on_read_back)
{
    testing::InSequence sequence;

    ExpectDefaultConfigurationWritten();
    ExpectReadBack(0x8000, 0x0f00);
    ExpectReadBack(0x9000, 0x05ba);
    ExpectReadBack(0xa000, 0x0130);
    ExpectReadBack(0xb000, 0x0080);
    ExpectReadBack(0xc000, 0x0110);
    ExpectReadBack(0xd000, 0x0040);
    ExpectReadBack(0xe000, 0x0a59);
    ExpectWrite(0x7000);
    ExpectWrite(0x0f01);
    EXPECT_CALL(done, Call(true));

    RunConfiguration();
}

TEST_F(Drv8711ConfigurationTest, read_back_mismatch_fails_without_enabling_the_driver)
{
    testing::InSequence sequence;

    ExpectDefaultConfigurationWritten();
    ExpectDefaultConfigurationReadBackUpToDrive();
    ExpectReadBack(0xe000, 0x0000);
    EXPECT_CALL(done, Call(false));

    RunConfiguration();
}

TEST_F(Drv8711ConfigurationTest, absent_driver_reads_back_nothing_and_fails)
{
    testing::InSequence sequence;

    ExpectDefaultConfigurationWritten();
    for (uint16_t address = 0; address != 7; ++address)
        ExpectReadBack(static_cast<uint16_t>(0x8000 | (address << 12)), 0x0000);
    EXPECT_CALL(done, Call(false));

    RunConfiguration();
}

TEST_F(Drv8711ConfigurationTest, current_limit_and_timing_are_taken_from_the_configuration)
{
    motion::Drv8711Configuration::Config config;
    config.senseResistanceMilliOhm = 50;
    config.tripCurrentMilliAmpere = 3000;
    config.deadTime = motion::Drv8711Configuration::DeadTime::nanoseconds400;
    config.fixedOffTimeSteps = 0x20;
    motion::Drv8711Configuration configured{ spi, config };

    std::vector<std::vector<uint8_t>> sent;
    EXPECT_CALL(spi, SendDataMock(testing::_, hal::SpiAction::stop)).WillRepeatedly([&sent](std::vector<uint8_t> frame, hal::SpiAction)
        {
            sent.push_back(frame);
        });
    EXPECT_CALL(spi, ReceiveDataMock(hal::SpiAction::stop)).WillRepeatedly(testing::Return(Frame(0x0000)));
    EXPECT_CALL(done, Call(false));

    configured.Configure(ReportDone());
    ForwardTime(std::chrono::milliseconds{ 1 });
    ExecuteAllActions();

    ASSERT_EQ(14, sent.size());
    EXPECT_EQ(Frame(0x0100), sent[0]);
    EXPECT_EQ(Frame(0x118b), sent[1]);
    EXPECT_EQ(Frame(0x2120), sent[2]);
}
