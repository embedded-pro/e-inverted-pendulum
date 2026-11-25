#include "core/platform_abstraction/test_doubles/WheelEncodersMock.hpp"
#include "core/wheel_odometry/implementations/WheelOdometryImpl.hpp"
#include "hal/synchronous_interfaces/test_doubles/SynchronousQuadratureEncoderMock.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <limits>
#include <numbers>
#include <optional>

namespace
{
    constexpr uint32_t resolution{ 4096 };
    constexpr std::chrono::microseconds samplePeriod{ 20000 };

    class WheelOdometryImplTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        WheelOdometryImplTest()
        {
            EXPECT_CALL(encoders, Left()).WillRepeatedly(testing::ReturnRef(leftEncoder));
            EXPECT_CALL(encoders, Right()).WillRepeatedly(testing::ReturnRef(rightEncoder));

            EXPECT_CALL(leftEncoder, Resolution()).WillRepeatedly(testing::Return(resolution));
            EXPECT_CALL(rightEncoder, Resolution()).WillRepeatedly(testing::Return(resolution));
        }

        odometry::WheelOdometryImpl& Odometry()
        {
            if (!odometry)
                odometry.emplace(encoders, config);

            return *odometry;
        }

        void Sample(uint32_t positionLeft, uint32_t positionRight)
        {
            EXPECT_CALL(leftEncoder, Position()).WillOnce(testing::Return(positionLeft));
            EXPECT_CALL(rightEncoder, Position()).WillOnce(testing::Return(positionRight));

            Odometry();
            ForwardTime(samplePeriod);
        }

        static float AngularVelocityOf(int32_t counts)
        {
            return 2.0f * std::numbers::pi_v<float> * static_cast<float>(counts) / (static_cast<float>(resolution) * std::chrono::duration<float>(samplePeriod).count());
        }

        testing::StrictMock<hal::SynchronousQuadratureEncoderMock> leftEncoder;
        testing::StrictMock<hal::SynchronousQuadratureEncoderMock> rightEncoder;
        testing::StrictMock<platform::WheelEncodersMock> encoders;
        odometry::WheelOdometryImpl::Config config;
        std::optional<odometry::WheelOdometryImpl> odometry;
    };
}

TEST_F(WheelOdometryImplTest, the_encoders_are_left_alone_until_the_first_sample_is_due)
{
    Odometry();
    ForwardTime(samplePeriod - std::chrono::microseconds{ 1 });

    EXPECT_EQ(0, Odometry().Left().position);
}

TEST_F(WheelOdometryImplTest, nothing_has_moved_before_the_first_two_samples)
{
    Sample(1000, 2000);

    EXPECT_EQ(0, Odometry().Left().position);
    EXPECT_EQ(0, Odometry().Right().position);
    EXPECT_FLOAT_EQ(0.0f, Odometry().Left().angularVelocity);
    EXPECT_FLOAT_EQ(0.0f, Odometry().Right().angularVelocity);
}

TEST_F(WheelOdometryImplTest, the_first_sample_seeds_rather_than_counting_from_zero)
{
    Sample(1000, 2000);
    Sample(1040, 2080);

    EXPECT_EQ(40, Odometry().Left().position);
    EXPECT_EQ(80, Odometry().Right().position);
}

TEST_F(WheelOdometryImplTest, forward_motion_accumulates_across_samples)
{
    Sample(0, 0);
    Sample(40, 40);
    Sample(90, 90);

    EXPECT_EQ(90, Odometry().Left().position);
    EXPECT_EQ(90, Odometry().Right().position);
}

TEST_F(WheelOdometryImplTest, reverse_motion_accumulates_negatively)
{
    Sample(1000, 1000);
    Sample(960, 900);

    EXPECT_EQ(-40, Odometry().Left().position);
    EXPECT_EQ(-100, Odometry().Right().position);
}

TEST_F(WheelOdometryImplTest, a_forward_wrap_is_a_small_step_not_a_revolution_backwards)
{
    Sample(4090, 4090);
    Sample(10, 10);

    EXPECT_EQ(16, Odometry().Left().position);
    EXPECT_EQ(16, Odometry().Right().position);
}

TEST_F(WheelOdometryImplTest, a_reverse_wrap_is_a_small_step_not_a_revolution_forwards)
{
    Sample(10, 10);
    Sample(4090, 4090);

    EXPECT_EQ(-16, Odometry().Left().position);
    EXPECT_EQ(-16, Odometry().Right().position);
}

TEST_F(WheelOdometryImplTest, exactly_half_a_revolution_resolves_by_the_order_of_the_readings)
{
    Sample(0, 0);
    Sample(resolution / 2, resolution / 2);

    EXPECT_EQ(static_cast<int32_t>(resolution / 2), Odometry().Left().position);
    EXPECT_EQ(static_cast<int32_t>(resolution / 2), Odometry().Right().position);
}

TEST_F(WheelOdometryImplTest, exactly_half_a_revolution_read_the_other_way_round_is_negative)
{
    Sample(resolution / 2, resolution / 2);
    Sample(0, 0);

    EXPECT_EQ(-static_cast<int32_t>(resolution / 2), Odometry().Left().position);
    EXPECT_EQ(-static_cast<int32_t>(resolution / 2), Odometry().Right().position);
}

TEST_F(WheelOdometryImplTest, position_survives_repeated_wraps_in_one_direction)
{
    Sample(0, 0);

    for (uint32_t revolution = 0; revolution != 3; ++revolution)
        for (uint32_t step = 1; step != 5; ++step)
            Sample(step * 1024 % resolution, step * 1024 % resolution);

    EXPECT_EQ(3 * static_cast<int32_t>(resolution), Odometry().Left().position);
    EXPECT_EQ(3 * static_cast<int32_t>(resolution), Odometry().Right().position);
}

TEST_F(WheelOdometryImplTest, angular_velocity_follows_the_counts_in_the_last_interval)
{
    Sample(0, 0);
    Sample(40, 80);

    EXPECT_NEAR(AngularVelocityOf(40), Odometry().Left().angularVelocity, 1e-4f);
    EXPECT_NEAR(AngularVelocityOf(80), Odometry().Right().angularVelocity, 1e-4f);
}

TEST_F(WheelOdometryImplTest, angular_velocity_is_signed_by_the_direction_of_travel)
{
    Sample(1000, 1000);
    Sample(960, 1040);

    EXPECT_NEAR(AngularVelocityOf(-40), Odometry().Left().angularVelocity, 1e-4f);
    EXPECT_NEAR(AngularVelocityOf(40), Odometry().Right().angularVelocity, 1e-4f);
}

TEST_F(WheelOdometryImplTest, velocity_divides_by_the_measured_interval_not_the_configured_one)
{
    Sample(0, 0);

    EXPECT_CALL(leftEncoder, Position()).WillOnce(testing::Return(40));
    EXPECT_CALL(rightEncoder, Position()).WillOnce(testing::Return(40));
    systemTimerService.TimeProgressed(samplePeriod * 2);
    ExecuteAllActions();

    EXPECT_NEAR(AngularVelocityOf(40) / 2.0f, Odometry().Left().angularVelocity, 1e-4f);
    EXPECT_NEAR(AngularVelocityOf(40) / 2.0f, Odometry().Right().angularVelocity, 1e-4f);
    EXPECT_EQ(40, Odometry().Left().position);
}

TEST_F(WheelOdometryImplTest, velocity_returns_to_zero_once_the_wheels_stop)
{
    Sample(0, 0);
    Sample(40, 40);
    Sample(40, 40);

    EXPECT_FLOAT_EQ(0.0f, Odometry().Left().angularVelocity);
    EXPECT_FLOAT_EQ(0.0f, Odometry().Right().angularVelocity);
    EXPECT_EQ(40, Odometry().Left().position);
}

TEST_F(WheelOdometryImplTest, a_gear_ratio_scales_the_wheel_down_from_the_encoder)
{
    config.geometry.gearRatio = 30.0f;

    Sample(0, 0);
    Sample(300, 300);

    EXPECT_NEAR(AngularVelocityOf(300) / 30.0f, Odometry().Left().angularVelocity, 1e-4f);
}

TEST_F(WheelOdometryImplTest, a_slower_sample_period_lowers_the_velocity_for_the_same_counts)
{
    config.samplePeriod = samplePeriod * 2;

    EXPECT_CALL(leftEncoder, Position()).WillOnce(testing::Return(0));
    EXPECT_CALL(rightEncoder, Position()).WillOnce(testing::Return(0));
    Odometry();
    ForwardTime(config.samplePeriod);

    EXPECT_CALL(leftEncoder, Position()).WillOnce(testing::Return(40));
    EXPECT_CALL(rightEncoder, Position()).WillOnce(testing::Return(40));
    ForwardTime(config.samplePeriod);

    EXPECT_NEAR(AngularVelocityOf(40) / 2.0f, Odometry().Left().angularVelocity, 1e-4f);
}

TEST_F(WheelOdometryImplTest, equal_wheel_speeds_drive_straight_ahead)
{
    Sample(0, 0);
    Sample(40, 40);

    const auto chassis = Odometry().Chassis();

    EXPECT_NEAR(config.geometry.wheelRadius * AngularVelocityOf(40), chassis.forwardVelocity, 1e-4f);
    EXPECT_NEAR(0.0f, chassis.yawRate, 1e-4f);
}

TEST_F(WheelOdometryImplTest, opposed_wheel_speeds_spin_in_place)
{
    Sample(0, 0);
    Sample(resolution - 40, 40);

    const auto chassis = Odometry().Chassis();

    EXPECT_NEAR(0.0f, chassis.forwardVelocity, 1e-4f);
    EXPECT_NEAR(2.0f * config.geometry.wheelRadius * AngularVelocityOf(40) / config.geometry.trackWidth, chassis.yawRate, 1e-4f);
}

TEST_F(WheelOdometryImplTest, forward_velocity_is_the_mean_of_the_two_wheels)
{
    Sample(0, 0);
    Sample(40, 80);

    EXPECT_NEAR(config.geometry.wheelRadius * AngularVelocityOf(60), Odometry().Chassis().forwardVelocity, 1e-4f);
}

TEST_F(WheelOdometryImplTest, the_right_wheel_running_ahead_yaws_positively)
{
    Sample(0, 0);
    Sample(40, 80);

    EXPECT_GT(Odometry().Chassis().yawRate, 0.0f);
}

TEST_F(WheelOdometryImplTest, the_left_wheel_running_ahead_yaws_negatively)
{
    Sample(0, 0);
    Sample(80, 40);

    EXPECT_LT(Odometry().Chassis().yawRate, 0.0f);
}

TEST_F(WheelOdometryImplTest, a_larger_wheel_covers_more_ground_per_turn)
{
    config.geometry.wheelRadius = 0.068f;

    Sample(0, 0);
    Sample(40, 40);

    EXPECT_NEAR(0.068f * AngularVelocityOf(40), Odometry().Chassis().forwardVelocity, 1e-4f);
}

TEST_F(WheelOdometryImplTest, a_wider_track_yaws_more_slowly_for_the_same_wheel_difference)
{
    config.geometry.trackWidth = 0.290f;

    Sample(0, 0);
    Sample(40, 80);

    EXPECT_NEAR(config.geometry.wheelRadius * (AngularVelocityOf(80) - AngularVelocityOf(40)) / 0.290f, Odometry().Chassis().yawRate, 1e-4f);
}

TEST_F(WheelOdometryImplTest, an_inverted_encoder_counts_the_other_way)
{
    Odometry().SetEncoderInverted(odometry::WheelSide::right, true);
    Sample(1000, 2000);
    Sample(1100, 2100);

    EXPECT_EQ(100, Odometry().Left().position);
    EXPECT_EQ(-100, Odometry().Right().position);
    EXPECT_NEAR(-AngularVelocityOf(100), Odometry().Right().angularVelocity, 1e-3f);
    EXPECT_TRUE(Odometry().EncoderInverted(odometry::WheelSide::right));
    EXPECT_FALSE(Odometry().EncoderInverted(odometry::WheelSide::left));
}

TEST_F(WheelOdometryImplTest, a_new_geometry_scales_the_chassis_motion)
{
    EXPECT_TRUE(Odometry().SetGeometry(odometry::Geometry{ 0.05f, 0.2f, 2.0f }));
    Sample(1000, 1000);
    Sample(1100, 1100);

    EXPECT_NEAR(AngularVelocityOf(100) / 2.0f, Odometry().Left().angularVelocity, 1e-3f);
    EXPECT_NEAR(0.05f * AngularVelocityOf(100) / 2.0f, Odometry().Chassis().forwardVelocity, 1e-4f);
    EXPECT_NEAR(0.2f, Odometry().CurrentGeometry().trackWidth, 1e-6f);
}

TEST_F(WheelOdometryImplTest, a_geometry_outside_its_ranges_is_rejected)
{
    EXPECT_FALSE(Odometry().SetGeometry(odometry::Geometry{ 0.005f, 0.145f, 1.0f }));
    EXPECT_FALSE(Odometry().SetGeometry(odometry::Geometry{ 0.034f, 0.6f, 1.0f }));
    EXPECT_FALSE(Odometry().SetGeometry(odometry::Geometry{ 0.034f, 0.145f, 0.05f }));
    EXPECT_FALSE(Odometry().SetGeometry(odometry::Geometry{ std::numeric_limits<float>::quiet_NaN(), 0.145f, 1.0f }));

    EXPECT_NEAR(0.034f, Odometry().CurrentGeometry().wheelRadius, 1e-6f);
}

TEST_F(WheelOdometryImplTest, an_inverted_left_encoder_counts_back_across_a_wrap)
{
    Odometry().SetEncoderInverted(odometry::WheelSide::left, true);
    Sample(resolution - 10, 0);
    Sample(5, 0);

    EXPECT_EQ(-15, Odometry().Left().position);
    EXPECT_EQ(0, Odometry().Right().position);
}

TEST_F(WheelOdometryImplTest, toggling_the_inversion_mid_run_keeps_the_position_continuous)
{
    Sample(1000, 1000);
    Sample(1100, 1000);
    ASSERT_EQ(100, Odometry().Left().position);

    Odometry().SetEncoderInverted(odometry::WheelSide::left, true);
    Sample(1150, 1000);

    EXPECT_EQ(50, Odometry().Left().position);
}

TEST_F(WheelOdometryImplTest, the_geometry_range_limits_are_accepted_and_nan_is_rejected)
{
    EXPECT_TRUE(Odometry().SetGeometry(odometry::Geometry{ 0.01f, 0.05f, 0.1f }));
    EXPECT_TRUE(Odometry().SetGeometry(odometry::Geometry{ 0.2f, 0.5f, 100.0f }));
    EXPECT_FALSE(Odometry().SetGeometry(odometry::Geometry{ 0.034f, std::numeric_limits<float>::quiet_NaN(), 1.0f }));
    EXPECT_FALSE(Odometry().SetGeometry(odometry::Geometry{ 0.034f, 0.145f, std::numeric_limits<float>::quiet_NaN() }));
}
