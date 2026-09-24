#include "core/inertial_sensing/implementations/InertialSensingImpl.hpp"
#include "core/platform_abstraction/test_doubles/InertialSensorMock.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <optional>
#include <vector>

namespace
{
    constexpr std::chrono::microseconds samplePeriod{ 1000 };

    class InertialSensingImplTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        InertialSensingImplTest()
        {
            EXPECT_CALL(sensor, Start(testing::_)).WillOnce(testing::SaveArg<0>(&onSample));
        }

        ~InertialSensingImplTest() override
        {
            if (sensing)
            {
                EXPECT_CALL(sensor, Stop());
                sensing = std::nullopt;
            }
        }

        sensing::InertialSensingImpl& Sensing()
        {
            if (!sensing)
                sensing.emplace(sensor, config);

            return *sensing;
        }

        void Deliver(float rateX, float rateY, float rateZ, bool valid = true)
        {
            platform::InertialSample sample;

            sample.angularRate = { rateX, rateY, rateZ };
            sample.acceleration = { 0.0f, 0.0f, -9.80665f };
            sample.sampledAt = infra::Now();
            sample.valid = valid;

            Sensing();
            onSample(sample);
        }

        void DeliverForWindow(float rateX, float rateY, float rateZ)
        {
            for (int i = 0; i != 1001; ++i)
            {
                Deliver(rateX, rateY, rateZ);
                ForwardTime(samplePeriod);
            }
        }

        testing::StrictMock<platform::InertialSensorMock> sensor;
        infra::Function<void(const platform::InertialSample&)> onSample;
        sensing::InertialSensingImpl::Config config;
        std::optional<sensing::InertialSensingImpl> sensing;
    };
}

TEST_F(InertialSensingImplTest, nothing_is_reported_before_the_first_sample)
{
    EXPECT_FALSE(Sensing().Latest().valid);
    EXPECT_EQ(sensing::InvalidCause::neverSampled, Sensing().Cause());
    EXPECT_EQ(sensing::InvalidCause::neverSampled, Sensing().Latest().cause);
}

TEST_F(InertialSensingImplTest, a_sample_before_calibration_is_not_usable)
{
    Deliver(0.01f, 0.0f, 0.0f);

    EXPECT_FALSE(Sensing().Latest().valid);
    EXPECT_EQ(sensing::InvalidCause::uncalibrated, Sensing().Cause());
    EXPECT_EQ(sensing::InvalidCause::uncalibrated, Sensing().Latest().cause);
}

TEST_F(InertialSensingImplTest, a_failed_transfer_is_reported_and_not_propagated)
{
    Sensing().StartCalibration();
    DeliverForWindow(0.0f, 0.0f, 0.0f);
    ASSERT_EQ(sensing::CalibrationState::calibrated, Sensing().Calibration());

    Deliver(0.0f, 0.0f, 0.0f, false);

    EXPECT_FALSE(Sensing().Latest().valid);
    EXPECT_EQ(sensing::InvalidCause::transferFailed, Sensing().Cause());
    EXPECT_EQ(sensing::InvalidCause::transferFailed, Sensing().Latest().cause);
}

TEST_F(InertialSensingImplTest, bias_is_the_mean_rate_measured_while_still)
{
    Sensing().StartCalibration();
    DeliverForWindow(0.02f, -0.01f, 0.005f);

    EXPECT_EQ(sensing::CalibrationState::calibrated, Sensing().Calibration());
    EXPECT_NEAR(0.02f, Sensing().GyroscopeBias().x, 1e-5f);
    EXPECT_NEAR(-0.01f, Sensing().GyroscopeBias().y, 1e-5f);
    EXPECT_NEAR(0.005f, Sensing().GyroscopeBias().z, 1e-5f);
}

TEST_F(InertialSensingImplTest, the_measured_bias_is_subtracted_from_later_samples)
{
    Sensing().StartCalibration();
    DeliverForWindow(0.02f, 0.0f, 0.0f);

    Deliver(0.5f, 0.0f, 0.0f);

    EXPECT_TRUE(Sensing().Latest().valid);
    EXPECT_NEAR(0.48f, Sensing().Latest().angularRate.x, 1e-5f);
}

TEST_F(InertialSensingImplTest, acceleration_is_passed_through_uncorrected)
{
    Sensing().StartCalibration();
    DeliverForWindow(0.0f, 0.0f, 0.0f);

    EXPECT_NEAR(-9.80665f, Sensing().Latest().acceleration.z, 1e-5f);
}

TEST_F(InertialSensingImplTest, calibration_is_abandoned_when_the_robot_moves)
{
    Sensing().StartCalibration();

    Deliver(0.0f, 0.0f, 0.0f);
    ForwardTime(samplePeriod);
    Deliver(0.0f, 0.5f, 0.0f);

    EXPECT_EQ(sensing::CalibrationState::failed, Sensing().Calibration());
    EXPECT_FALSE(Sensing().Latest().valid);
}

TEST_F(InertialSensingImplTest, a_failed_transfer_during_calibration_abandons_it)
{
    Sensing().StartCalibration();

    Deliver(0.0f, 0.0f, 0.0f, false);

    EXPECT_EQ(sensing::CalibrationState::failed, Sensing().Calibration());
}

TEST_F(InertialSensingImplTest, a_failed_calibration_leaves_the_previous_bias_unchanged)
{
    Sensing().StartCalibration();
    DeliverForWindow(0.02f, 0.0f, 0.0f);

    Sensing().StartCalibration();
    Deliver(0.0f, 0.5f, 0.0f);

    EXPECT_EQ(sensing::CalibrationState::failed, Sensing().Calibration());
    EXPECT_NEAR(0.02f, Sensing().GyroscopeBias().x, 1e-5f);
}

TEST_F(InertialSensingImplTest, calibration_can_be_retried_after_a_failure)
{
    Sensing().StartCalibration();
    Deliver(0.0f, 0.5f, 0.0f);
    ASSERT_EQ(sensing::CalibrationState::failed, Sensing().Calibration());

    Sensing().StartCalibration();
    DeliverForWindow(0.01f, 0.0f, 0.0f);

    EXPECT_EQ(sensing::CalibrationState::calibrated, Sensing().Calibration());
    EXPECT_NEAR(0.01f, Sensing().GyroscopeBias().x, 1e-5f);
}

TEST_F(InertialSensingImplTest, a_sample_stays_usable_within_three_periods)
{
    Sensing().StartCalibration();
    DeliverForWindow(0.0f, 0.0f, 0.0f);

    Deliver(0.0f, 0.0f, 0.0f);
    ForwardTime(samplePeriod * 3);

    EXPECT_TRUE(Sensing().Latest().valid);
    EXPECT_EQ(sensing::InvalidCause::none, Sensing().Cause());
    EXPECT_EQ(sensing::InvalidCause::none, Sensing().Latest().cause);
}

TEST_F(InertialSensingImplTest, a_stalled_sensor_is_reported_as_stale)
{
    Sensing().StartCalibration();
    DeliverForWindow(0.0f, 0.0f, 0.0f);

    Deliver(0.0f, 0.0f, 0.0f);
    ForwardTime(samplePeriod * 3 + std::chrono::microseconds{ 1 });

    EXPECT_FALSE(Sensing().Latest().valid);
    EXPECT_EQ(sensing::InvalidCause::stale, Sensing().Cause());
    EXPECT_EQ(sensing::InvalidCause::stale, Sensing().Latest().cause);
}

TEST_F(InertialSensingImplTest, a_fresh_sample_clears_staleness)
{
    Sensing().StartCalibration();
    DeliverForWindow(0.0f, 0.0f, 0.0f);

    ForwardTime(samplePeriod * 10);
    ASSERT_EQ(sensing::InvalidCause::stale, Sensing().Cause());

    Deliver(0.0f, 0.0f, 0.0f);

    EXPECT_EQ(sensing::InvalidCause::none, Sensing().Cause());
}

TEST_F(InertialSensingImplTest, calibration_fails_when_the_sensor_stalls_mid_window)
{
    Sensing().StartCalibration();

    Deliver(0.0f, 0.0f, 0.0f);
    ForwardTime(samplePeriod * 4);
    Deliver(0.0f, 0.0f, 0.0f);

    EXPECT_EQ(sensing::CalibrationState::failed, Sensing().Calibration());
}

TEST_F(InertialSensingImplTest, calibration_fails_when_the_first_sample_is_late)
{
    Sensing().StartCalibration();

    ForwardTime(samplePeriod * 4);
    Deliver(0.0f, 0.0f, 0.0f);

    EXPECT_EQ(sensing::CalibrationState::failed, Sensing().Calibration());
}

TEST_F(InertialSensingImplTest, a_window_spanned_by_two_samples_is_not_accepted)
{
    Sensing().StartCalibration();

    Deliver(0.0f, 0.0f, 0.0f);
    ForwardTime(config.calibrationWindow);
    Deliver(0.0f, 0.0f, 0.0f);

    EXPECT_NE(sensing::CalibrationState::calibrated, Sensing().Calibration());
}

TEST_F(InertialSensingImplTest, every_sample_is_reported_as_it_arrives)
{
    std::vector<sensing::Measurement> reported;
    Sensing().OnMeasurement([&reported](const sensing::Measurement& measurement)
        {
            reported.push_back(measurement);
        });

    Deliver(0.1f, 0.2f, 0.3f);
    ForwardTime(samplePeriod);
    Deliver(0.1f, 0.2f, 0.3f, false);

    ASSERT_EQ(2, reported.size());
    EXPECT_EQ(sensing::InvalidCause::uncalibrated, reported[0].cause);
    EXPECT_FLOAT_EQ(0.2f, reported[0].angularRate.y);
    EXPECT_EQ(sensing::InvalidCause::transferFailed, reported[1].cause);
}

TEST_F(InertialSensingImplTest, reported_measurements_are_bias_corrected_once_calibrated)
{
    Sensing().StartCalibration();
    DeliverForWindow(0.01f, -0.02f, 0.03f);

    std::optional<sensing::Measurement> reported;
    Sensing().OnMeasurement([&reported](const sensing::Measurement& measurement)
        {
            reported = measurement;
        });

    Deliver(0.11f, 0.08f, 0.03f);

    ASSERT_TRUE(reported.has_value());
    EXPECT_TRUE(reported->valid);
    EXPECT_NEAR(0.10f, reported->angularRate.y, 1e-5f);
}
