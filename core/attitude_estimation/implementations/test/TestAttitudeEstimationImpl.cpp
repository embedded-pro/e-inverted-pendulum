#include "core/attitude_estimation/implementations/AttitudeEstimationImpl.hpp"
#include "gtest/gtest.h"
#include <chrono>
#include <cmath>

namespace
{
    constexpr float gravity{ 9.80665f };

    platform::InertialAxes GravityAtPitch(float pitch, float scale = 1.0f)
    {
        return platform::InertialAxes{ -gravity * scale * std::sin(pitch), 0.0f, -gravity * scale * std::cos(pitch) };
    }

    class AttitudeEstimationImplTest
        : public testing::Test
    {
    public:
        sensing::Measurement Sample(float pitch, float pitchRate = 0.0f, float gravityScale = 1.0f)
        {
            sensing::Measurement measurement;
            measurement.angularRate = platform::InertialAxes{ 0.0f, -pitchRate, 0.0f };
            measurement.acceleration = GravityAtPitch(pitch, gravityScale);
            measurement.sampledAt = now;
            measurement.valid = true;
            measurement.cause = sensing::InvalidCause::none;

            now += std::chrono::milliseconds{ 1 };
            return measurement;
        }

        sensing::Measurement Invalid()
        {
            sensing::Measurement measurement;
            measurement.sampledAt = now;
            measurement.valid = false;
            measurement.cause = sensing::InvalidCause::stale;

            now += std::chrono::milliseconds{ 1 };
            return measurement;
        }

        estimation::Estimate Settle(float pitch, int32_t samples = 600)
        {
            estimation::Estimate estimate;

            for (int32_t sample = 0; sample < samples; ++sample)
                estimate = estimation.Update(Sample(pitch));

            return estimate;
        }

        infra::TimePoint now{ std::chrono::seconds{ 1 } };
        estimation::AttitudeEstimationImpl estimation;
    };
}

TEST(AttitudeEstimationGeometryTest, nose_up_rotation_is_negative_rate_about_body_y)
{
    EXPECT_FLOAT_EQ(0.5f, estimation::PitchRateAboutBodyY(platform::InertialAxes{ 0.0f, -0.5f, 0.0f }));
}

TEST(AttitudeEstimationGeometryTest, upright_is_zero_pitch_and_nose_up_is_positive)
{
    EXPECT_NEAR(0.0f, estimation::AccelerometerPitch(platform::InertialAxes{ 0.0f, 0.0f, -gravity }), 1e-6f);
    EXPECT_NEAR(0.2f, estimation::AccelerometerPitch(GravityAtPitch(0.2f)), 1e-5f);
    EXPECT_NEAR(-0.2f, estimation::AccelerometerPitch(GravityAtPitch(-0.2f)), 1e-5f);
}

TEST(AttitudeEstimationGeometryTest, trust_falls_off_linearly_across_the_acceleration_band)
{
    EXPECT_FLOAT_EQ(1.0f, estimation::AccelerometerTrust(GravityAtPitch(0.0f), 0.15f));
    EXPECT_NEAR(0.5f, estimation::AccelerometerTrust(GravityAtPitch(0.0f, 1.075f), 0.15f), 1e-4f);
    EXPECT_NEAR(0.5f, estimation::AccelerometerTrust(GravityAtPitch(0.0f, 0.925f), 0.15f), 1e-4f);
    EXPECT_FLOAT_EQ(0.0f, estimation::AccelerometerTrust(GravityAtPitch(0.0f, 1.5f), 0.15f));
}

TEST_F(AttitudeEstimationImplTest, nothing_estimated_yet_is_invalid_for_sensing)
{
    EXPECT_FALSE(estimation.Latest().valid);
    EXPECT_EQ(estimation::InvalidCause::sensing, estimation.Latest().cause);
}

TEST_F(AttitudeEstimationImplTest, first_measurement_starts_at_the_accelerometer_pitch_while_converging)
{
    const auto estimate = estimation.Update(Sample(0.1f, 0.2f));

    EXPECT_NEAR(0.1f, estimate.pitch, 1e-5f);
    EXPECT_NEAR(0.2f, estimate.pitchRate, 1e-6f);
    EXPECT_FALSE(estimate.valid);
    EXPECT_EQ(estimation::InvalidCause::converging, estimate.cause);
}

TEST_F(AttitudeEstimationImplTest, becomes_valid_after_the_convergence_interval)
{
    Settle(0.1f, 500);
    EXPECT_FALSE(estimation.Latest().valid);

    const auto estimate = estimation.Update(Sample(0.1f));

    EXPECT_TRUE(estimate.valid);
    EXPECT_EQ(estimation::InvalidCause::none, estimate.cause);
    EXPECT_NEAR(0.1f, estimate.pitch, 1e-4f);
}

TEST_F(AttitudeEstimationImplTest, an_invalid_measurement_invalidates_the_estimate_and_restarts_convergence)
{
    Settle(0.1f);

    EXPECT_FALSE(estimation.Update(Invalid()).valid);
    EXPECT_EQ(estimation::InvalidCause::sensing, estimation.Latest().cause);

    EXPECT_EQ(estimation::InvalidCause::converging, estimation.Update(Sample(0.1f)).cause);
}

TEST_F(AttitudeEstimationImplTest, a_gap_longer_than_the_maximum_interval_restarts_convergence)
{
    Settle(0.1f);

    now += std::chrono::milliseconds{ 20 };

    EXPECT_EQ(estimation::InvalidCause::converging, estimation.Update(Sample(0.1f)).cause);
}

TEST_F(AttitudeEstimationImplTest, the_rate_is_integrated_over_the_measured_interval)
{
    estimation::AttitudeEstimationImpl::Config config;
    config.accelerationBand = 1.0e-6f;
    estimation::AttitudeEstimationImpl gyroscopeOnly{ config };

    gyroscopeOnly.Update(Sample(0.0f));
    now += std::chrono::milliseconds{ 2 };
    const auto estimate = gyroscopeOnly.Update(Sample(0.0f, 1.0f, 1.01f));

    EXPECT_NEAR(0.003f, estimate.pitch, 1e-5f);
}

TEST_F(AttitudeEstimationImplTest, complementary_is_selected_by_default)
{
    EXPECT_EQ(estimation::Filter::complementary, estimation.Selected());
}

TEST_F(AttitudeEstimationImplTest, selecting_a_filter_restarts_convergence)
{
    Settle(0.1f);

    estimation.Select(estimation::Filter::kalman);

    EXPECT_EQ(estimation::Filter::kalman, estimation.Selected());
    EXPECT_FALSE(estimation.Latest().valid);
    EXPECT_EQ(estimation::InvalidCause::converging, estimation.Latest().cause);
    EXPECT_EQ(estimation::InvalidCause::converging, estimation.Update(Sample(0.1f)).cause);
}

TEST_F(AttitudeEstimationImplTest, kalman_tracks_the_pitch_once_selected)
{
    estimation.Select(estimation::Filter::kalman);

    const auto estimate = Settle(-0.15f);

    EXPECT_TRUE(estimate.valid);
    EXPECT_NEAR(-0.15f, estimate.pitch, 1e-3f);
}
