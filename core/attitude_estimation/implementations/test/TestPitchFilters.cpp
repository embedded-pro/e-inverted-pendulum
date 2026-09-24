#include "core/attitude_estimation/implementations/ComplementaryPitchFilter.hpp"
#include "core/attitude_estimation/implementations/KalmanPitchFilter.hpp"
#include "gtest/gtest.h"

namespace
{
    constexpr float interval{ 0.001f };

    float RunSamples(estimation::PitchFilter& filter, float pitchRate, float accelerometerPitch, float trust, int32_t samples)
    {
        float pitch{ 0.0f };

        for (int32_t sample = 0; sample < samples; ++sample)
            pitch = filter.Update(pitchRate, accelerometerPitch, trust, interval);

        return pitch;
    }
}

TEST(ComplementaryPitchFilterTest, restart_places_the_estimate_at_the_given_pitch)
{
    estimation::ComplementaryPitchFilter filter;

    filter.Restart(0.2f);

    EXPECT_NEAR(0.2f, filter.Update(0.0f, 0.2f, 1.0f, interval), 1e-6f);
}

TEST(ComplementaryPitchFilterTest, integrates_the_rate_over_the_measured_interval)
{
    estimation::ComplementaryPitchFilter filter;
    filter.Restart(0.0f);

    const auto pitch = filter.Update(1.0f, 0.0f, 0.0f, 0.004f);

    EXPECT_NEAR(0.004f, pitch, 1e-6f);
}

TEST(ComplementaryPitchFilterTest, converges_to_the_accelerometer_pitch_within_a_few_crossover_periods)
{
    estimation::ComplementaryPitchFilter filter;
    filter.Restart(0.0f);

    EXPECT_NEAR(0.3f, RunSamples(filter, 0.0f, 0.3f, 1.0f, 3000), 0.3f * 0.01f);
}

TEST(ComplementaryPitchFilterTest, an_untrusted_accelerometer_is_ignored)
{
    estimation::ComplementaryPitchFilter filter;
    filter.Restart(0.0f);

    EXPECT_NEAR(0.0f, RunSamples(filter, 0.0f, 0.3f, 0.0f, 1000), 1e-6f);
}

TEST(ComplementaryPitchFilterTest, partial_trust_pulls_more_slowly_than_full_trust)
{
    estimation::ComplementaryPitchFilter full;
    estimation::ComplementaryPitchFilter half;
    full.Restart(0.0f);
    half.Restart(0.0f);

    EXPECT_LT(RunSamples(half, 0.0f, 0.3f, 0.5f, 200), RunSamples(full, 0.0f, 0.3f, 1.0f, 200));
}

TEST(KalmanPitchFilterTest, restart_places_the_estimate_at_the_given_pitch)
{
    estimation::KalmanPitchFilter filter;

    filter.Restart(-0.1f);

    EXPECT_NEAR(-0.1f, filter.Update(0.0f, -0.1f, 1.0f, interval), 1e-5f);
}

TEST(KalmanPitchFilterTest, converges_to_the_accelerometer_pitch)
{
    estimation::KalmanPitchFilter filter;
    filter.Restart(0.0f);

    EXPECT_NEAR(0.3f, RunSamples(filter, 0.0f, 0.3f, 1.0f, 3000), 0.3f * 0.01f);
}

TEST(KalmanPitchFilterTest, learns_a_constant_gyroscope_bias)
{
    estimation::KalmanPitchFilter filter;
    filter.Restart(0.0f);

    const auto pitch = RunSamples(filter, 0.05f, 0.0f, 1.0f, 20000);

    EXPECT_NEAR(0.05f, filter.EstimatedBias(), 0.005f);
    EXPECT_NEAR(0.0f, pitch, 0.005f);
}

TEST(KalmanPitchFilterTest, below_minimum_trust_only_the_rate_is_integrated)
{
    estimation::KalmanPitchFilter filter;
    filter.Restart(0.0f);

    EXPECT_NEAR(0.1f, RunSamples(filter, 1.0f, 0.5f, 0.0f, 100), 1e-4f);
}
