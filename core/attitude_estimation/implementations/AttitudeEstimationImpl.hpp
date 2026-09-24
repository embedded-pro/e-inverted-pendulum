#pragma once

#include "core/attitude_estimation/implementations/ComplementaryPitchFilter.hpp"
#include "core/attitude_estimation/implementations/KalmanPitchFilter.hpp"
#include "core/attitude_estimation/interfaces/AttitudeEstimation.hpp"
#include <chrono>

namespace estimation
{
    float PitchRateAboutBodyY(const platform::InertialAxes& angularRate);
    float AccelerometerPitch(const platform::InertialAxes& acceleration);
    float AccelerometerTrust(const platform::InertialAxes& acceleration, float accelerationBand);

    class AttitudeEstimationImpl final
        : public AttitudeEstimation
    {
    public:
        struct Config
        {
            Config();

            Filter initialFilter{ Filter::complementary };
            std::chrono::milliseconds convergenceInterval{ 500 };
            std::chrono::microseconds maximumInterval{ 10000 };
            float accelerationBand{ 0.15f };
            ComplementaryPitchFilter::Config complementary;
            KalmanPitchFilter::Config kalman;
        };

        explicit AttitudeEstimationImpl(const Config& config = Config());
        AttitudeEstimationImpl(const AttitudeEstimationImpl& other) = delete;
        AttitudeEstimationImpl& operator=(const AttitudeEstimationImpl& other) = delete;

        Estimate Update(const sensing::Measurement& measurement) override;
        Estimate Latest() const override;

        void Select(Filter filter) override;
        Filter Selected() const override;

    private:
        PitchFilter& Active();
        Estimate Restart(const sensing::Measurement& measurement);
        Estimate Fuse(const sensing::Measurement& measurement, infra::Duration interval);

        Config config;
        ComplementaryPitchFilter complementary;
        KalmanPitchFilter kalman;
        Filter selected;

        bool running{ false };
        infra::TimePoint previousSample;
        infra::TimePoint convergenceStarted;
        Estimate latest;
    };
}
