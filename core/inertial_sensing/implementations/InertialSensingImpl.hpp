#pragma once

#include "core/inertial_sensing/interfaces/InertialSensing.hpp"
#include <chrono>
#include <cstdint>

namespace sensing
{
    class InertialSensingImpl final
        : public InertialSensing
    {
    public:
        struct Config
        {
            Config();

            std::chrono::microseconds samplePeriod{ 1000 };
            std::chrono::milliseconds calibrationWindow{ 1000 };
            float stillnessThreshold{ 0.09f };
            uint8_t stalePeriods{ 3 };
        };

        explicit InertialSensingImpl(platform::InertialSensor& sensor, const Config& config = Config());
        InertialSensingImpl(const InertialSensingImpl& other) = delete;
        InertialSensingImpl& operator=(const InertialSensingImpl& other) = delete;
        ~InertialSensingImpl();

        Measurement Latest() const override;
        InvalidCause Cause() const override;

        void StartCalibration() override;
        CalibrationState Calibration() const override;
        platform::InertialAxes GyroscopeBias() const override;

    private:
        void OnSample(const platform::InertialSample& sample);
        void Accumulate(const platform::InertialSample& sample);
        void ConcludeCalibration();
        bool Still(const platform::InertialAxes& rate) const;
        bool Stale() const;
        bool GapSince(infra::TimePoint previous, infra::TimePoint now) const;

        platform::InertialSensor& sensor;
        Config config;

        platform::InertialSample lastSample;
        bool sampled{ false };

        CalibrationState calibration{ CalibrationState::uncalibrated };
        platform::InertialAxes bias;
        platform::InertialAxes biasSum;
        uint32_t biasSamples{ 0 };
        infra::TimePoint calibrationStarted;
        infra::TimePoint lastCalibrationSample;
    };
}
