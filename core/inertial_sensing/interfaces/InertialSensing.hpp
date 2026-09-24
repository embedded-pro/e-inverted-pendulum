#pragma once

#include "core/platform_abstraction/InertialSensor.hpp"
#include "infra/util/Function.hpp"

namespace sensing
{
    enum class CalibrationState : uint8_t
    {
        uncalibrated,
        calibrating,
        calibrated,
        failed
    };

    enum class InvalidCause : uint8_t
    {
        none,
        neverSampled,
        stale,
        transferFailed,
        uncalibrated
    };

    struct Measurement
    {
        platform::InertialAxes angularRate;
        platform::InertialAxes acceleration;
        infra::TimePoint sampledAt;
        bool valid{ false };
        InvalidCause cause{ InvalidCause::neverSampled };
    };

    class InertialSensing
    {
    public:
        InertialSensing() = default;
        InertialSensing(const InertialSensing& other) = delete;
        InertialSensing& operator=(const InertialSensing& other) = delete;

        virtual Measurement Latest() const = 0;
        virtual void OnMeasurement(const infra::Function<void(const Measurement&)>& onMeasurement) = 0;
        virtual InvalidCause Cause() const = 0;

        virtual void StartCalibration() = 0;
        virtual CalibrationState Calibration() const = 0;
        virtual platform::InertialAxes GyroscopeBias() const = 0;

    protected:
        ~InertialSensing() = default;
    };
}
