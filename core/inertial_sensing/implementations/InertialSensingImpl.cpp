#include "core/inertial_sensing/implementations/InertialSensingImpl.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <cmath>

namespace sensing
{
    InertialSensingImpl::Config::Config() = default;

    InertialSensingImpl::InertialSensingImpl(platform::InertialSensor& sensor, const Config& config)
        : sensor(sensor)
        , config(config)
    {
        really_assert(config.samplePeriod.count() > 0);
        really_assert(config.calibrationWindow.count() > 0);
        really_assert(config.stillnessThreshold > 0.0f);
        really_assert(config.stalePeriods > 0);

        sensor.Start([this](const platform::InertialSample& sample)
            {
                OnSample(sample);
            });
    }

    InertialSensingImpl::~InertialSensingImpl()
    {
        sensor.Stop();
    }

    void InertialSensingImpl::OnSample(const platform::InertialSample& sample)
    {
        lastSample = sample;
        sampled = true;

        if (!sample.valid)
        {
            if (calibration == CalibrationState::calibrating)
                calibration = CalibrationState::failed;

            return;
        }

        if (calibration == CalibrationState::calibrating)
            Accumulate(sample);
    }

    void InertialSensingImpl::Accumulate(const platform::InertialSample& sample)
    {
        if (!Still(sample.angularRate))
        {
            calibration = CalibrationState::failed;
            return;
        }

        if (GapSince(biasSamples == 0 ? calibrationStarted : lastCalibrationSample, sample.sampledAt))
        {
            calibration = CalibrationState::failed;
            return;
        }

        lastCalibrationSample = sample.sampledAt;

        biasSum.x += sample.angularRate.x;
        biasSum.y += sample.angularRate.y;
        biasSum.z += sample.angularRate.z;
        ++biasSamples;

        if (sample.sampledAt - calibrationStarted >= config.calibrationWindow)
            ConcludeCalibration();
    }

    void InertialSensingImpl::ConcludeCalibration()
    {
        really_assert(biasSamples > 0);

        const auto count = static_cast<float>(biasSamples);

        bias.x = biasSum.x / count;
        bias.y = biasSum.y / count;
        bias.z = biasSum.z / count;

        calibration = CalibrationState::calibrated;
    }

    bool InertialSensingImpl::Still(const platform::InertialAxes& rate) const
    {
        return std::fabs(rate.x) <= config.stillnessThreshold && std::fabs(rate.y) <= config.stillnessThreshold && std::fabs(rate.z) <= config.stillnessThreshold;
    }

    bool InertialSensingImpl::Stale() const
    {
        return GapSince(lastSample.sampledAt, infra::Now());
    }

    bool InertialSensingImpl::GapSince(infra::TimePoint previous, infra::TimePoint now) const
    {
        return now - previous > config.stalePeriods * config.samplePeriod;
    }

    Measurement InertialSensingImpl::Latest() const
    {
        Measurement measurement;

        measurement.cause = Cause();

        if (!sampled)
            return measurement;

        measurement.acceleration = lastSample.acceleration;
        measurement.sampledAt = lastSample.sampledAt;

        measurement.angularRate.x = lastSample.angularRate.x - bias.x;
        measurement.angularRate.y = lastSample.angularRate.y - bias.y;
        measurement.angularRate.z = lastSample.angularRate.z - bias.z;

        measurement.valid = measurement.cause == InvalidCause::none;

        return measurement;
    }

    InvalidCause InertialSensingImpl::Cause() const
    {
        if (!sampled)
            return InvalidCause::neverSampled;

        if (!lastSample.valid)
            return InvalidCause::transferFailed;

        if (Stale())
            return InvalidCause::stale;

        if (calibration != CalibrationState::calibrated)
            return InvalidCause::uncalibrated;

        return InvalidCause::none;
    }

    void InertialSensingImpl::StartCalibration()
    {
        calibration = CalibrationState::calibrating;
        biasSum = platform::InertialAxes{};
        biasSamples = 0;
        calibrationStarted = infra::Now();
        lastCalibrationSample = calibrationStarted;
    }

    CalibrationState InertialSensingImpl::Calibration() const
    {
        return calibration;
    }

    platform::InertialAxes InertialSensingImpl::GyroscopeBias() const
    {
        return bias;
    }
}
