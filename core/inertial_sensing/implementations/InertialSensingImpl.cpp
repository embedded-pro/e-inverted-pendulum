#include "core/inertial_sensing/implementations/InertialSensingImpl.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <cmath>

namespace
{
    constexpr float minimumStillness{ 0.01f };
    constexpr float maximumStillness{ 1.0f };
}

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

    void InertialSensingImpl::OnSample(const platform::InertialSample& raw)
    {
        const auto sample = ToBody(raw);
        lastSample = sample;
        sampled = true;

        if (!sample.valid && calibration == CalibrationState::calibrating)
            calibration = CalibrationState::failed;
        else if (calibration == CalibrationState::calibrating)
            Accumulate(sample);

        if (onMeasurement)
            onMeasurement(Latest());
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

    void InertialSensingImpl::OnMeasurement(const infra::Function<void(const Measurement&)>& onMeasurement)
    {
        this->onMeasurement = onMeasurement;
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

    bool InertialSensingImpl::SetAxisMap(const platform::AxisMap& map)
    {
        if (!platform::IsValid(map))
            return false;

        axisMap = map;
        sampled = false;
        ClearBias();
        return true;
    }

    platform::AxisMap InertialSensingImpl::CurrentAxisMap() const
    {
        return axisMap;
    }

    bool InertialSensingImpl::SetStillnessThreshold(float threshold)
    {
        if (!(threshold >= minimumStillness && threshold <= maximumStillness))
            return false;

        config.stillnessThreshold = threshold;
        ClearBias();
        return true;
    }

    float InertialSensingImpl::StillnessThreshold() const
    {
        return config.stillnessThreshold;
    }

    void InertialSensingImpl::ClearBias()
    {
        bias = {};
        biasSum = {};
        biasSamples = 0;
        calibration = calibration == CalibrationState::calibrating ? CalibrationState::failed : CalibrationState::uncalibrated;
    }

    platform::InertialSample InertialSensingImpl::ToBody(const platform::InertialSample& sample) const
    {
        auto body = sample;
        body.angularRate = platform::ToBodyFrame(axisMap, sample.angularRate.x, sample.angularRate.y, sample.angularRate.z);
        body.acceleration = platform::ToBodyFrame(axisMap, sample.acceleration.x, sample.acceleration.y, sample.acceleration.z);
        return body;
    }
}
