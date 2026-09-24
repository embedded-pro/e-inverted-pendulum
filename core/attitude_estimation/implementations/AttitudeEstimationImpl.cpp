#include "core/attitude_estimation/implementations/AttitudeEstimationImpl.hpp"
#include <algorithm>
#include <cmath>

namespace estimation
{
    namespace
    {
        constexpr float standardGravity{ 9.80665f };

        float Seconds(infra::Duration interval)
        {
            return std::chrono::duration<float>(interval).count();
        }
    }

    float PitchRateAboutBodyY(const platform::InertialAxes& angularRate)
    {
        return -angularRate.y;
    }

    float AccelerometerPitch(const platform::InertialAxes& acceleration)
    {
        return std::atan2(-acceleration.x, -acceleration.z);
    }

    float AccelerometerTrust(const platform::InertialAxes& acceleration, float accelerationBand)
    {
        const auto magnitude = std::sqrt(acceleration.x * acceleration.x + acceleration.y * acceleration.y + acceleration.z * acceleration.z);
        const auto deviation = std::fabs(magnitude - standardGravity) / (accelerationBand * standardGravity);

        return std::clamp(1.0f - deviation, 0.0f, 1.0f);
    }

    AttitudeEstimationImpl::Config::Config() = default;

    AttitudeEstimationImpl::AttitudeEstimationImpl(const Config& config)
        : config(config)
        , complementary(config.complementary)
        , kalman(config.kalman)
        , selected(config.initialFilter)
    {}

    Estimate AttitudeEstimationImpl::Update(const sensing::Measurement& measurement)
    {
        if (!measurement.valid)
        {
            running = false;
            latest = Estimate{ latest.pitch, 0.0f, false, InvalidCause::sensing };
            return latest;
        }

        const auto interval = measurement.sampledAt - previousSample;

        if (!running || interval <= infra::Duration::zero() || interval > config.maximumInterval)
            latest = Restart(measurement);
        else
            latest = Fuse(measurement, interval);

        previousSample = measurement.sampledAt;
        return latest;
    }

    Estimate AttitudeEstimationImpl::Latest() const
    {
        return latest;
    }

    void AttitudeEstimationImpl::Select(Filter filter)
    {
        selected = filter;
        running = false;
        latest.valid = false;
        latest.cause = InvalidCause::converging;
    }

    Filter AttitudeEstimationImpl::Selected() const
    {
        return selected;
    }

    PitchFilter& AttitudeEstimationImpl::Active()
    {
        if (selected == Filter::kalman)
            return kalman;

        return complementary;
    }

    Estimate AttitudeEstimationImpl::Restart(const sensing::Measurement& measurement)
    {
        const auto pitch = AccelerometerPitch(measurement.acceleration);

        Active().Restart(pitch);
        running = true;
        convergenceStarted = measurement.sampledAt;

        return Estimate{ pitch, PitchRateAboutBodyY(measurement.angularRate), false, InvalidCause::converging };
    }

    Estimate AttitudeEstimationImpl::Fuse(const sensing::Measurement& measurement, infra::Duration interval)
    {
        const auto pitchRate = PitchRateAboutBodyY(measurement.angularRate);
        const auto pitch = Active().Update(pitchRate, AccelerometerPitch(measurement.acceleration), AccelerometerTrust(measurement.acceleration, config.accelerationBand), Seconds(interval));
        const bool converged = measurement.sampledAt - convergenceStarted >= config.convergenceInterval;

        return Estimate{ pitch, pitchRate, converged, converged ? InvalidCause::none : InvalidCause::converging };
    }
}
