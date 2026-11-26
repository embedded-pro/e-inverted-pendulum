#include "core/attitude_estimation/implementations/AttitudeEstimationImpl.hpp"

namespace estimation
{
    namespace
    {
        constexpr float standardGravity{ 9.80665f };
        constexpr float maximumUprightOffset{ 0.35f };

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
        return math::Atan2(-acceleration.x, -acceleration.z);
    }

    filters::GravityTilt<float> AccelerometerTilt(const platform::InertialAxes& acceleration, float accelerationBand)
    {
        return filters::TiltFromGravity(-acceleration.x, -acceleration.z, acceleration.y, standardGravity, accelerationBand);
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

    bool AttitudeEstimationImpl::SetUprightOffset(float offset)
    {
        if (!(std::fabs(offset) <= maximumUprightOffset))
            return false;

        latest.pitch += uprightOffset - offset;
        uprightOffset = offset;
        return true;
    }

    float AttitudeEstimationImpl::UprightOffset() const
    {
        return uprightOffset;
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

        return Estimate{ pitch - uprightOffset, PitchRateAboutBodyY(measurement.angularRate), false, InvalidCause::converging };
    }

    Estimate AttitudeEstimationImpl::Fuse(const sensing::Measurement& measurement, infra::Duration interval)
    {
        const auto pitchRate = PitchRateAboutBodyY(measurement.angularRate);
        const auto tilt = AccelerometerTilt(measurement.acceleration, config.accelerationBand);
        const auto pitch = Active().Update(pitchRate, tilt.angle, tilt.trust, Seconds(interval));
        const bool converged = measurement.sampledAt - convergenceStarted >= config.convergenceInterval;

        return Estimate{ pitch - uprightOffset, pitchRate, converged, converged ? InvalidCause::none : InvalidCause::converging };
    }
}
