#include "core/wheel_odometry/implementations/WheelOdometryImpl.hpp"
#include "infra/util/ReallyAssert.hpp"
#include "robotics/mobile/DifferentialDrive.hpp"
#include <numbers>

namespace
{
    constexpr float minimumRadius{ 0.01f };
    constexpr float maximumRadius{ 0.2f };
    constexpr float minimumTrack{ 0.05f };
    constexpr float maximumTrack{ 0.5f };
    constexpr float minimumGear{ 0.1f };
    constexpr float maximumGear{ 100.0f };
    constexpr float maximumTrackableWheelRate{ 25.0f * 2.0f * std::numbers::pi_v<float> };

    bool Within(float value, float minimum, float maximum)
    {
        return value >= minimum && value <= maximum;
    }

    bool Valid(const odometry::Geometry& geometry)
    {
        return Within(geometry.wheelRadius, minimumRadius, maximumRadius) && Within(geometry.trackWidth, minimumTrack, maximumTrack) && Within(geometry.gearRatio, minimumGear, maximumGear);
    }

    std::chrono::microseconds VerifiedSamplePeriod(std::chrono::microseconds samplePeriod)
    {
        really_assert(samplePeriod.count() > 0);
        return samplePeriod;
    }
}

namespace odometry
{
    WheelOdometryImpl::Config::Config() = default;

    WheelOdometryImpl::WheelOdometryImpl(platform::WheelEncoders& encoders, const Config& config)
        : encoders(encoders)
        , config(config)
        , previousSampleTime{ infra::Now() }
        , sampleTimer{ VerifiedSamplePeriod(config.samplePeriod), [this]()
            {
                Sample();
            } }
    {
        really_assert(Valid(config.geometry));
    }

    int32_t WheelOdometryImpl::ShortestDelta(uint32_t previous, uint32_t current, uint32_t resolution)
    {
        const auto span = static_cast<int32_t>(resolution);
        auto delta = static_cast<int32_t>(current) - static_cast<int32_t>(previous);

        if (delta > span / 2)
            delta -= span;
        else if (delta < -span / 2)
            delta += span;

        return delta;
    }

    void WheelOdometryImpl::SampleWheel(Wheel& wheel, hal::SynchronousQuadratureEncoder& encoder, infra::Duration elapsed) const
    {
        const auto resolution = encoder.Resolution();
        const auto raw = encoder.Position();

        really_assert(resolution >= 2);
        really_assert(raw < resolution);

        if (!wheel.sampled)
        {
            wheel.previousRaw = raw;
            wheel.sampled = true;
            return;
        }

        const auto counted = ShortestDelta(wheel.previousRaw, raw, resolution);
        const auto delta = wheel.inverted ? -counted : counted;
        wheel.previousRaw = raw;

        const auto seconds = std::chrono::duration<float>(elapsed).count();
        const auto countsPerWheelRevolution = static_cast<float>(resolution) * config.geometry.gearRatio;

        wheel.motion.position += delta;
        wheel.motion.angularVelocity = 2.0f * std::numbers::pi_v<float> * static_cast<float>(delta) / (countsPerWheelRevolution * seconds);
    }

    void WheelOdometryImpl::Sample()
    {
        const auto now = infra::Now();
        const auto elapsed = now - previousSampleTime;
        previousSampleTime = now;

        really_assert(elapsed > infra::Duration::zero());

        SampleWheel(left, encoders.Left(), elapsed);
        SampleWheel(right, encoders.Right(), elapsed);
    }

    WheelMotion WheelOdometryImpl::Left() const
    {
        return left.motion;
    }

    WheelMotion WheelOdometryImpl::Right() const
    {
        return right.motion;
    }

    ChassisMotion WheelOdometryImpl::Chassis() const
    {
        const mobile::DifferentialDrive drive{ config.geometry.wheelRadius, config.geometry.trackWidth, maximumTrackableWheelRate };
        const auto twist = drive.Forward(mobile::WheelSpeeds<float>{ left.motion.angularVelocity, right.motion.angularVelocity });

        return ChassisMotion{ twist[0], twist[2] };
    }

    void WheelOdometryImpl::SetEncoderInverted(WheelSide side, bool inverted)
    {
        WheelOn(side).inverted = inverted;
    }

    bool WheelOdometryImpl::EncoderInverted(WheelSide side) const
    {
        return WheelOn(side).inverted;
    }

    bool WheelOdometryImpl::SetGeometry(const Geometry& geometry)
    {
        if (!Valid(geometry))
            return false;

        config.geometry = geometry;
        return true;
    }

    Geometry WheelOdometryImpl::CurrentGeometry() const
    {
        return config.geometry;
    }

    WheelOdometryImpl::Wheel& WheelOdometryImpl::WheelOn(WheelSide side)
    {
        return side == WheelSide::left ? left : right;
    }

    const WheelOdometryImpl::Wheel& WheelOdometryImpl::WheelOn(WheelSide side) const
    {
        return side == WheelSide::left ? left : right;
    }
}
