#include "core/balance_control/implementations/CascadedPidStrategy.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <algorithm>

namespace balance
{
    namespace
    {
        constexpr std::array<ParameterDescriptor, CascadedPidStrategy::parameterCount> descriptors{ {
            { "pitch.kp", 0.0f, 50.0f },
            { "pitch.ki", 0.0f, 100.0f },
            { "pitch.kd", 0.0f, 5.0f },
            { "velocity.kp", 0.0f, 2.0f },
            { "velocity.ki", 0.0f, 5.0f },
            { "velocity.kd", 0.0f, 1.0f },
            { "yaw.kp", 0.0f, 2.0f },
            { "yaw.ki", 0.0f, 5.0f },
            { "yaw.kd", 0.0f, 1.0f },
        } };
    }

    CascadedPidStrategy::Config::Config() = default;

    CascadedPidStrategy::CascadedPidStrategy(const Config& config)
        : gains(config.gains)
        , pitchLoop({}, { -1.0f, 1.0f })
        , velocityLoop(config.maximumLean)
        , yawLoop(config.maximumDifferential)
    {
        Clear();
    }

    const char* CascadedPidStrategy::Name() const
    {
        return "pid";
    }

    void CascadedPidStrategy::Reset()
    {
        Clear();
    }

    void CascadedPidStrategy::Clear()
    {
        pitchLoop.Reset();
        pitchLoop.SetPoint(0.0f);
        velocityLoop.Reset();
        yawLoop.Reset();

        differential = 0.0f;
        damping = 0.0f;
    }

    void CascadedPidStrategy::Steer(const Setpoints& setpoints, const odometry::ChassisMotion& chassis, float intervalSeconds)
    {
        pitchLoop.SetPoint(-velocityLoop.Process(setpoints.velocity, chassis.forwardVelocity, GainsFrom(velocityKp), intervalSeconds));
        differential = yawLoop.Process(setpoints.yawRate, chassis.yawRate, GainsFrom(yawKp), intervalSeconds);
    }

    Effort CascadedPidStrategy::Balance(const estimation::Estimate& estimate, float intervalSeconds)
    {
        pitchLoop.SetTunings({ gains[pitchKp], gains[pitchKi] * intervalSeconds, 0.0f });
        damping = gains[pitchKd] * estimate.pitchRate;

        const auto common = pitchLoop.Process(estimate.pitch) - damping;

        return Effort{ common - differential, common + differential };
    }

    void CascadedPidStrategy::Saturated(const Effort& applied)
    {
        const auto appliedCommon = (applied.left + applied.right) / 2.0f;

        pitchLoop.SetPreviousOutput(std::clamp(appliedCommon + damping, -1.0f, 1.0f));
    }

    infra::MemoryRange<const ParameterDescriptor> CascadedPidStrategy::Parameters() const
    {
        return infra::MakeRange(descriptors);
    }

    float CascadedPidStrategy::Parameter(std::size_t index) const
    {
        really_assert(index < parameterCount);
        return gains[index];
    }

    void CascadedPidStrategy::SetParameter(std::size_t index, float value)
    {
        really_assert(index < parameterCount);
        gains[index] = value;
    }

    PidGains CascadedPidStrategy::GainsFrom(std::size_t proportional) const
    {
        return PidGains{ gains[proportional], gains[proportional + 1], gains[proportional + 2] };
    }
}
