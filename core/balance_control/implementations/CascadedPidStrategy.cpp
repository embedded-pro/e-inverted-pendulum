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
        , velocityLoop({}, { -config.maximumLean, config.maximumLean })
        , yawLoop({}, { -config.maximumDifferential, config.maximumDifferential })
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
        for (auto* loop : { &pitchLoop, &velocityLoop, &yawLoop })
        {
            loop->Reset();
            loop->SetPoint(0.0f);
        }

        differential = 0.0f;
        damping = 0.0f;
    }

    void CascadedPidStrategy::Steer(const Setpoints& setpoints, const odometry::ChassisMotion& chassis, float intervalSeconds)
    {
        velocityLoop.SetTunings(Discretised(velocityKp, intervalSeconds));
        velocityLoop.SetPoint(setpoints.velocity);
        pitchLoop.SetPoint(-velocityLoop.Process(chassis.forwardVelocity));

        yawLoop.SetTunings(Discretised(yawKp, intervalSeconds));
        yawLoop.SetPoint(setpoints.yawRate);
        differential = yawLoop.Process(chassis.yawRate);
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

    controllers::PidTunings<float> CascadedPidStrategy::Discretised(std::size_t proportional, float intervalSeconds) const
    {
        return { gains[proportional], gains[proportional + 1] * intervalSeconds, gains[proportional + 2] / intervalSeconds };
    }
}
