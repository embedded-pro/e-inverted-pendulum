#include "core/balance_control/implementations/LqrStrategy.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <algorithm>

namespace balance
{
    namespace
    {
        constexpr std::array<ParameterDescriptor, LqrStrategy::parameterCount> descriptors{ {
            { "k.pitch", -100.0f, 100.0f },
            { "k.pitchRate", -10.0f, 10.0f },
            { "k.position", -10.0f, 10.0f },
            { "k.velocity", -10.0f, 10.0f },
            { "yaw.kp", 0.0f, 2.0f },
            { "yaw.ki", 0.0f, 5.0f },
            { "yaw.kd", 0.0f, 1.0f },
        } };
    }

    LqrStrategy::Config::Config() = default;

    LqrStrategy::LqrStrategy(const Config& config)
        : maximumPositionDeviation(config.maximumPositionDeviation)
        , gains(config.gains)
        , regulator(RegulatorFor(config.gains))
        , yawLoop(config.maximumDifferential)
    {}

    const char* LqrStrategy::Name() const
    {
        return "lqr";
    }

    void LqrStrategy::Reset()
    {
        Clear();
    }

    void LqrStrategy::Steer(const Setpoints& setpoints, const odometry::ChassisMotion& chassis, float intervalSeconds)
    {
        velocityDeviation = chassis.forwardVelocity - setpoints.velocity;

        if (!saturated)
            positionDeviation = std::clamp(positionDeviation + velocityDeviation * intervalSeconds, -maximumPositionDeviation, maximumPositionDeviation);

        saturated = false;
        differential = yawLoop.Process(setpoints.yawRate, chassis.yawRate, PidGains{ gains[yawKp], gains[yawKi], gains[yawKd] }, intervalSeconds);
    }

    Effort LqrStrategy::Balance(const estimation::Estimate& estimate, float)
    {
        const Regulator::StateVector state{ { estimate.pitch }, { estimate.pitchRate }, { positionDeviation }, { velocityDeviation } };
        const auto common = regulator.ComputeControl(state).at(0, 0);

        return Effort{ common - differential, common + differential };
    }

    void LqrStrategy::Saturated(const Effort&)
    {
        saturated = true;
    }

    infra::MemoryRange<const ParameterDescriptor> LqrStrategy::Parameters() const
    {
        return infra::MakeRange(descriptors);
    }

    float LqrStrategy::Parameter(std::size_t index) const
    {
        really_assert(index < parameterCount);
        return gains[index];
    }

    void LqrStrategy::SetParameter(std::size_t index, float value)
    {
        really_assert(index < parameterCount);
        gains[index] = value;
        regulator = RegulatorFor(gains);
    }

    LqrStrategy::Regulator LqrStrategy::RegulatorFor(const std::array<float, parameterCount>& gains)
    {
        return Regulator{ Regulator::GainMatrix{ { gains[pitchGain], gains[pitchRateGain], gains[positionGain], gains[velocityGain] } } };
    }

    void LqrStrategy::Clear()
    {
        yawLoop.Reset();
        positionDeviation = 0.0f;
        velocityDeviation = 0.0f;
        differential = 0.0f;
        saturated = false;
    }
}
