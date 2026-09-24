#pragma once

#include "core/balance_control/interfaces/ControlStrategy.hpp"
#include "numerical/controllers/implementations/PidIncremental.hpp"
#include <array>

namespace balance
{
    class CascadedPidStrategy final
        : public ControlStrategy
    {
    public:
        enum Gain : std::size_t
        {
            pitchKp,
            pitchKi,
            pitchKd,
            velocityKp,
            velocityKi,
            velocityKd,
            yawKp,
            yawKi,
            yawKd,
            parameterCount
        };

        struct Config
        {
            Config();

            float maximumLean{ 10.0f * 3.14159265f / 180.0f };
            float maximumDifferential{ 0.3f };
            std::array<float, parameterCount> gains{ 2.0f, 0.0f, 0.1f, 0.05f, 0.0f, 0.0f, 0.1f, 0.0f, 0.0f };
        };

        explicit CascadedPidStrategy(const Config& config = Config());

        const char* Name() const override;
        void Reset() override;

        void Steer(const Setpoints& setpoints, const odometry::ChassisMotion& chassis, float intervalSeconds) override;
        Effort Balance(const estimation::Estimate& estimate, float intervalSeconds) override;
        void Saturated(const Effort& applied) override;

        infra::MemoryRange<const ParameterDescriptor> Parameters() const override;
        float Parameter(std::size_t index) const override;
        void SetParameter(std::size_t index, float value) override;

    private:
        using Loop = controllers::PidIncrementalSynchronous<float>;

        void Clear();
        controllers::PidTunings<float> Discretised(std::size_t proportional, float intervalSeconds) const;

        std::array<float, parameterCount> gains;
        Loop pitchLoop;
        Loop velocityLoop;
        Loop yawLoop;
        float differential{ 0.0f };
        float damping{ 0.0f };
    };
}
