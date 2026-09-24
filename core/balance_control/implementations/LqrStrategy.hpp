#pragma once

#include "core/balance_control/implementations/ContinuousPid.hpp"
#include "core/balance_control/interfaces/ControlStrategy.hpp"
#include "numerical/controllers/implementations/Lqr.hpp"
#include <array>

namespace balance
{
    class LqrStrategy final
        : public ControlStrategy
    {
    public:
        enum Gain : std::size_t
        {
            pitchGain,
            pitchRateGain,
            positionGain,
            velocityGain,
            yawKp,
            yawKi,
            yawKd,
            parameterCount
        };

        struct Config
        {
            Config();

            float maximumDifferential{ 0.3f };
            float maximumPositionDeviation{ 0.5f };
            std::array<float, parameterCount> gains{ 2.0f, 0.1f, -0.02f, -0.1f, 0.1f, 0.0f, 0.0f };
        };

        explicit LqrStrategy(const Config& config = Config());

        const char* Name() const override;
        void Reset() override;

        void Steer(const Setpoints& setpoints, const odometry::ChassisMotion& chassis, float intervalSeconds) override;
        Effort Balance(const estimation::Estimate& estimate, float intervalSeconds) override;
        void Saturated(const Effort& applied) override;

        infra::MemoryRange<const ParameterDescriptor> Parameters() const override;
        float Parameter(std::size_t index) const override;
        void SetParameter(std::size_t index, float value) override;

    private:
        using Regulator = controllers::Lqr<float, 4, 1>;

        static Regulator RegulatorFor(const std::array<float, parameterCount>& gains);
        void Clear();

        float maximumPositionDeviation;
        std::array<float, parameterCount> gains;
        Regulator regulator;
        ContinuousPid yawLoop;
        float positionDeviation{ 0.0f };
        float velocityDeviation{ 0.0f };
        float differential{ 0.0f };
        bool saturated{ false };
    };
}
