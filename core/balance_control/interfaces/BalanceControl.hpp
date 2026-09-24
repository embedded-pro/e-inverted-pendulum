#pragma once

#include "core/balance_control/interfaces/ControlStrategy.hpp"
#include <cstddef>

namespace balance
{
    class BalanceControl
    {
    public:
        BalanceControl() = default;
        BalanceControl(const BalanceControl& other) = delete;
        BalanceControl& operator=(const BalanceControl& other) = delete;

        virtual std::size_t StrategyCount() const = 0;
        virtual const char* StrategyName(std::size_t index) const = 0;
        virtual std::size_t ActiveStrategy() const = 0;
        virtual bool Select(std::size_t index) = 0;

        virtual infra::MemoryRange<const ParameterDescriptor> Parameters() const = 0;
        virtual float Parameter(std::size_t index) const = 0;
        virtual bool SetParameter(std::size_t index, float value) = 0;

        virtual bool Move(const Setpoints& setpoints) = 0;
        virtual void CancelMotion() = 0;
        virtual bool Engaged() const = 0;
        virtual Effort AppliedEffort() const = 0;

    protected:
        ~BalanceControl() = default;
    };
}
