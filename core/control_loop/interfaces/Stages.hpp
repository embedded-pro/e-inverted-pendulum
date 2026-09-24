#pragma once

#include "core/attitude_estimation/interfaces/AttitudeEstimation.hpp"
#include "infra/timer/Timer.hpp"

namespace control
{
    class BalanceStage
    {
    public:
        BalanceStage() = default;
        BalanceStage(const BalanceStage& other) = delete;
        BalanceStage& operator=(const BalanceStage& other) = delete;

        virtual void Balance(const estimation::Estimate& estimate, infra::Duration interval) = 0;

    protected:
        ~BalanceStage() = default;
    };

    class OuterStage
    {
    public:
        OuterStage() = default;
        OuterStage(const OuterStage& other) = delete;
        OuterStage& operator=(const OuterStage& other) = delete;

        virtual void Steer(infra::Duration interval) = 0;

    protected:
        ~OuterStage() = default;
    };
}
