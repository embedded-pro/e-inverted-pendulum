#pragma once

#include "core/control_loop/interfaces/Stages.hpp"

namespace control
{
    class IdleStages final
        : public BalanceStage
        , public OuterStage
    {
    public:
        void Balance(const estimation::Estimate&, infra::Duration) override
        {}

        void Steer(infra::Duration) override
        {}
    };
}
