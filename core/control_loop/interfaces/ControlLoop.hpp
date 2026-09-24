#pragma once

#include "infra/timer/Timer.hpp"
#include <cstdint>

namespace control
{
    struct TimingStatistics
    {
        uint32_t iterations{ 0 };
        infra::Duration worstJitter{};
        uint32_t lateIterations{ 0 };
    };

    class ControlLoop
    {
    public:
        ControlLoop() = default;
        ControlLoop(const ControlLoop& other) = delete;
        ControlLoop& operator=(const ControlLoop& other) = delete;

        virtual TimingStatistics Statistics() const = 0;
        virtual void ResetStatistics() = 0;

    protected:
        ~ControlLoop() = default;
    };
}
