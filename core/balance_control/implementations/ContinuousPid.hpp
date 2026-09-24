#pragma once

#include "numerical/controllers/implementations/PidIncremental.hpp"

namespace balance
{
    struct PidGains
    {
        float kp{ 0.0f };
        float ki{ 0.0f };
        float kd{ 0.0f };
    };

    class ContinuousPid
    {
    public:
        explicit ContinuousPid(float limit);

        void Reset();
        float Process(float setpoint, float measured, const PidGains& gains, float intervalSeconds);

    private:
        controllers::PidIncrementalSynchronous<float> loop;
    };
}
