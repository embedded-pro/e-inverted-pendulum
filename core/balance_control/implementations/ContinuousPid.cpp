#include "core/balance_control/implementations/ContinuousPid.hpp"

namespace balance
{
    ContinuousPid::ContinuousPid(float limit)
        : loop({}, { -limit, limit })
    {
        loop.SetPoint(0.0f);
    }

    void ContinuousPid::Reset()
    {
        loop.Reset();
        loop.SetPoint(0.0f);
    }

    float ContinuousPid::Process(float setpoint, float measured, const PidGains& gains, float intervalSeconds)
    {
        loop.SetTunings({ gains.kp, gains.ki * intervalSeconds, gains.kd / intervalSeconds });
        loop.SetPoint(setpoint);
        return loop.Process(measured);
    }
}
