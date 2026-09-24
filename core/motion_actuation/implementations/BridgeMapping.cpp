#include "core/motion_actuation/implementations/BridgeMapping.hpp"
#include <algorithm>
#include <cmath>

namespace motion
{
    namespace
    {
        constexpr hal::DutyCycle off{ 0 };
        constexpr hal::DutyCycle full{ hal::DutyCycle::fullScale };

        hal::DutyCycle DutyOf(float magnitude)
        {
            return hal::DutyCycle{ static_cast<uint32_t>(std::lround(std::clamp(magnitude, 0.0f, 1.0f) * static_cast<float>(hal::DutyCycle::fullScale))) };
        }

        hal::DutyCycle RemainderOf(hal::DutyCycle duty)
        {
            return hal::DutyCycle{ hal::DutyCycle::fullScale - duty.Value() };
        }

        platform::BridgeInputs SlowDecayInputs(bool forward, hal::DutyCycle duty)
        {
            if (forward)
                return { full, RemainderOf(duty) };

            return { RemainderOf(duty), full };
        }

        platform::BridgeInputs FastDecayInputs(bool forward, hal::DutyCycle duty)
        {
            if (forward)
                return { duty, off };

            return { off, duty };
        }
    }

    platform::BridgeInputs InputsFor(float effort, Decay decay)
    {
        const auto clamped = std::isfinite(effort) ? std::clamp(effort, -1.0f, 1.0f) : 0.0f;
        const bool forward = clamped >= 0.0f;
        const auto duty = DutyOf(std::fabs(clamped));

        if (decay == Decay::slow)
            return SlowDecayInputs(forward, duty);

        return FastDecayInputs(forward, duty);
    }

    platform::BridgeInputs ReleasedInputs()
    {
        return { off, off };
    }

    platform::BridgeInputs BrakedInputs()
    {
        return { full, full };
    }
}
