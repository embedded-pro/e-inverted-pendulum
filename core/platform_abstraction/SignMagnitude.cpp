#include "core/platform_abstraction/SignMagnitude.hpp"
#include "infra/util/ReallyAssert.hpp"

namespace platform
{
    namespace
    {
        hal::DutyCycle Off()
        {
            return hal::DutyCycle::FromPercent(0);
        }

        hal::DutyCycle Full()
        {
            return hal::DutyCycle::FromPercent(100);
        }
    }

    SignMagnitude AsSignMagnitude(hal::DutyCycle input1, hal::DutyCycle input2)
    {
        really_assert(input1 == Off() || input2 == Off() || (input1 == Full() && input2 == Full()));

        if (input1 == Full() && input2 == Full())
            return { true, Full() };

        if (input2 == Off())
            return { false, input1 };

        return { true, hal::DutyCycle{ Full().Value() - input2.Value() } };
    }
}
