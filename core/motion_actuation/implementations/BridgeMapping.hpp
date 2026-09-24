#pragma once

#include "core/platform_abstraction/MotorDriver.hpp"
#include <cstdint>

namespace motion
{
    enum class Decay : uint8_t
    {
        slow,
        fast
    };

    platform::BridgeInputs InputsFor(float effort, Decay decay);
    platform::BridgeInputs ReleasedInputs();
    platform::BridgeInputs BrakedInputs();
}
