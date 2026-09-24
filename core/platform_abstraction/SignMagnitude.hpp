#pragma once

#include "hal/synchronous_interfaces/SynchronousPwm.hpp"

namespace platform
{
    struct SignMagnitude
    {
        bool secondInputHigh;
        hal::DutyCycle dutyCycle;
    };

    SignMagnitude AsSignMagnitude(hal::DutyCycle input1, hal::DutyCycle input2);
}
