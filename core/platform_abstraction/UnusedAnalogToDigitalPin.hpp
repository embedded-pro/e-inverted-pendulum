#pragma once

#include "hal/interfaces/AnalogToDigitalPin.hpp"
#include "infra/util/Unit.hpp"
#include <cstdint>

namespace platform
{
    class UnusedAnalogToDigitalPin final
        : public hal::AnalogToDigitalPin<infra::MilliVolt, uint32_t>
    {
    public:
        void Measure(SamplesRange, const infra::Function<void()>&) override
        {}
    };
}
