#pragma once

#include "core/platform_abstraction/MotorBridge.hpp"

namespace application
{
    class MotorBridgeStub final
        : public platform::MotorBridge
    {
    public:
        void SetBaseFrequency(hal::Hertz) override
        {}

        void Start(hal::DutyCycle, hal::DutyCycle) override
        {}

        void Stop() override
        {}
    };
}
