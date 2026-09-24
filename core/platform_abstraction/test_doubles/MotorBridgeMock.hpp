#pragma once

#include "core/platform_abstraction/MotorBridge.hpp"
#include "gmock/gmock.h"

namespace platform
{
    class MotorBridgeMock
        : public MotorBridge
    {
    public:
        virtual ~MotorBridgeMock() = default;

        MOCK_METHOD(void, SetBaseFrequency, (hal::Hertz baseFrequency), (override));
        MOCK_METHOD(void, Start, (hal::DutyCycle input1, hal::DutyCycle input2), (override));
        MOCK_METHOD(void, Stop, (), (override));
    };
}
