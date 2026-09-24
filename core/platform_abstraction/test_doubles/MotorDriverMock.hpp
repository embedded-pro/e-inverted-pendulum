#pragma once

#include "core/platform_abstraction/MotorDriver.hpp"
#include "gmock/gmock.h"

namespace platform
{
    class MotorDriverMock
        : public MotorDriver
    {
    public:
        virtual ~MotorDriverMock() = default;

        MOCK_METHOD(void, SetBaseFrequency, (hal::Hertz baseFrequency), (override));
        MOCK_METHOD(void, Drive, (const BridgeInputs& left, const BridgeInputs& right), (override));
        MOCK_METHOD(hal::SpiMaster&, ConfigurationChannel, (), (override));
        MOCK_METHOD(void, EnableFaultNotification, (const infra::Function<void()>& onFault), (override));
        MOCK_METHOD(void, DisableFaultNotification, (), (override));
    };
}
