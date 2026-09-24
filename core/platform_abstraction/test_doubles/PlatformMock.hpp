#pragma once

#include "core/platform_abstraction/Platform.hpp"
#include "gmock/gmock.h"

namespace platform
{
    class PlatformMock
        : public Platform
    {
    public:
        virtual ~PlatformMock() = default;

        MOCK_METHOD(hal::GpioPin&, StatusLed, (), (override));
        MOCK_METHOD(hal::SerialCommunication&, Communication, (), (override));
        MOCK_METHOD(services::Tracer&, Tracer, (), (override));
        MOCK_METHOD(MotorDriver&, Motors, (), (override));
        MOCK_METHOD(WheelEncoders&, Encoders, (), (override));
        MOCK_METHOD(InertialSensor&, Inertial, (), (override));
        MOCK_METHOD(void, StartBluetooth, (const infra::Function<void(Bluetooth& bluetooth)>& onReady), (override));
        MOCK_METHOD(void, Run, (), (override));
    };
}
