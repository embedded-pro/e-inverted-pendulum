#pragma once

#include "core/platform_abstraction/InertialSensor.hpp"
#include "gmock/gmock.h"

namespace platform
{
    class InertialSensorMock
        : public InertialSensor
    {
    public:
        virtual ~InertialSensorMock() = default;

        MOCK_METHOD(void, Start, (const infra::Function<void(const InertialSample&)>& onSample), (override));
        MOCK_METHOD(void, Stop, (), (override));
    };
}
