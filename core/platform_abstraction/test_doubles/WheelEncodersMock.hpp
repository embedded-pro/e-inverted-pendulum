#pragma once

#include "core/platform_abstraction/WheelEncoders.hpp"
#include "gmock/gmock.h"

namespace platform
{
    class WheelEncodersMock
        : public WheelEncoders
    {
    public:
        MOCK_METHOD(hal::SynchronousQuadratureEncoder&, Left, (), (override));
        MOCK_METHOD(hal::SynchronousQuadratureEncoder&, Right, (), (override));
    };
}
