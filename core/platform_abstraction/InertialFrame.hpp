#pragma once

#include "core/platform_abstraction/InertialSensor.hpp"
#include <cstdint>

namespace platform
{
    struct AxisMap
    {
        uint8_t xFrom{ 0 };
        uint8_t yFrom{ 1 };
        uint8_t zFrom{ 2 };

        float xSign{ 1.0f };
        float ySign{ 1.0f };
        float zSign{ 1.0f };
    };

    InertialAxes ToBodyFrame(const AxisMap& map, float first, float second, float third);

    float MilliDegreePerSecondToRadianPerSecond(int32_t milliDegreePerSecond);
    float MilliMeterPerSecondSquaredToMeterPerSecondSquared(int32_t milliMeterPerSecondSquared);
}
