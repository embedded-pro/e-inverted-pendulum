#include "core/platform_abstraction/InertialFrame.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <array>
#include <numbers>

namespace platform
{
    namespace
    {
        bool IsUnitSign(float sign)
        {
            return sign == 1.0f || sign == -1.0f;
        }
    }

    bool IsValid(const AxisMap& map)
    {
        const bool inRange = map.xFrom < 3 && map.yFrom < 3 && map.zFrom < 3;
        const bool distinct = map.xFrom != map.yFrom && map.yFrom != map.zFrom && map.xFrom != map.zFrom;

        return inRange && distinct && IsUnitSign(map.xSign) && IsUnitSign(map.ySign) && IsUnitSign(map.zSign);
    }

    InertialAxes ToBodyFrame(const AxisMap& map, float first, float second, float third)
    {
        really_assert(IsValid(map));

        const std::array<float, 3> sensor{ { first, second, third } };

        return {
            map.xSign * sensor[map.xFrom],
            map.ySign * sensor[map.yFrom],
            map.zSign * sensor[map.zFrom]
        };
    }

    float MilliDegreePerSecondToRadianPerSecond(int32_t milliDegreePerSecond)
    {
        return static_cast<float>(milliDegreePerSecond) * std::numbers::pi_v<float> / 180000.0f;
    }

    float MilliMeterPerSecondSquaredToMeterPerSecondSquared(int32_t milliMeterPerSecondSquared)
    {
        return static_cast<float>(milliMeterPerSecondSquared) * 0.001f;
    }
}
