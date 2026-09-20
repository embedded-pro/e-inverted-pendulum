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

    InertialAxes ToBodyFrame(const AxisMap& map, float first, float second, float third)
    {
        really_assert(map.xFrom < 3 && map.yFrom < 3 && map.zFrom < 3);
        really_assert(map.xFrom != map.yFrom && map.yFrom != map.zFrom && map.xFrom != map.zFrom);
        really_assert(IsUnitSign(map.xSign) && IsUnitSign(map.ySign) && IsUnitSign(map.zSign));

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
