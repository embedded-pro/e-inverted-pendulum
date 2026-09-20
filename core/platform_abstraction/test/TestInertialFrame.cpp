#include "core/platform_abstraction/InertialFrame.hpp"
#include "gtest/gtest.h"
#include <numbers>

TEST(InertialFrameTest, the_identity_map_passes_every_axis_through)
{
    const auto axes = platform::ToBodyFrame(platform::AxisMap{}, 1.0f, 2.0f, 3.0f);

    EXPECT_FLOAT_EQ(1.0f, axes.x);
    EXPECT_FLOAT_EQ(2.0f, axes.y);
    EXPECT_FLOAT_EQ(3.0f, axes.z);
}

TEST(InertialFrameTest, a_permutation_reorders_the_axes)
{
    platform::AxisMap map;
    map.xFrom = 1;
    map.yFrom = 2;
    map.zFrom = 0;

    const auto axes = platform::ToBodyFrame(map, 1.0f, 2.0f, 3.0f);

    EXPECT_FLOAT_EQ(2.0f, axes.x);
    EXPECT_FLOAT_EQ(3.0f, axes.y);
    EXPECT_FLOAT_EQ(1.0f, axes.z);
}

TEST(InertialFrameTest, signs_invert_individual_axes)
{
    platform::AxisMap map;
    map.xSign = -1.0f;
    map.zSign = -1.0f;

    const auto axes = platform::ToBodyFrame(map, 1.0f, 2.0f, 3.0f);

    EXPECT_FLOAT_EQ(-1.0f, axes.x);
    EXPECT_FLOAT_EQ(2.0f, axes.y);
    EXPECT_FLOAT_EQ(-3.0f, axes.z);
}

TEST(InertialFrameTest, a_mounting_that_is_upside_down_reads_gravity_the_other_way)
{
    platform::AxisMap map;
    map.ySign = -1.0f;
    map.zSign = -1.0f;

    const auto axes = platform::ToBodyFrame(map, 0.0f, 0.0f, 9.80665f);

    EXPECT_FLOAT_EQ(-9.80665f, axes.z);
}

TEST(InertialFrameTest, a_full_scale_rate_converts_to_radians_per_second)
{
    EXPECT_NEAR(std::numbers::pi_v<float>, platform::MilliDegreePerSecondToRadianPerSecond(180000), 1e-5f);
    EXPECT_NEAR(-std::numbers::pi_v<float>, platform::MilliDegreePerSecondToRadianPerSecond(-180000), 1e-5f);
    EXPECT_FLOAT_EQ(0.0f, platform::MilliDegreePerSecondToRadianPerSecond(0));
}

TEST(InertialFrameTest, gravity_converts_from_millimetres_to_metres_per_second_squared)
{
    EXPECT_NEAR(9.80665f, platform::MilliMeterPerSecondSquaredToMeterPerSecondSquared(9806), 1e-3f);
    EXPECT_NEAR(-9.80665f, platform::MilliMeterPerSecondSquaredToMeterPerSecondSquared(-9806), 1e-3f);
}
