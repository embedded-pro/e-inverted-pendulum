#pragma once

#include "core/motion_actuation/interfaces/MotionActuation.hpp"
#include "core/platform_abstraction/InertialFrame.hpp"
#include "core/wheel_odometry/interfaces/WheelOdometry.hpp"
#include <cstdint>

namespace calibration
{
    enum class Result : uint8_t
    {
        accepted,
        armed,
        invalid,
        restoring
    };

    class Calibration
    {
    public:
        Calibration() = default;
        Calibration(const Calibration& other) = delete;
        Calibration& operator=(const Calibration& other) = delete;

        virtual Result SetAxisMap(const platform::AxisMap& map) = 0;
        virtual Result SetStillnessThreshold(float threshold) = 0;
        virtual Result SetUprightOffset(float offset) = 0;
        virtual Result CaptureUpright() = 0;
        virtual Result SetMotorInverted(motion::WheelSide side, bool inverted) = 0;
        virtual Result SetEncoderInverted(odometry::WheelSide side, bool inverted) = 0;
        virtual Result SetGeometry(const odometry::Geometry& geometry) = 0;

    protected:
        ~Calibration() = default;
    };
}
