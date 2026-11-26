#pragma once

#include <cstdint>

namespace odometry
{
    struct WheelMotion
    {
        int32_t position{ 0 };
        float angularVelocity{ 0.0f };
    };

    struct ChassisMotion
    {
        float forwardVelocity{ 0.0f };
        float yawRate{ 0.0f };
    };

    enum class WheelSide : uint8_t
    {
        left,
        right
    };

    struct Geometry
    {
        float wheelRadius{ 0.034f };
        float trackWidth{ 0.145f };
        float gearRatio{ 1.0f };
    };

    class WheelOdometry
    {
    public:
        WheelOdometry() = default;
        WheelOdometry(const WheelOdometry& other) = delete;
        WheelOdometry& operator=(const WheelOdometry& other) = delete;

        virtual WheelMotion Left() const = 0;
        virtual WheelMotion Right() const = 0;
        virtual ChassisMotion Chassis() const = 0;

        virtual void SetEncoderInverted(WheelSide side, bool inverted) = 0;
        virtual bool EncoderInverted(WheelSide side) const = 0;
        virtual bool SetGeometry(const Geometry& geometry) = 0;
        virtual Geometry CurrentGeometry() const = 0;

    protected:
        ~WheelOdometry() = default;
    };
}
