#pragma once

#include <cstdint>

namespace motion
{
    enum class DisableState : uint8_t
    {
        tristate,
        brake
    };

    enum class FaultCause : uint8_t
    {
        none,
        driverFault
    };

    enum class WheelSide : uint8_t
    {
        left,
        right
    };

    enum class DriverState : uint8_t
    {
        configuring,
        ready,
        failed
    };

    class MotionActuation
    {
    public:
        MotionActuation() = default;
        MotionActuation(const MotionActuation& other) = delete;
        MotionActuation& operator=(const MotionActuation& other) = delete;

        virtual void Apply(float effortLeft, float effortRight) = 0;
        virtual void Disable(DisableState state) = 0;
        virtual FaultCause Fault() const = 0;
        virtual void ClearFault() = 0;
        virtual DriverState State() const = 0;

        virtual void SetMotorInverted(WheelSide side, bool inverted) = 0;
        virtual bool MotorInverted(WheelSide side) const = 0;

    protected:
        ~MotionActuation() = default;
    };
}
