#pragma once

#include "core/attitude_estimation/interfaces/AttitudeEstimation.hpp"
#include "core/wheel_odometry/interfaces/WheelOdometry.hpp"
#include "infra/util/MemoryRange.hpp"
#include <cstddef>

namespace balance
{
    struct Effort
    {
        float left{ 0.0f };
        float right{ 0.0f };

        bool operator==(const Effort& other) const = default;
    };

    struct Setpoints
    {
        float velocity{ 0.0f };
        float yawRate{ 0.0f };

        bool operator==(const Setpoints& other) const = default;
    };

    struct ParameterDescriptor
    {
        const char* name;
        float minimum;
        float maximum;
    };

    class ControlStrategy
    {
    public:
        ControlStrategy() = default;
        ControlStrategy(const ControlStrategy& other) = delete;
        ControlStrategy& operator=(const ControlStrategy& other) = delete;

        virtual const char* Name() const = 0;
        virtual void Reset() = 0;

        virtual void Steer(const Setpoints& setpoints, const odometry::ChassisMotion& chassis, float intervalSeconds) = 0;
        virtual Effort Balance(const estimation::Estimate& estimate, float intervalSeconds) = 0;
        virtual void Saturated(const Effort& applied) = 0;

        virtual infra::MemoryRange<const ParameterDescriptor> Parameters() const = 0;
        virtual float Parameter(std::size_t index) const = 0;
        virtual void SetParameter(std::size_t index, float value) = 0;

    protected:
        ~ControlStrategy() = default;
    };
}
