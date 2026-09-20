#pragma once

#include "infra/timer/Timer.hpp"
#include "infra/util/Function.hpp"

namespace platform
{
    struct InertialAxes
    {
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };
    };

    struct InertialSample
    {
        InertialAxes angularRate;
        InertialAxes acceleration;
        infra::TimePoint sampledAt;
        bool valid{ false };
    };

    class InertialSensor
    {
    public:
        InertialSensor() = default;
        InertialSensor(const InertialSensor& other) = delete;
        InertialSensor& operator=(const InertialSensor& other) = delete;

        virtual void Start(const infra::Function<void(const InertialSample&)>& onSample) = 0;
        virtual void Stop() = 0;

    protected:
        ~InertialSensor() = default;
    };
}
