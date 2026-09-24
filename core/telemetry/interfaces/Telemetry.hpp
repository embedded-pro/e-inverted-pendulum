#pragma once

#include "core/safety_supervisor/interfaces/SafetySupervisor.hpp"

namespace telemetry
{
    struct Sample
    {
        float pitch{ 0.0f };
        float pitchRate{ 0.0f };
        float forwardVelocity{ 0.0f };
        float yawRate{ 0.0f };
        float effortLeft{ 0.0f };
        float effortRight{ 0.0f };
        safety::Mode mode{ safety::Mode::init };
        safety::FaultCause cause{ safety::FaultCause::none };

        bool operator==(const Sample& other) const = default;
    };

    class TelemetrySource
    {
    public:
        TelemetrySource() = default;
        TelemetrySource(const TelemetrySource& other) = delete;
        TelemetrySource& operator=(const TelemetrySource& other) = delete;

        virtual Sample Latest() const = 0;

    protected:
        ~TelemetrySource() = default;
    };
}
