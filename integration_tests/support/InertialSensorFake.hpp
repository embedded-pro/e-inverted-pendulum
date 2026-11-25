#pragma once

#include "core/platform_abstraction/InertialSensor.hpp"
#include "infra/timer/Timer.hpp"

namespace integration
{
    class InertialSensorFake final
        : public platform::InertialSensor
    {
    public:
        void Start(const infra::Function<void(const platform::InertialSample&)>& onSample) override;
        void Stop() override;

        void TiltTo(float pitch, float rate);
        void Silence();
        void FailTransfers();

        float Pitch() const;

    private:
        void Emit();
        void Advance();

        infra::Function<void(const platform::InertialSample&)> onSample;
        infra::TimerRepeating sampleTimer;
        float pitch{ 0.0f };
        float targetPitch{ 0.0f };
        float pitchRate{ 0.0f };
        float slewRate{ 0.0f };
        bool silent{ false };
        bool failing{ false };
    };
}
