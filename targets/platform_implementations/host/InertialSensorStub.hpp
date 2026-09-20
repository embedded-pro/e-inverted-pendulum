#pragma once

#include "core/platform_abstraction/InertialSensor.hpp"
#include "infra/timer/Timer.hpp"

namespace application
{
    class InertialSensorStub final
        : public platform::InertialSensor
    {
    public:
        void Start(const infra::Function<void(const platform::InertialSample&)>& onSample) override
        {
            this->onSample = onSample;

            sampleTimer.Start(samplePeriod, [this]()
                {
                    Emit();
                });
        }

        void Stop() override
        {
            sampleTimer.Cancel();
            onSample = nullptr;
        }

        void SetUpright(bool upright)
        {
            this->upright = upright;
        }

    private:
        void Emit() const
        {
            platform::InertialSample sample;

            sample.angularRate = { 0.0f, 0.0f, 0.0f };
            sample.acceleration = { 0.0f, 0.0f, upright ? -gravity : gravity };
            sample.sampledAt = infra::Now();
            sample.valid = true;

            if (onSample)
                onSample(sample);
        }

        static constexpr float gravity{ 9.80665f };
        static constexpr std::chrono::microseconds samplePeriod{ 1000 };

        infra::Function<void(const platform::InertialSample&)> onSample;
        infra::TimerRepeating sampleTimer;
        bool upright{ true };
    };
}
