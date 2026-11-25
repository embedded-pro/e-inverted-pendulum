#include "integration_tests/support/InertialSensorFake.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>

namespace integration
{
    namespace
    {
        constexpr float gravity{ 9.80665f };
        constexpr std::chrono::microseconds samplePeriod{ 1000 };
        constexpr float sampleSeconds{ 0.001f };
    }

    void InertialSensorFake::Start(const infra::Function<void(const platform::InertialSample&)>& onSample)
    {
        this->onSample = onSample;
        sampleTimer.Start(samplePeriod, [this]()
            {
                Emit();
            });
    }

    void InertialSensorFake::Stop()
    {
        sampleTimer.Cancel();
        onSample = nullptr;
    }

    void InertialSensorFake::TiltTo(float pitch, float rate)
    {
        targetPitch = pitch;
        slewRate = std::fabs(rate);
    }

    void InertialSensorFake::Silence()
    {
        silent = true;
    }

    void InertialSensorFake::FailTransfers()
    {
        failing = true;
    }

    float InertialSensorFake::Pitch() const
    {
        return pitch;
    }

    void InertialSensorFake::Advance()
    {
        const auto step = std::clamp(targetPitch - pitch, -slewRate * sampleSeconds, slewRate * sampleSeconds);
        pitch += step;
        pitchRate = step / sampleSeconds;
    }

    void InertialSensorFake::Emit()
    {
        Advance();

        if (silent || !onSample)
            return;

        platform::InertialSample sample;
        sample.angularRate = { 0.0f, -pitchRate, 0.0f };
        sample.acceleration = { -gravity * std::sin(pitch), 0.0f, -gravity * std::cos(pitch) };
        sample.sampledAt = infra::Now();
        sample.valid = !failing;

        onSample(sample);
    }
}
