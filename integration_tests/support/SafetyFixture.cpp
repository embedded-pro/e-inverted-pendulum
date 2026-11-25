#include "integration_tests/support/SafetyFixture.hpp"
#include <chrono>
#include <cmath>
#include <numbers>

namespace integration
{
    namespace
    {
        constexpr std::chrono::milliseconds step{ 1 };
        constexpr std::chrono::seconds startupLimit{ 5 };
        constexpr std::chrono::milliseconds settle{ 100 };
        constexpr std::chrono::seconds convergence{ 1 };
        constexpr float fallThresholdDegrees{ 35.0f };
        constexpr float tipRate{ 3.0f };

        float Radians(float degrees)
        {
            return degrees * std::numbers::pi_v<float> / 180.0f;
        }
    }

    void SafetyFixture::Forward(infra::Duration duration)
    {
        clock.ForwardTime(duration);
    }

    std::optional<infra::Duration> SafetyFixture::ForwardUntil(const infra::Function<bool()>& condition, infra::Duration limit)
    {
        const auto start = infra::Now();

        while (!condition())
        {
            if (infra::Now() - start >= limit)
                return std::nullopt;

            clock.ForwardTime(step);
        }

        return infra::Now() - start;
    }

    bool SafetyFixture::ModeHolds(safety::Mode mode, infra::Duration duration)
    {
        return !ForwardUntil([this, mode]()
            {
                return supervisor.Current() != mode;
            },
            duration);
    }

    void SafetyFixture::Calibrate()
    {
        ForwardUntil([this]()
            {
                return supervisor.Current() == safety::Mode::idle;
            },
            startupLimit);
        Forward(convergence);
    }

    void SafetyFixture::Arm()
    {
        supervisor.Arm();
        Forward(settle);
    }

    void SafetyFixture::TipTo(float degrees)
    {
        expectedCause = safety::FaultCause::fall;
        inertial.TiltTo(Radians(degrees), tipRate);
        const auto crossed = ForwardUntil([this]()
            {
                return std::fabs(inertial.Pitch()) > Radians(fallThresholdDegrees);
            },
            startupLimit);
        thresholdCrossed = crossed ? std::optional(infra::Now()) : std::nullopt;
    }
}
