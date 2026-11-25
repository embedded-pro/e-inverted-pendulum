#pragma once

#include "core/platform_abstraction/WheelEncoders.hpp"
#include "core/wheel_odometry/interfaces/WheelOdometry.hpp"
#include "infra/timer/Timer.hpp"
#include <chrono>

namespace odometry
{
    class WheelOdometryImpl final
        : public WheelOdometry
    {
    public:
        struct Config
        {
            Config();

            std::chrono::microseconds samplePeriod{ 20000 };
            Geometry geometry;
        };

        explicit WheelOdometryImpl(platform::WheelEncoders& encoders, const Config& config = Config());

        WheelMotion Left() const override;
        WheelMotion Right() const override;
        ChassisMotion Chassis() const override;

        void SetEncoderInverted(WheelSide side, bool inverted) override;
        bool EncoderInverted(WheelSide side) const override;
        bool SetGeometry(const Geometry& geometry) override;
        Geometry CurrentGeometry() const override;

    private:
        struct Wheel
        {
            uint32_t previousRaw{ 0 };
            bool sampled{ false };
            bool inverted{ false };
            WheelMotion motion;
        };

        void Sample();
        void SampleWheel(Wheel& wheel, hal::SynchronousQuadratureEncoder& encoder, infra::Duration elapsed) const;
        Wheel& WheelOn(WheelSide side);
        const Wheel& WheelOn(WheelSide side) const;
        static int32_t ShortestDelta(uint32_t previous, uint32_t current, uint32_t resolution);

        platform::WheelEncoders& encoders;
        Config config;
        Wheel left;
        Wheel right;
        infra::TimePoint previousSampleTime;
        infra::TimerRepeating sampleTimer;
    };
}
