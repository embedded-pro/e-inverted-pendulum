#pragma once

#include "core/balance_control/interfaces/BalanceControl.hpp"
#include "core/control_loop/interfaces/Stages.hpp"
#include "core/motion_actuation/interfaces/MotionActuation.hpp"
#include "core/wheel_odometry/interfaces/WheelOdometry.hpp"
#include "infra/timer/Timer.hpp"
#include <chrono>

namespace balance
{
    class BalanceControlImpl final
        : public BalanceControl
        , public control::SupervisedControl
    {
    public:
        struct Config
        {
            Config();

            float maximumVelocity{ 1.0f };
            float maximumYawRate{ 1.6f };
            std::chrono::milliseconds setpointHold{ 1000 };
        };

        BalanceControlImpl(infra::MemoryRange<ControlStrategy* const> strategies, motion::MotionActuation& actuation, odometry::WheelOdometry& odometry, const Config& config = Config());

        std::size_t StrategyCount() const override;
        const char* StrategyName(std::size_t index) const override;
        std::size_t ActiveStrategy() const override;
        bool Select(std::size_t index) override;

        infra::MemoryRange<const ParameterDescriptor> Parameters() const override;
        float Parameter(std::size_t index) const override;
        bool SetParameter(std::size_t index, float value) override;

        bool Move(const Setpoints& setpoints) override;
        bool Engaged() const override;

        void Balance(const estimation::Estimate& estimate, infra::Duration interval) override;
        void Steer(infra::Duration interval) override;
        void Engage() override;
        void Disengage() override;

    private:
        ControlStrategy& Active() const;
        Setpoints CurrentSetpoints() const;

        infra::MemoryRange<ControlStrategy* const> strategies;
        motion::MotionActuation& actuation;
        odometry::WheelOdometry& odometry;
        Config config;

        std::size_t active{ 0 };
        bool engaged{ false };
        Setpoints setpoints;
        infra::TimePoint commandedAt;
    };
}
