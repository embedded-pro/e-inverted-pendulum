#pragma once

#include "core/attitude_estimation/interfaces/AttitudeEstimation.hpp"
#include "core/control_loop/interfaces/ControlLoop.hpp"
#include "core/control_loop/interfaces/Stages.hpp"
#include "core/inertial_sensing/interfaces/InertialSensing.hpp"
#include <chrono>
#include <optional>

namespace control
{
    class ControlLoopImpl final
        : public ControlLoop
    {
    public:
        struct Config
        {
            Config();

            std::chrono::microseconds samplePeriod{ 1000 };
            std::chrono::microseconds balancePeriod{ 2000 };
            std::chrono::microseconds outerPeriod{ 20000 };
            uint32_t latePercentage{ 10 };
        };

        ControlLoopImpl(sensing::InertialSensing& sensing, estimation::AttitudeEstimation& estimation, BalanceStage& balance, OuterStage& outer, const Config& config = Config());

        TimingStatistics Statistics() const override;
        void ResetStatistics() override;

    private:
        void OnMeasurement(const sensing::Measurement& measurement);
        void RunBalance(const estimation::Estimate& estimate, infra::TimePoint sampledAt);
        void RunOuter(infra::TimePoint sampledAt);
        void Observe(infra::Duration interval);

        estimation::AttitudeEstimation& estimation;
        BalanceStage& balance;
        OuterStage& outer;
        Config config;

        uint32_t samplesPerBalance;
        uint32_t balancesPerOuter;
        infra::Duration lateThreshold;

        uint32_t samplesSinceBalance{ 0 };
        uint32_t balancesSinceOuter{ 0 };
        std::optional<infra::TimePoint> previousBalance;
        std::optional<infra::TimePoint> previousOuter;
        TimingStatistics statistics;
    };
}
