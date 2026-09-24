#include "core/control_loop/implementations/ControlLoopImpl.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <algorithm>

namespace control
{
    ControlLoopImpl::Config::Config() = default;

    ControlLoopImpl::ControlLoopImpl(sensing::InertialSensing& sensing, estimation::AttitudeEstimation& estimation, BalanceStage& balance, OuterStage& outer, const Config& config)
        : estimation(estimation)
        , balance(balance)
        , outer(outer)
        , config(config)
        , samplesPerBalance(static_cast<uint32_t>(config.balancePeriod / config.samplePeriod))
        , balancesPerOuter(static_cast<uint32_t>(config.outerPeriod / config.balancePeriod))
        , lateThreshold(config.balancePeriod + config.balancePeriod * config.latePercentage / 100)
    {
        really_assert(config.samplePeriod.count() > 0);
        really_assert(samplesPerBalance > 0 && config.balancePeriod % config.samplePeriod == std::chrono::microseconds::zero());
        really_assert(balancesPerOuter > 0 && config.outerPeriod % config.balancePeriod == std::chrono::microseconds::zero());

        sensing.OnMeasurement([this](const sensing::Measurement& measurement)
            {
                OnMeasurement(measurement);
            });
    }

    TimingStatistics ControlLoopImpl::Statistics() const
    {
        return statistics;
    }

    void ControlLoopImpl::ResetStatistics()
    {
        statistics = TimingStatistics{};
    }

    void ControlLoopImpl::OnMeasurement(const sensing::Measurement& measurement)
    {
        const auto estimate = estimation.Update(measurement);

        if (++samplesSinceBalance < samplesPerBalance)
            return;

        samplesSinceBalance = 0;
        RunBalance(estimate, measurement.sampledAt);

        if (++balancesSinceOuter < balancesPerOuter)
            return;

        balancesSinceOuter = 0;
        RunOuter(measurement.sampledAt);
    }

    void ControlLoopImpl::RunBalance(const estimation::Estimate& estimate, infra::TimePoint sampledAt)
    {
        infra::Duration interval{ config.balancePeriod };

        if (previousBalance)
        {
            interval = sampledAt - *previousBalance;
            Observe(interval);
        }

        previousBalance = sampledAt;
        ++statistics.iterations;
        balance.Balance(estimate, interval);
    }

    void ControlLoopImpl::RunOuter(infra::TimePoint sampledAt)
    {
        infra::Duration interval{ config.outerPeriod };

        if (previousOuter)
            interval = sampledAt - *previousOuter;

        previousOuter = sampledAt;
        outer.Steer(interval);
    }

    void ControlLoopImpl::Observe(infra::Duration interval)
    {
        const auto deviation = interval > infra::Duration{ config.balancePeriod } ? interval - config.balancePeriod : config.balancePeriod - interval;
        statistics.worstJitter = std::max(statistics.worstJitter, deviation);

        if (interval > lateThreshold)
            ++statistics.lateIterations;
    }
}
