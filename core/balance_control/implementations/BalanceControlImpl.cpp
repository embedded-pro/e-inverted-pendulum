#include "core/balance_control/implementations/BalanceControlImpl.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <algorithm>
#include <cmath>

namespace balance
{
    namespace
    {
        constexpr float minimumIntervalSeconds{ 1.0e-4f };

        float Seconds(infra::Duration interval)
        {
            return std::max(std::chrono::duration<float>(interval).count(), minimumIntervalSeconds);
        }

        Effort Saturate(const Effort& effort)
        {
            return Effort{ std::clamp(effort.left, -1.0f, 1.0f), std::clamp(effort.right, -1.0f, 1.0f) };
        }

        float TowardsZero(float value, float step)
        {
            if (std::fabs(value) <= step)
                return 0.0f;

            return value > 0.0f ? value - step : value + step;
        }
    }

    BalanceControlImpl::Config::Config() = default;

    BalanceControlImpl::BalanceControlImpl(infra::MemoryRange<ControlStrategy* const> strategies, motion::MotionActuation& actuation, odometry::WheelOdometry& odometry, const Config& config)
        : strategies(strategies)
        , actuation(actuation)
        , odometry(odometry)
        , config(config)
    {
        really_assert(!strategies.empty());
    }

    std::size_t BalanceControlImpl::StrategyCount() const
    {
        return strategies.size();
    }

    const char* BalanceControlImpl::StrategyName(std::size_t index) const
    {
        really_assert(index < strategies.size());
        return strategies[index]->Name();
    }

    std::size_t BalanceControlImpl::ActiveStrategy() const
    {
        return active;
    }

    bool BalanceControlImpl::Select(std::size_t index)
    {
        if (engaged || index >= strategies.size())
            return false;

        active = index;
        return true;
    }

    infra::MemoryRange<const ParameterDescriptor> BalanceControlImpl::Parameters() const
    {
        return StrategyParameters(active);
    }

    float BalanceControlImpl::Parameter(std::size_t index) const
    {
        return StrategyParameter(active, index);
    }

    bool BalanceControlImpl::SetParameter(std::size_t index, float value)
    {
        return SetStrategyParameter(active, index, value);
    }

    infra::MemoryRange<const ParameterDescriptor> BalanceControlImpl::StrategyParameters(std::size_t strategy) const
    {
        if (strategy >= strategies.size())
            return {};

        return strategies[strategy]->Parameters();
    }

    float BalanceControlImpl::StrategyParameter(std::size_t strategy, std::size_t index) const
    {
        return strategies[strategy]->Parameter(index);
    }

    bool BalanceControlImpl::SetStrategyParameter(std::size_t strategy, std::size_t index, float value)
    {
        if (engaged || strategy >= strategies.size())
            return false;

        const auto descriptors = strategies[strategy]->Parameters();

        if (index >= descriptors.size() || !(value >= descriptors[index].minimum && value <= descriptors[index].maximum))
            return false;

        strategies[strategy]->SetParameter(index, value);
        return true;
    }

    bool BalanceControlImpl::Move(const Setpoints& commanded)
    {
        if (!engaged || !(std::fabs(commanded.velocity) <= config.maximumVelocity) || !(std::fabs(commanded.yawRate) <= config.maximumYawRate))
            return false;

        setpoints = commanded;
        commandedAt = infra::Now();
        cancelled = false;
        return true;
    }

    void BalanceControlImpl::CancelMotion()
    {
        cancelled = true;
    }

    bool BalanceControlImpl::Engaged() const
    {
        return engaged;
    }

    Effort BalanceControlImpl::AppliedEffort() const
    {
        return applied;
    }

    void BalanceControlImpl::Balance(const estimation::Estimate& estimate, infra::Duration interval)
    {
        if (!engaged)
            return;

        if (!estimate.valid)
        {
            applied = Effort{};
            actuation.Apply(0.0f, 0.0f);
            return;
        }

        const auto requested = Active().Balance(estimate, Seconds(interval));
        applied = Saturate(requested);

        if (applied != requested)
            Active().Saturated(applied);

        actuation.Apply(applied.left, applied.right);
    }

    void BalanceControlImpl::Steer(infra::Duration interval)
    {
        if (!engaged)
            return;

        const auto intervalSeconds = Seconds(interval);
        DecayUnlessCommanded(intervalSeconds);
        Active().Steer(setpoints, odometry.Chassis(), intervalSeconds);
    }

    void BalanceControlImpl::Engage()
    {
        setpoints = Setpoints{};
        cancelled = true;
        Active().Reset();
        engaged = true;
    }

    void BalanceControlImpl::Disengage()
    {
        engaged = false;
        setpoints = Setpoints{};
        cancelled = true;
        applied = Effort{};
    }

    ControlStrategy& BalanceControlImpl::Active() const
    {
        return *strategies[active];
    }

    void BalanceControlImpl::DecayUnlessCommanded(float intervalSeconds)
    {
        if (!cancelled && infra::Now() - commandedAt <= config.commandTimeout)
            return;

        setpoints.velocity = TowardsZero(setpoints.velocity, config.velocityDeceleration * intervalSeconds);
        setpoints.yawRate = TowardsZero(setpoints.yawRate, config.yawDeceleration * intervalSeconds);
    }
}
