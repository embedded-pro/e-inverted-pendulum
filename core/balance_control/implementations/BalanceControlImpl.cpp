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
        return Active().Parameters();
    }

    float BalanceControlImpl::Parameter(std::size_t index) const
    {
        return Active().Parameter(index);
    }

    bool BalanceControlImpl::SetParameter(std::size_t index, float value)
    {
        const auto descriptors = Active().Parameters();

        if (engaged || index >= descriptors.size() || value < descriptors[index].minimum || value > descriptors[index].maximum)
            return false;

        Active().SetParameter(index, value);
        return true;
    }

    bool BalanceControlImpl::Move(const Setpoints& commanded)
    {
        if (!engaged || std::fabs(commanded.velocity) > config.maximumVelocity || std::fabs(commanded.yawRate) > config.maximumYawRate)
            return false;

        setpoints = commanded;
        commandedAt = infra::Now();
        return true;
    }

    bool BalanceControlImpl::Engaged() const
    {
        return engaged;
    }

    void BalanceControlImpl::Balance(const estimation::Estimate& estimate, infra::Duration interval)
    {
        if (!engaged)
            return;

        if (!estimate.valid)
        {
            actuation.Apply(0.0f, 0.0f);
            return;
        }

        const auto requested = Active().Balance(estimate, Seconds(interval));
        const auto applied = Saturate(requested);

        if (applied != requested)
            Active().Saturated(applied);

        actuation.Apply(applied.left, applied.right);
    }

    void BalanceControlImpl::Steer(infra::Duration interval)
    {
        if (engaged)
            Active().Steer(CurrentSetpoints(), odometry.Chassis(), Seconds(interval));
    }

    void BalanceControlImpl::Engage()
    {
        setpoints = Setpoints{};
        Active().Reset();
        engaged = true;
    }

    void BalanceControlImpl::Disengage()
    {
        engaged = false;
        setpoints = Setpoints{};
    }

    ControlStrategy& BalanceControlImpl::Active() const
    {
        return *strategies[active];
    }

    Setpoints BalanceControlImpl::CurrentSetpoints() const
    {
        if (infra::Now() - commandedAt > config.setpointHold)
            return Setpoints{};

        return setpoints;
    }
}
