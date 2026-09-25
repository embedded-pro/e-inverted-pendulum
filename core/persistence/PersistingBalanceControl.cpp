#include "core/persistence/PersistingBalanceControl.hpp"
#include <algorithm>
#include <bit>
#include <optional>

namespace persistence
{
    namespace
    {
        template<class BoundedName>
        void AssignName(BoundedName& target, const char* name)
        {
            const infra::BoundedConstString source{ name };
            target.assign(source.substr(0, std::min(source.size(), target.max_size())));
        }

        bool NameMatches(infra::BoundedConstString stored, const char* name)
        {
            return stored == infra::BoundedConstString{ name };
        }
    }

    PersistingBalanceControl::Config::Config() = default;

    PersistingBalanceControl::PersistingBalanceControl(balance::BalanceControl& control, const safety::SafetySupervisor& supervisor, services::ConfigurationStoreAccess<Tuning> store, const Config& config)
        : control(control)
        , supervisor(supervisor)
        , store(store)
        , config(config)
    {}

    void PersistingBalanceControl::Restore()
    {
        for (const auto& stored : store->strategies)
            RestoreStrategy(stored);

        if (const auto active = StrategyNamed(store->activeStrategy))
            control.Select(*active);
    }

    std::size_t PersistingBalanceControl::StrategyCount() const
    {
        return control.StrategyCount();
    }

    const char* PersistingBalanceControl::StrategyName(std::size_t index) const
    {
        return control.StrategyName(index);
    }

    std::size_t PersistingBalanceControl::ActiveStrategy() const
    {
        return control.ActiveStrategy();
    }

    bool PersistingBalanceControl::Select(std::size_t index)
    {
        return ScheduleSaveIf(control.Select(index));
    }

    infra::MemoryRange<const balance::ParameterDescriptor> PersistingBalanceControl::Parameters() const
    {
        return control.Parameters();
    }

    float PersistingBalanceControl::Parameter(std::size_t index) const
    {
        return control.Parameter(index);
    }

    bool PersistingBalanceControl::SetParameter(std::size_t index, float value)
    {
        return ScheduleSaveIf(control.SetParameter(index, value));
    }

    infra::MemoryRange<const balance::ParameterDescriptor> PersistingBalanceControl::StrategyParameters(std::size_t strategy) const
    {
        return control.StrategyParameters(strategy);
    }

    float PersistingBalanceControl::StrategyParameter(std::size_t strategy, std::size_t index) const
    {
        return control.StrategyParameter(strategy, index);
    }

    bool PersistingBalanceControl::SetStrategyParameter(std::size_t strategy, std::size_t index, float value)
    {
        return ScheduleSaveIf(control.SetStrategyParameter(strategy, index, value));
    }

    bool PersistingBalanceControl::Move(const balance::Setpoints& setpoints)
    {
        return control.Move(setpoints);
    }

    void PersistingBalanceControl::CancelMotion()
    {
        control.CancelMotion();
    }

    bool PersistingBalanceControl::Engaged() const
    {
        return control.Engaged();
    }

    balance::Effort PersistingBalanceControl::AppliedEffort() const
    {
        return control.AppliedEffort();
    }

    bool PersistingBalanceControl::ScheduleSaveIf(bool accepted)
    {
        if (accepted)
            saveTimer.Start(config.saveDelay, [this]()
                {
                    SaveUnlessArmed();
                });

        return accepted;
    }

    void PersistingBalanceControl::SaveUnlessArmed()
    {
        if (supervisor.Current() == safety::Mode::armed)
        {
            ScheduleSaveIf(true);
            return;
        }

        Capture(*store);
        store.Write();
    }

    void PersistingBalanceControl::Capture(Tuning& tuning) const
    {
        AssignName(tuning.activeStrategy, control.StrategyName(control.ActiveStrategy()));
        tuning.strategies.clear();

        for (std::size_t strategy = 0; strategy != control.StrategyCount() && !tuning.strategies.full(); ++strategy)
        {
            tuning.strategies.emplace_back();
            auto& stored = tuning.strategies.back();
            AssignName(stored.name, control.StrategyName(strategy));

            const auto descriptors = control.StrategyParameters(strategy);
            for (std::size_t index = 0; index != descriptors.size() && !stored.parameters.full(); ++index)
            {
                stored.parameters.emplace_back();
                auto& parameter = stored.parameters.back();
                AssignName(parameter.name, descriptors[index].name);
                parameter.value = std::bit_cast<uint32_t>(control.StrategyParameter(strategy, index));
            }
        }
    }

    void PersistingBalanceControl::RestoreStrategy(const StoredStrategy& stored)
    {
        const auto strategy = StrategyNamed(stored.name);
        if (!strategy)
            return;

        const auto descriptors = control.StrategyParameters(*strategy);
        for (const auto& parameter : stored.parameters)
            for (std::size_t index = 0; index != descriptors.size(); ++index)
                if (NameMatches(parameter.name, descriptors[index].name))
                    control.SetStrategyParameter(*strategy, index, std::bit_cast<float>(parameter.value));
    }

    std::optional<std::size_t> PersistingBalanceControl::StrategyNamed(infra::BoundedConstString name) const
    {
        for (std::size_t strategy = 0; strategy != control.StrategyCount(); ++strategy)
            if (NameMatches(name, control.StrategyName(strategy)))
                return strategy;

        return std::nullopt;
    }
}
