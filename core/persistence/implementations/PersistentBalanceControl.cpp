#include "core/persistence/implementations/PersistentBalanceControl.hpp"
#include "core/persistence/implementations/StoreStatus.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <algorithm>

namespace persistence
{
    namespace
    {
        services::NvmRecordStore::Config StoreConfig(const PersistentBalanceControl::Config& config)
        {
            services::NvmRecordStore::Config storeConfig;
            storeConfig.saveDelay = config.saveDelay;
            return storeConfig;
        }
    }

    PersistentBalanceControl::Config::Config() = default;

    PersistentBalanceControl::PersistentBalanceControl(balance::BalanceControl& control, hal::Eeprom& eeprom, infra::ClaimableResource& access, const Config& config)
        : control(control)
        , first(eeprom, 0, regionSize)
        , second(eeprom, regionSize, regionSize)
        , store(first, second, access, static_cast<services::NvmRecordStore::Client&>(*this), StoreConfig(config))
    {}

    std::size_t PersistentBalanceControl::StrategyCount() const
    {
        return control.StrategyCount();
    }

    const char* PersistentBalanceControl::StrategyName(std::size_t index) const
    {
        return control.StrategyName(index);
    }

    std::size_t PersistentBalanceControl::ActiveStrategy() const
    {
        return control.ActiveStrategy();
    }

    bool PersistentBalanceControl::Select(std::size_t index)
    {
        return Changed(control.Select(index));
    }

    infra::MemoryRange<const balance::ParameterDescriptor> PersistentBalanceControl::Parameters() const
    {
        return control.Parameters();
    }

    float PersistentBalanceControl::Parameter(std::size_t index) const
    {
        return control.Parameter(index);
    }

    bool PersistentBalanceControl::SetParameter(std::size_t index, float value)
    {
        return Changed(control.SetParameter(index, value));
    }

    infra::MemoryRange<const balance::ParameterDescriptor> PersistentBalanceControl::StrategyParameters(std::size_t strategy) const
    {
        return control.StrategyParameters(strategy);
    }

    float PersistentBalanceControl::StrategyParameter(std::size_t strategy, std::size_t index) const
    {
        return control.StrategyParameter(strategy, index);
    }

    bool PersistentBalanceControl::SetStrategyParameter(std::size_t strategy, std::size_t index, float value)
    {
        return Changed(control.SetStrategyParameter(strategy, index, value));
    }

    bool PersistentBalanceControl::Move(const balance::Setpoints& setpoints)
    {
        return control.Move(setpoints);
    }

    void PersistentBalanceControl::CancelMotion()
    {
        control.CancelMotion();
    }

    bool PersistentBalanceControl::Engaged() const
    {
        return control.Engaged();
    }

    balance::Effort PersistentBalanceControl::AppliedEffort() const
    {
        return control.AppliedEffort();
    }

    RestoreResult PersistentBalanceControl::Restore() const
    {
        return RestoreOf(store.Restore(), undecodable);
    }

    SaveResult PersistentBalanceControl::LastSave() const
    {
        return SaveOf(store.LastSave());
    }

    void PersistentBalanceControl::Restored(infra::ConstByteRange payload)
    {
        const auto record = Decode(payload);

        if (record.has_value())
            Apply(*record);
        else
            undecodable = true;
    }

    std::size_t PersistentBalanceControl::Capture(infra::ByteRange payload)
    {
        return Encode(Capture(), payload);
    }

    bool PersistentBalanceControl::Busy() const
    {
        return control.Engaged();
    }

    void PersistentBalanceControl::Apply(const TuningRecord& record)
    {
        for (std::size_t strategy = 0; strategy != control.StrategyCount(); ++strategy)
        {
            ApplyStrategy(record, strategy);

            if (KeyOf(control.StrategyName(strategy), "") == record.activeStrategy)
                control.Select(strategy);
        }
    }

    void PersistentBalanceControl::ApplyStrategy(const TuningRecord& record, std::size_t strategy)
    {
        const auto name = control.StrategyName(strategy);
        const auto descriptors = control.StrategyParameters(strategy);

        for (std::size_t index = 0; index != descriptors.size(); ++index)
        {
            const auto key = KeyOf(name, descriptors[index].name);
            const auto entry = std::ranges::find_if(record.entries, [key](const TuningEntry& candidate)
                {
                    return candidate.key == key;
                });

            if (entry != record.entries.end())
                control.SetStrategyParameter(strategy, index, entry->value);
        }
    }

    bool PersistentBalanceControl::Changed(bool accepted)
    {
        if (accepted)
            store.ScheduleSave();

        return accepted;
    }

    TuningRecord PersistentBalanceControl::Capture() const
    {
        TuningRecord record;
        record.activeStrategy = KeyOf(control.StrategyName(control.ActiveStrategy()), "");

        for (std::size_t strategy = 0; strategy != control.StrategyCount(); ++strategy)
        {
            const auto name = control.StrategyName(strategy);
            const auto descriptors = control.StrategyParameters(strategy);

            for (std::size_t index = 0; index != descriptors.size(); ++index)
            {
                really_assert(!record.entries.full());
                record.entries.push_back({ KeyOf(name, descriptors[index].name), control.StrategyParameter(strategy, index) });
            }
        }

        return record;
    }
}
