#include "core/persistence/implementations/PersistentBalanceControl.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <algorithm>

namespace persistence
{
    namespace
    {
        constexpr uint8_t erasedByte{ 0xff };

        bool Newer(uint16_t candidate, uint16_t reference)
        {
            return static_cast<int16_t>(candidate - reference) > 0;
        }
    }

    PersistentBalanceControl::Config::Config() = default;

    PersistentBalanceControl::PersistentBalanceControl(balance::BalanceControl& control, hal::Eeprom& eeprom, const Config& config)
        : control(control)
        , config(config)
        , regions{ { { eeprom, 0, regionSize }, { eeprom, regionSize, regionSize } } }
        , images{ { services::NvmImage<regionSize>{ regions[0] }, services::NvmImage<regionSize>{ regions[1] } } }
    {
        ReadCopy(0);
    }

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
        return restore;
    }

    SaveResult PersistentBalanceControl::LastSave() const
    {
        return lastSave;
    }

    void PersistentBalanceControl::ReadCopy(std::size_t copy)
    {
        reading = copy;
        images[copy].Read([this](infra::ConstByteRange data)
            {
                CopyRead(data);
            });
    }

    void PersistentBalanceControl::CopyRead(infra::ConstByteRange data)
    {
        copies[reading] = Decode(data);
        erased[reading] = std::all_of(data.begin(), data.end(), [](uint8_t byte)
            {
                return byte == erasedByte;
            });

        if (reading == 0)
            ReadCopy(1);
        else
            CopiesRead();
    }

    void PersistentBalanceControl::CopiesRead()
    {
        const auto newest = NewestCopy();

        if (newest)
        {
            sequence = copies[*newest]->sequence;
            nextCopy = 1 - *newest;
            Apply(*copies[*newest]);
            restore = RestoreResult::restored;
        }
        else
            restore = erased[0] && erased[1] ? RestoreResult::defaults : RestoreResult::corrupt;
    }

    std::optional<std::size_t> PersistentBalanceControl::NewestCopy() const
    {
        if (copies[0] && copies[1])
            return Newer(copies[1]->sequence, copies[0]->sequence) ? 1 : 0;
        if (copies[0])
            return 0;
        if (copies[1])
            return 1;
        return std::nullopt;
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
            const auto entry = std::find_if(record.entries.begin(), record.entries.end(), [key](const TuningEntry& candidate)
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
            ScheduleSave();

        return accepted;
    }

    void PersistentBalanceControl::ScheduleSave()
    {
        lastSave = SaveResult::pending;
        saveTimer.Start(config.saveDelay, [this]()
            {
                SaveWhenIdle();
            });
    }

    void PersistentBalanceControl::SaveWhenIdle()
    {
        if (saving)
            dirty = true;
        else if (restore == RestoreResult::pending || control.Engaged())
            ScheduleSave();
        else
            Save();
    }

    void PersistentBalanceControl::Save()
    {
        auto record = Capture();
        record.sequence = static_cast<uint16_t>(sequence + 1);
        writing = record.sequence;

        auto contents = images[nextCopy].Contents();
        std::fill(contents.begin(), contents.end(), erasedByte);
        Encode(record, contents);

        saving = true;
        images[nextCopy].Write([this](bool verified)
            {
                Saved(verified);
            });
    }

    void PersistentBalanceControl::Saved(bool verified)
    {
        saving = false;
        lastSave = verified ? SaveResult::saved : SaveResult::failed;

        if (verified)
        {
            sequence = writing;
            nextCopy = 1 - nextCopy;
        }

        if (dirty)
        {
            dirty = false;
            ScheduleSave();
        }
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
