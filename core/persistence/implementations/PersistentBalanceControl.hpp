#pragma once

#include "core/balance_control/interfaces/BalanceControl.hpp"
#include "core/persistence/implementations/TuningRecord.hpp"
#include "core/persistence/interfaces/TuningStore.hpp"
#include "hal/interfaces/Eeprom.hpp"
#include "infra/timer/Timer.hpp"
#include "services/util/NvmEepromRegion.hpp"
#include "services/util/NvmImage.hpp"
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace persistence
{
    class PersistentBalanceControl final
        : public balance::BalanceControl
        , public TuningStore
    {
    public:
        static constexpr uint32_t regionSize{ 128 };

        struct Config
        {
            Config();

            infra::Duration saveDelay{ std::chrono::seconds(2) };
        };

        PersistentBalanceControl(balance::BalanceControl& control, hal::Eeprom& eeprom, const Config& config = Config());

        std::size_t StrategyCount() const override;
        const char* StrategyName(std::size_t index) const override;
        std::size_t ActiveStrategy() const override;
        bool Select(std::size_t index) override;

        infra::MemoryRange<const balance::ParameterDescriptor> Parameters() const override;
        float Parameter(std::size_t index) const override;
        bool SetParameter(std::size_t index, float value) override;

        infra::MemoryRange<const balance::ParameterDescriptor> StrategyParameters(std::size_t strategy) const override;
        float StrategyParameter(std::size_t strategy, std::size_t index) const override;
        bool SetStrategyParameter(std::size_t strategy, std::size_t index, float value) override;

        bool Move(const balance::Setpoints& setpoints) override;
        void CancelMotion() override;
        bool Engaged() const override;
        balance::Effort AppliedEffort() const override;

        RestoreResult Restore() const override;
        SaveResult LastSave() const override;

    private:
        static_assert(TuningRecord::maxSize <= regionSize);

        void ReadCopy(std::size_t copy);
        void CopyRead(infra::ConstByteRange data);
        void CopiesRead();
        std::optional<std::size_t> NewestCopy() const;
        void Apply(const TuningRecord& record);
        void ApplyStrategy(const TuningRecord& record, std::size_t strategy);
        bool Changed(bool accepted);
        void ScheduleSave();
        void SaveWhenIdle();
        void Save();
        void Saved(bool verified);
        TuningRecord Capture() const;

        balance::BalanceControl& control;
        Config config;
        std::array<services::NvmEepromRegion, 2> regions;
        std::array<services::NvmImage<regionSize>, 2> images;
        std::array<std::optional<TuningRecord>, 2> copies;
        std::array<bool, 2> erased{};
        uint16_t sequence{ 0 };
        uint16_t writing{ 0 };
        std::size_t reading{ 0 };
        std::size_t nextCopy{ 0 };
        bool saving{ false };
        bool dirty{ false };
        RestoreResult restore{ RestoreResult::pending };
        SaveResult lastSave{ SaveResult::none };
        infra::TimerSingleShot saveTimer;
    };
}
