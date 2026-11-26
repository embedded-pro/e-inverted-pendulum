#pragma once

#include "core/balance_control/interfaces/BalanceControl.hpp"
#include "core/persistence/implementations/TuningRecord.hpp"
#include "core/persistence/interfaces/TuningStore.hpp"
#include "hal/interfaces/Eeprom.hpp"
#include "infra/event/ClaimableResource.hpp"
#include "services/util/NvmEepromRegion.hpp"
#include "services/util/NvmRecordStore.hpp"
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace persistence
{
    class PersistentBalanceControl final
        : public balance::BalanceControl
        , public TuningStore
        , private services::NvmRecordStore::Client
    {
    public:
        static constexpr uint32_t regionSize{ 128 };

        struct Config
        {
            Config();

            infra::Duration saveDelay{ std::chrono::seconds(2) };
        };

        PersistentBalanceControl(balance::BalanceControl& control, hal::Eeprom& eeprom, infra::ClaimableResource& access, const Config& config = Config());

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
        static_assert(services::NvmRecordStore::headerSize + TuningRecord::maxSize <= regionSize);

        void Restored(infra::ConstByteRange payload) override;
        std::size_t Capture(infra::ByteRange payload) override;
        bool Busy() const override;

        void Apply(const TuningRecord& record);
        void ApplyStrategy(const TuningRecord& record, std::size_t strategy);
        bool Changed(bool accepted);
        TuningRecord Capture() const;

        balance::BalanceControl& control;
        services::NvmEepromRegion first;
        services::NvmEepromRegion second;
        bool undecodable{ false };
        services::NvmRecordStore::WithRegionSize<regionSize> store;
    };
}
