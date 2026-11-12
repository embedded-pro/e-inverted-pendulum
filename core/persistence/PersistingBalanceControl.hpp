#pragma once

#include "core/balance_control/interfaces/BalanceControl.hpp"
#include "core/safety_supervisor/interfaces/SafetySupervisor.hpp"
#include "generated/echo/Tuning.pb.hpp"
#include "infra/timer/Timer.hpp"
#include "services/util/ConfigurationStore.hpp"
#include <chrono>

namespace persistence
{
    class PersistingBalanceControl
        : public balance::BalanceControl
    {
    public:
        struct Config
        {
            Config();

            std::chrono::milliseconds saveDelay{ 2000 };
        };

        PersistingBalanceControl(balance::BalanceControl& control, const safety::SafetySupervisor& supervisor, services::ConfigurationStoreAccess<Tuning> store, const Config& config = Config());

        void Restore();

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

    private:
        bool ScheduleSaveIf(bool accepted);
        void SaveUnlessArmed();
        void Capture(Tuning& tuning) const;
        void RestoreStrategy(const StoredStrategy& stored);
        std::optional<std::size_t> StrategyNamed(infra::BoundedConstString name) const;

        balance::BalanceControl& control;
        const safety::SafetySupervisor& supervisor;
        services::ConfigurationStoreAccess<Tuning> store;
        Config config;
        infra::TimerSingleShot saveTimer;
    };
}
