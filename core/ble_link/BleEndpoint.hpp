#pragma once

#include "core/ble_link/BleLink.hpp"
#include "core/ble_link/interfaces/LinkStatus.hpp"
#include "core/platform_abstraction/Platform.hpp"
#include <optional>

namespace ble
{
    class BleEndpoint
        : public LinkStatus
    {
    public:
        BleEndpoint(platform::Platform& platform, infra::BoundedConstString deviceName, safety::SafetySupervisor& supervisor, balance::BalanceControl& balanceControl, const telemetry::TelemetrySource& telemetry);

        LinkReport Report() const override;

    private:
        void RadioReady(platform::Bluetooth& bluetooth);

        infra::BoundedConstString deviceName;
        safety::SafetySupervisor& supervisor;
        balance::BalanceControl& balanceControl;
        const telemetry::TelemetrySource& telemetry;
        std::optional<BleLink> link;
    };
}
