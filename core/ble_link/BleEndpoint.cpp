#include "core/ble_link/BleEndpoint.hpp"

namespace ble
{
    BleEndpoint::BleEndpoint(platform::Platform& platform, infra::BoundedConstString deviceName, safety::SafetySupervisor& supervisor, balance::BalanceControl& balanceControl, const telemetry::TelemetrySource& telemetry)
        : deviceName{ deviceName }
        , supervisor{ supervisor }
        , balanceControl{ balanceControl }
        , telemetry{ telemetry }
    {
        platform.StartBluetooth(deviceName, [this](platform::Bluetooth& bluetooth)
            {
                RadioReady(bluetooth);
            });
    }

    std::optional<LinkReport> BleEndpoint::Report() const
    {
        if (link)
            return link->Report();

        return std::nullopt;
    }

    void BleEndpoint::RadioReady(platform::Bluetooth& bluetooth)
    {
        link.emplace(bluetooth, deviceName, supervisor, balanceControl, telemetry);
    }
}
