#pragma once

#include "core/ble_link/RobotControlService.hpp"
#include "core/platform_abstraction/Bluetooth.hpp"
#include "infra/util/BoundedString.hpp"
#include "infra/util/BoundedVector.hpp"
#include "services/ble/GapPeripheral.hpp"

namespace ble
{
    class BleLink
        : private services::GapPeripheralObserver
    {
    public:
        static constexpr std::size_t maximumAdvertisementSize{ 31 };

        BleLink(platform::Bluetooth& bluetooth, infra::BoundedConstString deviceName, safety::SafetySupervisor& supervisor, balance::BalanceControl& balanceControl, const telemetry::TelemetrySource& telemetry);
        ~BleLink();

        bool Connected() const;

    private:
        void StateChanged(services::GapPeripheralState state) override;
        void Advertise();

        RobotControlService service;
        infra::BoundedVector<uint8_t>::WithMaxSize<maximumAdvertisementSize> advertisement;
        infra::BoundedVector<uint8_t>::WithMaxSize<maximumAdvertisementSize> scanResponse;
        bool connected{ false };
    };
}
