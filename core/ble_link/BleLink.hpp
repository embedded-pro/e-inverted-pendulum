#pragma once

#include "core/ble_link/RobotControlService.hpp"
#include "core/ble_link/interfaces/LinkStatus.hpp"
#include "core/platform_abstraction/Bluetooth.hpp"
#include "infra/util/BoundedString.hpp"
#include "infra/util/BoundedVector.hpp"
#include "services/ble/GapPeripheral.hpp"
#include "services/ble/GattServer.hpp"

namespace ble
{
    class BleLink
        : private services::GapPeripheralObserver
        , private services::GattServerObserver
    {
    public:
        static constexpr std::size_t maximumAdvertisementSize{ 31 };

        BleLink(platform::Bluetooth& bluetooth, infra::BoundedConstString deviceName, safety::SafetySupervisor& supervisor, balance::BalanceControl& balanceControl, const telemetry::TelemetrySource& telemetry);
        ~BleLink();

        bool Connected() const;
        LinkReport Report() const;

    private:
        void StateChanged(services::GapPeripheralState newState) override;
        void MaxAttMtuSizeChanged(uint16_t maxAttMtuSize) override;
        void Advertise();

        RobotControlService service;
        infra::BoundedVector<uint8_t>::WithMaxSize<maximumAdvertisementSize> advertisement;
        infra::BoundedVector<uint8_t>::WithMaxSize<maximumAdvertisementSize> scanResponse;
        services::GapPeripheralState state{ services::GapPeripheralState::standby };
    };
}
