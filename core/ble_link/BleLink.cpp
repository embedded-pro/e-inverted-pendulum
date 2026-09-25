#include "core/ble_link/BleLink.hpp"
#include "core/ble_link/WireFormat.hpp"
#include "services/ble/GapAdvertisingData.hpp"

namespace ble
{
    BleLink::BleLink(platform::Bluetooth& bluetooth, infra::BoundedConstString deviceName, safety::SafetySupervisor& supervisor, balance::BalanceControl& balanceControl, const telemetry::TelemetrySource& telemetry)
        : services::GapPeripheralObserver(bluetooth.Gap())
        , services::GattServerObserver(bluetooth.GattServer())
        , service(bluetooth.GattServer(), supervisor, balanceControl, telemetry)
    {
        services::GapAdvertisementFormatter advertisementFormatter{ advertisement };
        advertisementFormatter.AppendFlags(services::GapAdvertisementFlags::leGeneralDiscoverableMode | services::GapAdvertisementFlags::brEdrNotSupported);
        advertisementFormatter.AppendCompleteLocalName(deviceName);

        std::array<services::AttAttribute::Uuid128, 1> serviceUuids{ uuid::robotControlService };
        services::GapAdvertisementFormatter scanResponseFormatter{ scanResponse };
        scanResponseFormatter.AppendListOfServicesUuid(infra::MakeRange(serviceUuids));

        Advertise();
    }

    BleLink::~BleLink()
    {
        services::GapPeripheralObserver::Detach();
    }

    bool BleLink::Connected() const
    {
        return state == services::GapPeripheralState::connected;
    }

    LinkReport BleLink::Report() const
    {
        return { state, services::GapPeripheralObserver::Subject().GetIdentityAddress().address, service.Mtu() };
    }

    void BleLink::StateChanged(services::GapPeripheralState newState)
    {
        const auto wasConnected = Connected();
        state = newState;

        if (newState == services::GapPeripheralState::connected)
            service.Connected();
        else if (newState == services::GapPeripheralState::standby)
        {
            if (wasConnected)
                service.Disconnected();

            Advertise();
        }
    }

    void BleLink::MaxAttMtuSizeChanged(uint16_t maxAttMtuSize)
    {
        service.AttMtuChanged(maxAttMtuSize);
    }

    void BleLink::Advertise()
    {
        auto& gap = services::GapPeripheralObserver::Subject();

        gap.SetAdvertisementData(infra::MakeRange(advertisement), [](services::GapPeripheral::Result) {});
        gap.SetScanResponseData(infra::MakeRange(scanResponse), [](services::GapPeripheral::Result) {});
        gap.Advertise(services::GapPeripheral::defaultAdvertisingParameters, [](services::GapPeripheral::Result) {});
    }
}
