#include "core/ble_link/BleLink.hpp"
#include "core/ble_link/WireFormat.hpp"
#include "services/ble/GapAdvertisingData.hpp"

namespace ble
{
    BleLink::BleLink(platform::Bluetooth& bluetooth, infra::BoundedConstString deviceName, safety::SafetySupervisor& supervisor, balance::BalanceControl& balanceControl, const telemetry::TelemetrySource& telemetry)
        : services::GapPeripheralObserver(bluetooth.Gap())
        , service(bluetooth.GattServer(), supervisor, balanceControl, telemetry)
    {
        bluetooth.SetLinkObserver(service);

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
        return connected;
    }

    void BleLink::StateChanged(services::GapPeripheralState state)
    {
        if (state == services::GapPeripheralState::connected)
        {
            connected = true;
            service.Connected();
        }
        else if (state == services::GapPeripheralState::standby)
        {
            if (connected)
                service.Disconnected();

            connected = false;
            Advertise();
        }
    }

    void BleLink::Advertise()
    {
        auto& gap = Subject();

        gap.SetAdvertisementData(infra::MakeRange(advertisement), [](services::GapPeripheral::Result) {});
        gap.SetScanResponseData(infra::MakeRange(scanResponse), [](services::GapPeripheral::Result) {});
        gap.Advertise(services::GapPeripheral::defaultAdvertisingParameters, [](services::GapPeripheral::Result) {});
    }
}
