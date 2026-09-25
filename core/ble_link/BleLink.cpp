#include "core/ble_link/BleLink.hpp"
#include "core/ble_link/WireFormat.hpp"
#include "services/ble/GapAdvertisingData.hpp"

namespace ble
{
    BleLink::MtuTracker::MtuTracker(RobotControlService& service)
        : service(service)
    {}

    void BleLink::MtuTracker::ServiceDiscovered(const services::GattService&)
    {}

    void BleLink::MtuTracker::IncludedServiceDiscovered(const services::GattIncludedService&)
    {}

    void BleLink::MtuTracker::CharacteristicDiscovered(const services::GattCharacteristic&)
    {}

    void BleLink::MtuTracker::DescriptorDiscovered(const services::GattDescriptor&)
    {}

    void BleLink::MtuTracker::MtuChanged(uint16_t mtu)
    {
        service.AttMtuChanged(mtu);
    }

    BleLink::BleLink(platform::Bluetooth& bluetooth, infra::BoundedConstString deviceName, safety::SafetySupervisor& supervisor, balance::BalanceControl& balanceControl, const telemetry::TelemetrySource& telemetry)
        : services::GapPeripheralObserver(bluetooth.Gap())
        , services::GattClientObserver(bluetooth.GattClient())
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

    void BleLink::ConnectionEstablished(infra::SharedPtr<services::GattClientConnection> established)
    {
        connection = established;
        mtuTracker.Attach(*connection);
        connection->ExchangeMtu([](services::GattResult) {});
    }

    void BleLink::ConnectionReleased(services::GattClientConnection& released)
    {
        if (connection == nullptr || &*connection != &released)
            return;

        mtuTracker.Detach();
        connection = nullptr;
    }

    void BleLink::Advertise()
    {
        auto& gap = services::GapPeripheralObserver::Subject();

        gap.SetAdvertisementData(infra::MakeRange(advertisement), [](services::GapPeripheral::Result) {});
        gap.SetScanResponseData(infra::MakeRange(scanResponse), [](services::GapPeripheral::Result) {});
        gap.Advertise(services::GapPeripheral::defaultAdvertisingParameters, [](services::GapPeripheral::Result) {});
    }
}
