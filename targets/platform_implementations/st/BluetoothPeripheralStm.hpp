#pragma once

#include "core/ble_link/DeviceIdentity.hpp"
#include "core/platform_abstraction/Bluetooth.hpp"
#include "hal_st/middlewares/ble_middleware/TracingGapPeripheralSt.hpp"
#include "hal_st/middlewares/ble_middleware/TracingGattClientSt.hpp"
#include "hal_st/middlewares/ble_middleware/TracingGattServerSt.hpp"

namespace application
{
    class BluetoothPeripheralStm final
        : public platform::Bluetooth
    {
    public:
        BluetoothPeripheralStm(hal::HciEventSource& hciEventSource, services::BondStorageSynchronizer& bondStorageSynchronizer, infra::BoundedConstString deviceName, services::Tracer& tracer);

        services::GapPeripheral& Gap() override;
        services::GattServer& GattServer() override;
        services::GattClient& GattClient() override;

    private:
        static constexpr uint16_t unknownAppearance{ 0 };
        static constexpr uint8_t zeroDbmPowerLevel{ 0x18 };
        static constexpr std::size_t numberOfLinks{ 1 };

        ble::DeviceIdentity identity;
        hal::GapSt::RootKeys rootKeys;
        hal::GapSt::GapService gapService;
        hal::GapSt::Configuration gapConfiguration;
        hal::TracingGapPeripheralSt gap;
        hal::TracingGattServerSt gattServer;
        hal::TracingGattClientSt::WithMaxConnections<numberOfLinks> gattClient;
    };
}
