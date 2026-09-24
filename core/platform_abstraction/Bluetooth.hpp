#pragma once

#include "services/ble/GapPeripheral.hpp"
#include "services/ble/GattServer.hpp"
#include <cstdint>

namespace platform
{
    class BluetoothLinkObserver
    {
    public:
        BluetoothLinkObserver() = default;
        BluetoothLinkObserver(const BluetoothLinkObserver& other) = delete;
        BluetoothLinkObserver& operator=(const BluetoothLinkObserver& other) = delete;

        virtual void AttMtuChanged(uint16_t mtu) = 0;
        virtual void ClientConfigurationWritten(services::AttAttribute::Handle handle, uint16_t value) = 0;

    protected:
        ~BluetoothLinkObserver() = default;
    };

    class Bluetooth
    {
    public:
        Bluetooth() = default;
        Bluetooth(const Bluetooth& other) = delete;
        Bluetooth& operator=(const Bluetooth& other) = delete;

        virtual services::GapPeripheral& Gap() = 0;
        virtual services::GattServer& GattServer() = 0;
        virtual void SetLinkObserver(BluetoothLinkObserver& observer) = 0;

    protected:
        ~Bluetooth() = default;
    };
}
