#pragma once

#include "services/ble/GapPeripheral.hpp"
#include "services/ble/GattClient.hpp"
#include "services/ble/GattServer.hpp"

namespace platform
{
    class Bluetooth
    {
    public:
        Bluetooth() = default;
        Bluetooth(const Bluetooth& other) = delete;
        Bluetooth& operator=(const Bluetooth& other) = delete;

        virtual services::GapPeripheral& Gap() = 0;
        virtual services::GattServer& GattServer() = 0;
        virtual services::GattClient& GattClient() = 0;

    protected:
        ~Bluetooth() = default;
    };
}
