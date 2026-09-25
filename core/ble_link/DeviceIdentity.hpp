#pragma once

#include "hal/interfaces/MacAddress.hpp"
#include <array>
#include <cstdint>

namespace ble
{
    struct FactoryIdentity
    {
        uint32_t uniqueDeviceNumber;
        uint8_t deviceId;
        uint32_t companyId;
        std::array<uint32_t, 3> uniqueId;
    };

    using RootKey = std::array<uint8_t, 16>;

    struct DeviceIdentity
    {
        hal::MacAddress address;
        RootKey identityRoot;
        RootKey encryptionRoot;
    };

    DeviceIdentity DeriveDeviceIdentity(const FactoryIdentity& factory);
}
