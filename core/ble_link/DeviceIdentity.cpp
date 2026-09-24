#include "core/ble_link/DeviceIdentity.hpp"
#include <cstddef>

namespace ble
{
    namespace
    {
        constexpr uint32_t unprogrammedDeviceNumber{ 0xffffffff };
        constexpr uint32_t identityRootDomain{ 0x49524b00 };
        constexpr uint32_t encryptionRootDomain{ 0x45524b00 };
        constexpr uint32_t addressDomain{ 0x41445200 };

        constexpr uint32_t Mix(uint32_t value)
        {
            value ^= value >> 16;
            value *= 0x7feb352du;
            value ^= value >> 15;
            value *= 0x846ca68bu;
            value ^= value >> 16;
            return value;
        }

        uint32_t Fold(const std::array<uint32_t, 3>& uniqueId, uint32_t seed)
        {
            auto value = Mix(seed);

            for (auto word : uniqueId)
                value = Mix(value ^ word);

            return value;
        }

        uint32_t AddressLowBytes(const FactoryIdentity& factory)
        {
            if (factory.uniqueDeviceNumber != unprogrammedDeviceNumber)
                return factory.uniqueDeviceNumber;

            return Fold(factory.uniqueId, addressDomain);
        }

        uint8_t ByteOf(uint32_t value, std::size_t index)
        {
            return static_cast<uint8_t>(value >> (8 * index));
        }

        RootKey DeriveRootKey(const std::array<uint32_t, 3>& uniqueId, uint32_t domain)
        {
            RootKey key{};
            constexpr std::size_t wordCount{ key.size() / sizeof(uint32_t) };

            for (std::size_t word = 0; word != wordCount; ++word)
            {
                auto value = Fold(uniqueId, domain + static_cast<uint32_t>(word));

                for (std::size_t byte = 0; byte != sizeof(uint32_t); ++byte)
                    key[word * sizeof(uint32_t) + byte] = ByteOf(value, byte);
            }

            return key;
        }
    }

    DeviceIdentity DeriveDeviceIdentity(const FactoryIdentity& factory)
    {
        auto lowBytes = AddressLowBytes(factory);

        return {
            { ByteOf(lowBytes, 0), ByteOf(lowBytes, 1), factory.deviceId, ByteOf(factory.companyId, 0), ByteOf(factory.companyId, 1), ByteOf(factory.companyId, 2) },
            DeriveRootKey(factory.uniqueId, identityRootDomain),
            DeriveRootKey(factory.uniqueId, encryptionRootDomain)
        };
    }
}
