#include "core/ble_link/DeviceIdentity.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace
{
    constexpr uint32_t stCompanyId{ 0x0080e1 };
    constexpr uint8_t wb55DeviceId{ 0x26 };

    ble::FactoryIdentity Programmed(uint32_t uniqueDeviceNumber, std::array<uint32_t, 3> uniqueId = { 0x00410029, 0x3032510b, 0x20393451 })
    {
        return { uniqueDeviceNumber, wb55DeviceId, stCompanyId, uniqueId };
    }

    ble::FactoryIdentity Unprogrammed(std::array<uint32_t, 3> uniqueId)
    {
        return { 0xffffffff, wb55DeviceId, stCompanyId, uniqueId };
    }
}

TEST(DeviceIdentityTest, address_carries_the_device_number_device_id_and_company_id_least_significant_byte_first)
{
    const auto identity = ble::DeriveDeviceIdentity(Programmed(0xa1b21234));

    EXPECT_THAT(identity.address, testing::ElementsAre(0x34, 0x12, wb55DeviceId, 0xe1, 0x80, 0x00));
}

TEST(DeviceIdentityTest, an_unprogrammed_device_number_falls_back_to_the_unique_id)
{
    const auto first = ble::DeriveDeviceIdentity(Unprogrammed({ 1, 2, 3 }));
    const auto second = ble::DeriveDeviceIdentity(Unprogrammed({ 1, 2, 4 }));

    EXPECT_THAT((std::array<uint8_t, 4>{ first.address[2], first.address[3], first.address[4], first.address[5] }), testing::ElementsAre(wb55DeviceId, 0xe1, 0x80, 0x00));
    EXPECT_NE(first.address, second.address);
}

TEST(DeviceIdentityTest, derivation_is_repeatable)
{
    const auto first = ble::DeriveDeviceIdentity(Programmed(0x1234));
    const auto second = ble::DeriveDeviceIdentity(Programmed(0x1234));

    EXPECT_EQ(first.address, second.address);
    EXPECT_EQ(first.identityRoot, second.identityRoot);
    EXPECT_EQ(first.encryptionRoot, second.encryptionRoot);
}

TEST(DeviceIdentityTest, identity_and_encryption_roots_differ)
{
    const auto identity = ble::DeriveDeviceIdentity(Programmed(0x1234));

    EXPECT_NE(identity.identityRoot, identity.encryptionRoot);
    EXPECT_NE(identity.identityRoot, ble::RootKey{});
    EXPECT_NE(identity.encryptionRoot, ble::RootKey{});
}

TEST(DeviceIdentityTest, roots_differ_between_parts)
{
    const auto first = ble::DeriveDeviceIdentity(Programmed(0x1234, { 1, 2, 3 }));
    const auto second = ble::DeriveDeviceIdentity(Programmed(0x1234, { 1, 2, 5 }));

    EXPECT_NE(first.identityRoot, second.identityRoot);
    EXPECT_NE(first.encryptionRoot, second.encryptionRoot);
}
