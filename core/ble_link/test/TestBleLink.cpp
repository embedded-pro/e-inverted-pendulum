#include "core/ble_link/BleLink.hpp"
#include "core/ble_link/WireFormat.hpp"
#include "core/ble_link/test/Mocks.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "services/ble/GapAdvertisingData.hpp"
#include "services/ble/test_doubles/GapPeripheralMock.hpp"
#include "services/ble/test_doubles/GattServerMock.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <optional>
#include <vector>

namespace
{
    using Bytes = std::vector<uint8_t>;

    class BleLinkTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        BleLinkTest()
        {
            EXPECT_CALL(bluetooth, Gap()).WillRepeatedly(testing::ReturnRef(gap));
            EXPECT_CALL(bluetooth, GattServer()).WillRepeatedly(testing::ReturnRef(gattServer));
            EXPECT_CALL(gattServer, AddService(testing::_)).WillOnce([this](services::GattServerService& added)
                {
                    registered = &added;
                });
            EXPECT_CALL(bluetooth, SetLinkObserver(testing::_));
            ExpectAdvertising();

            link.emplace(bluetooth, "inverted-pendulum", supervisor, balanceControl, telemetrySource);

            for (auto& characteristic : registered->Characteristics())
                characteristic.Attach(operations);
        }

        void ExpectAdvertising()
        {
            EXPECT_CALL(gap, SetAdvertisementData(testing::_, testing::_)).WillOnce([this](infra::ConstByteRange data, const auto&)
                {
                    advertisement.assign(data.begin(), data.end());
                    return services::GapRequestStatus::accepted;
                });
            EXPECT_CALL(gap, SetScanResponseData(testing::_, testing::_)).WillOnce([this](infra::ConstByteRange data, const auto&)
                {
                    scanResponse.assign(data.begin(), data.end());
                    return services::GapRequestStatus::accepted;
                });
            EXPECT_CALL(gap, Advertise(testing::_, testing::_)).WillOnce(testing::Return(services::GapRequestStatus::accepted));
        }

        void Connect()
        {
            EXPECT_CALL(supervisor, Current()).WillRepeatedly(testing::Return(safety::Mode::idle));
            EXPECT_CALL(supervisor, LatchedCause()).WillRepeatedly(testing::Return(safety::FaultCause::none));
            EXPECT_CALL(operations, Update(testing::_, testing::_)).WillRepeatedly(testing::Return(services::GattRequestStatus::accepted));
            gap.ChangeState(services::GapPeripheralState::connected);
        }

        testing::StrictMock<ble::BluetoothMock> bluetooth;
        testing::StrictMock<services::GapPeripheralMock> gap;
        testing::StrictMock<services::GattServerMock> gattServer;
        testing::StrictMock<services::GattServerCharacteristicOperationsMock> operations;
        testing::StrictMock<ble::SafetySupervisorMock> supervisor;
        testing::StrictMock<ble::BalanceControlMock> balanceControl;
        testing::StrictMock<ble::TelemetrySourceMock> telemetrySource;
        services::GattServerService* registered{ nullptr };
        Bytes advertisement;
        Bytes scanResponse;
        std::optional<ble::BleLink> link;
    };
}

TEST_F(BleLinkTest, advertises_the_device_name_and_the_robot_control_service)
{
    const Bytes name{ 'i', 'n', 'v', 'e', 'r', 't', 'e', 'd', '-', 'p', 'e', 'n', 'd', 'u', 'l', 'u', 'm' };
    Bytes expectedAdvertisement{ 0x02, 0x01, 0x06, static_cast<uint8_t>(name.size() + 1), 0x09 };
    expectedAdvertisement.insert(expectedAdvertisement.end(), name.begin(), name.end());

    EXPECT_EQ(expectedAdvertisement, advertisement);
    ASSERT_EQ(18u, scanResponse.size());
    EXPECT_EQ(17, scanResponse[0]);
    EXPECT_EQ(0x07, scanResponse[1]);
    EXPECT_FALSE(link->Connected());
}

TEST_F(BleLinkTest, a_connection_starts_the_service)
{
    Connect();

    EXPECT_TRUE(link->Connected());
}

TEST_F(BleLinkTest, a_disconnection_decays_motion_and_advertises_again)
{
    Connect();
    EXPECT_CALL(balanceControl, CancelMotion());
    ExpectAdvertising();

    gap.ChangeState(services::GapPeripheralState::standby);

    EXPECT_FALSE(link->Connected());
}

TEST_F(BleLinkTest, entering_advertising_changes_nothing)
{
    gap.ChangeState(services::GapPeripheralState::advertising);

    EXPECT_FALSE(link->Connected());
}
