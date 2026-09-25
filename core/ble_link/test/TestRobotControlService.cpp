#include "core/ble_link/RobotControlService.hpp"
#include "core/ble_link/WireFormat.hpp"
#include "core/ble_link/test/Mocks.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "services/ble/test_doubles/GattServerMock.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <vector>

namespace
{
    using namespace std::chrono_literals;
    using Service = ble::RobotControlService;
    using Bytes = std::vector<uint8_t>;

    constexpr uint16_t largeMtu{ 247 };

    Bytes Float(float value)
    {
        std::array<uint8_t, 4> bytes{};
        ble::PutFloat(infra::MakeRange(bytes), 0, value);
        return Bytes{ bytes.begin(), bytes.end() };
    }

    Bytes Concatenate(std::initializer_list<Bytes> parts)
    {
        Bytes result;

        for (const auto& part : parts)
            result.insert(result.end(), part.begin(), part.end());

        return result;
    }

    constexpr std::array<balance::ParameterDescriptor, 2> descriptors{ {
        { "pitch.kp", 0.0f, 50.0f },
        { "pitch.ki", 0.0f, 100.0f },
    } };

    class RobotControlServiceTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        RobotControlServiceTest()
        {
            EXPECT_CALL(gattServer, AddService(testing::_)).WillOnce([this](services::GattServerService& added)
                {
                    registered = &added;
                });

            service.emplace(gattServer, supervisor, balanceControl, telemetrySource);

            for (auto& characteristic : registered->Characteristics())
                characteristic.Attach(operations);
        }

        services::GattServerCharacteristic& Characteristic(const services::AttAttribute::Uuid128& type)
        {
            for (auto& characteristic : registered->Characteristics())
                if (characteristic.Type() == services::AttAttribute::Uuid{ type })
                    return characteristic;

            std::abort();
        }

        void Write(const services::AttAttribute::Uuid128& type, const Bytes& data)
        {
            Characteristic(type).NotifyObservers([&data](auto& observer)
                {
                    observer.DataReceived(infra::ConstByteRange{ data.data(), data.data() + data.size() });
                });
        }

        void ExpectUpdate(const services::AttAttribute::Uuid128& type, Bytes& sent, services::GattRequestStatus status = services::GattRequestStatus::accepted)
        {
            EXPECT_CALL(operations, Update(testing::Ref(Characteristic(type)), testing::_)).WillOnce([&sent, status](const auto&, infra::ConstByteRange data)
                {
                    sent.assign(data.begin(), data.end());
                    return status;
                });
        }

        void ExpectMode(safety::Mode mode, safety::FaultCause cause = safety::FaultCause::none)
        {
            EXPECT_CALL(supervisor, Current()).WillRepeatedly(testing::Return(mode));
            EXPECT_CALL(supervisor, LatchedCause()).WillRepeatedly(testing::Return(cause));
        }

        void Connect()
        {
            ExpectMode(safety::Mode::idle);
            Bytes mode;
            ExpectUpdate(ble::uuid::mode, mode);
            service->Connected();
        }

        void RaiseMtu()
        {
            service->AttMtuChanged(largeMtu);
        }

        testing::StrictMock<services::GattServerMock> gattServer;
        testing::StrictMock<services::GattServerCharacteristicOperationsMock> operations;
        testing::StrictMock<ble::SafetySupervisorMock> supervisor;
        testing::StrictMock<ble::BalanceControlMock> balanceControl;
        testing::StrictMock<ble::TelemetrySourceMock> telemetrySource;
        services::GattServerService* registered{ nullptr };
        std::optional<Service> service;
    };
}

TEST_F(RobotControlServiceTest, registers_the_robot_control_service_with_four_characteristics)
{
    EXPECT_TRUE(registered->Type() == services::AttAttribute::Uuid{ ble::uuid::robotControlService });

    std::size_t count{ 0 };
    for ([[maybe_unused]] auto& characteristic : registered->Characteristics())
        ++count;

    EXPECT_EQ(4u, count);
}

TEST_F(RobotControlServiceTest, every_commanding_characteristic_needs_an_encrypted_link)
{
    using Permissions = services::GattServerCharacteristic::PermissionFlags;

    EXPECT_EQ(Permissions::encryptedWrite, Characteristic(ble::uuid::motion).Permissions());
    EXPECT_EQ(Permissions::encryptedWrite, Characteristic(ble::uuid::mode).Permissions());
    EXPECT_EQ(Permissions::encryptedWrite, Characteristic(ble::uuid::tuning).Permissions());
    EXPECT_EQ(Permissions::none, Characteristic(ble::uuid::telemetry).Permissions());
}

TEST_F(RobotControlServiceTest, a_motion_write_commands_velocity_and_yaw_rate)
{
    EXPECT_CALL(balanceControl, Move(balance::Setpoints{ 0.25f, -0.5f })).WillOnce(testing::Return(true));

    Write(ble::uuid::motion, Concatenate({ Float(0.25f), Float(-0.5f) }));
}

TEST_F(RobotControlServiceTest, a_motion_write_of_the_wrong_length_is_ignored)
{
    Write(ble::uuid::motion, Float(0.25f));
    Write(ble::uuid::motion, Concatenate({ Float(0.25f), Float(-0.5f), Bytes{ 0 } }));
}

TEST_F(RobotControlServiceTest, connecting_publishes_the_mode)
{
    ExpectMode(safety::Mode::idle);
    Bytes mode;
    ExpectUpdate(ble::uuid::mode, mode);

    service->Connected();

    EXPECT_EQ((Bytes{ 2, 0, 0, 0 }), mode);
}

TEST_F(RobotControlServiceTest, an_arm_command_is_forwarded_and_its_outcome_reported)
{
    Connect();
    EXPECT_CALL(supervisor, Arm()).WillOnce(testing::Return(true));
    ExpectMode(safety::Mode::armed);
    Bytes mode;
    ExpectUpdate(ble::uuid::mode, mode);

    Write(ble::uuid::mode, Bytes{ 1 });

    EXPECT_EQ((Bytes{ 3, 0, 1, 1 }), mode);
}

TEST_F(RobotControlServiceTest, every_mode_command_reaches_the_supervisor)
{
    Connect();
    EXPECT_CALL(supervisor, Disarm()).WillOnce(testing::Return(false));
    EXPECT_CALL(supervisor, ClearFault()).WillOnce(testing::Return(true));
    EXPECT_CALL(supervisor, Calibrate()).WillOnce(testing::Return(false));
    EXPECT_CALL(operations, Update(testing::Ref(Characteristic(ble::uuid::mode)), testing::_)).Times(3).WillRepeatedly(testing::Return(services::GattRequestStatus::accepted));

    Write(ble::uuid::mode, Bytes{ 2 });
    Write(ble::uuid::mode, Bytes{ 3 });
    Write(ble::uuid::mode, Bytes{ 4 });
}

TEST_F(RobotControlServiceTest, an_unknown_mode_command_is_refused)
{
    Connect();
    Bytes mode;
    ExpectUpdate(ble::uuid::mode, mode);

    Write(ble::uuid::mode, Bytes{ 9 });

    EXPECT_EQ((Bytes{ 2, 0, 9, 2 }), mode);
}

TEST_F(RobotControlServiceTest, a_mode_change_is_published_on_the_next_tick_only_once)
{
    Connect();
    ExpectMode(safety::Mode::fault, safety::FaultCause::fall);
    Bytes mode;
    ExpectUpdate(ble::uuid::mode, mode);

    ForwardTime(40ms);
    EXPECT_EQ((Bytes{ 4, 1, 0, 0 }), mode);

    ForwardTime(40ms);
}

TEST_F(RobotControlServiceTest, telemetry_is_not_sent_while_disconnected)
{
    RaiseMtu();

    ForwardTime(200ms);
}

TEST_F(RobotControlServiceTest, telemetry_waits_for_an_mtu_large_enough)
{
    Connect();

    ForwardTime(200ms);
}

TEST_F(RobotControlServiceTest, a_connected_client_receives_telemetry_every_forty_milliseconds)
{
    Connect();
    RaiseMtu();
    EXPECT_CALL(telemetrySource, Latest()).WillRepeatedly(testing::Return(telemetry::Sample{ 0.1f, -0.2f, 0.3f, -0.4f, 0.5f, -0.6f, safety::Mode::armed, safety::FaultCause::none }));
    Bytes sent;
    ExpectUpdate(ble::uuid::telemetry, sent);

    ForwardTime(39ms);
    ForwardTime(1ms);

    EXPECT_EQ(Concatenate({ Float(0.1f), Float(-0.2f), Float(0.3f), Float(-0.4f), Float(0.5f), Float(-0.6f), Bytes{ 3, 0 } }), sent);

    EXPECT_CALL(operations, Update(testing::Ref(Characteristic(ble::uuid::telemetry)), testing::_)).Times(2).WillRepeatedly(testing::Return(services::GattRequestStatus::accepted));
    ForwardTime(80ms);
}

TEST_F(RobotControlServiceTest, telemetry_reports_the_latched_fault_cause)
{
    Connect();
    RaiseMtu();
    EXPECT_CALL(telemetrySource, Latest()).WillOnce(testing::Return(telemetry::Sample{ 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, safety::Mode::fault, safety::FaultCause::loopStalled }));
    Bytes sent;
    ExpectUpdate(ble::uuid::telemetry, sent);

    ForwardTime(40ms);

    ASSERT_EQ(Service::telemetrySize, sent.size());
    EXPECT_EQ(4, sent[24]);
    EXPECT_EQ(4, sent[25]);
}

TEST_F(RobotControlServiceTest, telemetry_is_dropped_while_the_previous_update_is_outstanding)
{
    Connect();
    RaiseMtu();
    EXPECT_CALL(telemetrySource, Latest()).WillOnce(testing::Return(telemetry::Sample{}));
    Bytes sent;
    ExpectUpdate(ble::uuid::telemetry, sent, services::GattRequestStatus::invalidState);

    ForwardTime(40ms);
    ForwardTime(200ms);
}

TEST_F(RobotControlServiceTest, disconnecting_decays_motion_and_forgets_the_mtu)
{
    Connect();
    RaiseMtu();
    EXPECT_CALL(balanceControl, CancelMotion());

    service->Disconnected();
    ForwardTime(200ms);

    Connect();
    ForwardTime(200ms);
}

TEST_F(RobotControlServiceTest, describing_a_strategy_reports_count_active_index_and_name)
{
    service->AttMtuChanged(largeMtu);
    EXPECT_CALL(balanceControl, StrategyCount()).WillRepeatedly(testing::Return(2));
    EXPECT_CALL(balanceControl, ActiveStrategy()).WillOnce(testing::Return(0));
    EXPECT_CALL(balanceControl, StrategyName(1)).WillOnce(testing::Return("lqr"));
    Bytes response;
    ExpectUpdate(ble::uuid::tuning, response);

    Write(ble::uuid::tuning, Bytes{ 1, 1 });

    EXPECT_EQ((Bytes{ 1, 1, 0, 2, 0, 3, 'l', 'q', 'r' }), response);
}

TEST_F(RobotControlServiceTest, describing_an_unknown_strategy_is_an_invalid_request)
{
    EXPECT_CALL(balanceControl, StrategyCount()).WillRepeatedly(testing::Return(2));
    Bytes response;
    ExpectUpdate(ble::uuid::tuning, response);

    Write(ble::uuid::tuning, Bytes{ 1, 2 });

    EXPECT_EQ((Bytes{ 1, 2, 2 }), response);
}

TEST_F(RobotControlServiceTest, selecting_a_strategy_reports_whether_balance_control_accepted_it)
{
    EXPECT_CALL(balanceControl, StrategyCount()).WillRepeatedly(testing::Return(2));
    EXPECT_CALL(balanceControl, Select(1)).WillOnce(testing::Return(false));
    Bytes response;
    ExpectUpdate(ble::uuid::tuning, response);

    Write(ble::uuid::tuning, Bytes{ 2, 1 });

    EXPECT_EQ((Bytes{ 2, 1, 1 }), response);
}

TEST_F(RobotControlServiceTest, describing_a_parameter_reports_count_value_range_and_name)
{
    service->AttMtuChanged(largeMtu);
    EXPECT_CALL(balanceControl, Parameters()).WillRepeatedly(testing::Return(infra::MakeRange(descriptors)));
    EXPECT_CALL(balanceControl, Parameter(1)).WillOnce(testing::Return(0.5f));
    Bytes response;
    ExpectUpdate(ble::uuid::tuning, response);

    Write(ble::uuid::tuning, Bytes{ 3, 1 });

    EXPECT_EQ(Concatenate({ Bytes{ 3, 1, 0, 2 }, Float(0.5f), Float(0.0f), Float(100.0f), Bytes{ 8, 'p', 'i', 't', 'c', 'h', '.', 'k', 'i' } }), response);
}

TEST_F(RobotControlServiceTest, a_description_that_does_not_fit_the_mtu_carries_only_its_status)
{
    EXPECT_CALL(balanceControl, Parameters()).WillRepeatedly(testing::Return(infra::MakeRange(descriptors)));
    EXPECT_CALL(balanceControl, Parameter(0)).WillOnce(testing::Return(2.0f));
    Bytes response;
    ExpectUpdate(ble::uuid::tuning, response);

    Write(ble::uuid::tuning, Bytes{ 3, 0 });

    EXPECT_EQ((Bytes{ 3, 0, 3 }), response);
}

TEST_F(RobotControlServiceTest, writing_a_parameter_reports_the_outcome_and_the_stored_value)
{
    EXPECT_CALL(balanceControl, Parameters()).WillRepeatedly(testing::Return(infra::MakeRange(descriptors)));
    EXPECT_CALL(balanceControl, SetParameter(0, 3.5f)).WillOnce(testing::Return(true));
    EXPECT_CALL(balanceControl, Parameter(0)).WillOnce(testing::Return(3.5f));
    Bytes response;
    ExpectUpdate(ble::uuid::tuning, response);

    Write(ble::uuid::tuning, Concatenate({ Bytes{ 4, 0 }, Float(3.5f) }));

    EXPECT_EQ(Concatenate({ Bytes{ 4, 0, 0 }, Float(3.5f) }), response);
}

TEST_F(RobotControlServiceTest, a_refused_parameter_write_reports_the_unchanged_value)
{
    EXPECT_CALL(balanceControl, Parameters()).WillRepeatedly(testing::Return(infra::MakeRange(descriptors)));
    EXPECT_CALL(balanceControl, SetParameter(0, 60.0f)).WillOnce(testing::Return(false));
    EXPECT_CALL(balanceControl, Parameter(0)).WillOnce(testing::Return(2.0f));
    Bytes response;
    ExpectUpdate(ble::uuid::tuning, response);

    Write(ble::uuid::tuning, Concatenate({ Bytes{ 4, 0 }, Float(60.0f) }));

    EXPECT_EQ(Concatenate({ Bytes{ 4, 0, 1 }, Float(2.0f) }), response);
}

TEST_F(RobotControlServiceTest, malformed_tuning_requests_are_invalid)
{
    EXPECT_CALL(balanceControl, Parameters()).WillRepeatedly(testing::Return(infra::MakeRange(descriptors)));
    std::vector<Bytes> responses;
    EXPECT_CALL(operations, Update(testing::Ref(Characteristic(ble::uuid::tuning)), testing::_)).Times(4).WillRepeatedly([&responses](const auto&, infra::ConstByteRange data)
        {
            responses.emplace_back(data.begin(), data.end());
            return services::GattRequestStatus::accepted;
        });

    Write(ble::uuid::tuning, Bytes{ 7 });
    Write(ble::uuid::tuning, Bytes{ 9, 0 });
    Write(ble::uuid::tuning, Bytes{ 4, 0, 1 });
    Write(ble::uuid::tuning, Bytes{ 3, 0, 0 });

    EXPECT_EQ((std::vector<Bytes>{ { 7, 0, 2 }, { 9, 0, 2 }, { 4, 0, 2 }, { 3, 0, 2 } }), responses);
}

TEST_F(RobotControlServiceTest, a_request_while_a_response_is_outstanding_is_ignored)
{
    EXPECT_CALL(balanceControl, StrategyCount()).WillRepeatedly(testing::Return(2));
    Bytes response;
    ExpectUpdate(ble::uuid::tuning, response, services::GattRequestStatus::invalidState);

    Write(ble::uuid::tuning, Bytes{ 1, 5 });
    Write(ble::uuid::tuning, Bytes{ 1, 6 });
}
