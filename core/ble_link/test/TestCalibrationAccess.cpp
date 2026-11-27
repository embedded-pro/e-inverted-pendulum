#include "core/ble_link/CalibrationAccess.hpp"
#include "core/ble_link/WireFormat.hpp"
#include "core/ble_link/test/Mocks.hpp"
#include "infra/event/test_helper/EventDispatcherFixture.hpp"
#include "services/ble/test_doubles/GattServerMock.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <array>
#include <cstdint>
#include <vector>

namespace
{
    using Access = ble::CalibrationAccess;
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

    MATCHER_P(AxisMapIs, expected, "")
    {
        return arg.xFrom == expected.xFrom && arg.yFrom == expected.yFrom && arg.zFrom == expected.zFrom && arg.xSign == expected.xSign && arg.ySign == expected.ySign && arg.zSign == expected.zSign;
    }

    MATCHER_P(GeometryIs, expected, "")
    {
        return arg.wheelRadius == expected.wheelRadius && arg.trackWidth == expected.trackWidth && arg.gearRatio == expected.gearRatio;
    }

    class CalibrationAccessTest
        : public testing::Test
        , public infra::EventDispatcherFixture
    {
    public:
        CalibrationAccessTest()
        {
            for (auto& characteristic : service.Characteristics())
                characteristic.Attach(operations);
        }

        services::GattServerCharacteristic& Characteristic()
        {
            return *service.Characteristics().begin();
        }

        void Write(const Bytes& data)
        {
            Characteristic().NotifyObservers([&data](auto& observer)
                {
                    observer.DataReceived(infra::ConstByteRange{ data.data(), data.data() + data.size() });
                });
        }

        Bytes Request(const Bytes& data, services::GattRequestStatus status = services::GattRequestStatus::accepted)
        {
            Bytes sent;
            EXPECT_CALL(operations, Update(testing::Ref(Characteristic()), testing::_)).WillOnce([&sent, status](const auto&, infra::ConstByteRange value)
                {
                    sent.assign(value.begin(), value.end());
                    return status;
                });

            Write(data);
            return sent;
        }

        testing::StrictMock<services::GattServerCharacteristicOperationsMock> operations;
        testing::StrictMock<ble::CalibrationMock> calibration;
        testing::StrictMock<ble::InertialSensingMock> sensing;
        testing::StrictMock<ble::AttitudeEstimationMock> estimation;
        testing::StrictMock<ble::MotionActuationMock> actuation;
        testing::StrictMock<ble::WheelOdometryMock> odometry;
        services::GattServerService service{ ble::uuid::robotControlService };
        Access access{ service, { calibration, sensing, estimation, actuation, odometry } };
    };
}

TEST_F(CalibrationAccessTest, adds_a_characteristic_that_needs_an_encrypted_link)
{
    using Properties = services::GattCharacteristic::PropertyFlags;
    using Permissions = services::GattServerCharacteristic::PermissionFlags;

    EXPECT_TRUE(Characteristic().Type() == services::AttAttribute::Uuid{ ble::uuid::calibration });
    EXPECT_EQ(Properties::write | Properties::notify, Characteristic().Properties());
    EXPECT_EQ(Permissions::encryptedWrite, Characteristic().Permissions());
}

TEST_F(CalibrationAccessTest, reading_all_reports_every_value)
{
    access.AttMtuChanged(largeMtu);
    EXPECT_CALL(sensing, CurrentAxisMap()).WillOnce(testing::Return(platform::AxisMap{ 0, 2, 1, 1.0f, -1.0f, 1.0f }));
    EXPECT_CALL(sensing, StillnessThreshold()).WillOnce(testing::Return(0.09f));
    EXPECT_CALL(estimation, UprightOffset()).WillOnce(testing::Return(-0.02f));
    EXPECT_CALL(actuation, MotorInverted(motion::WheelSide::left)).WillOnce(testing::Return(true));
    EXPECT_CALL(actuation, MotorInverted(motion::WheelSide::right)).WillOnce(testing::Return(false));
    EXPECT_CALL(odometry, EncoderInverted(odometry::WheelSide::left)).WillOnce(testing::Return(false));
    EXPECT_CALL(odometry, EncoderInverted(odometry::WheelSide::right)).WillOnce(testing::Return(true));
    EXPECT_CALL(odometry, CurrentGeometry()).WillOnce(testing::Return(odometry::Geometry{ 0.04f, 0.15f, 2.0f }));

    EXPECT_EQ(Concatenate({ { 1, 0, 0, 1, 0xfd, 2 }, Float(0.09f), Float(-0.02f), { 0xff, 1, 1, 0xff }, Float(0.04f), Float(0.15f), Float(2.0f) }), Request({ 1, 0 }));
}

TEST_F(CalibrationAccessTest, reading_all_with_a_small_mtu_carries_only_the_status)
{
    access.AttMtuChanged(32);

    EXPECT_EQ((Bytes{ 1, 0, 4 }), Request({ 1, 0 }));
}

TEST_F(CalibrationAccessTest, reading_all_fits_an_mtu_of_thirty_three_bytes)
{
    access.AttMtuChanged(33);
    EXPECT_CALL(sensing, CurrentAxisMap()).WillOnce(testing::Return(platform::AxisMap{}));
    EXPECT_CALL(sensing, StillnessThreshold()).WillOnce(testing::Return(0.09f));
    EXPECT_CALL(estimation, UprightOffset()).WillOnce(testing::Return(0.0f));
    EXPECT_CALL(actuation, MotorInverted(testing::_)).WillRepeatedly(testing::Return(false));
    EXPECT_CALL(odometry, EncoderInverted(testing::_)).WillRepeatedly(testing::Return(false));
    EXPECT_CALL(odometry, CurrentGeometry()).WillOnce(testing::Return(odometry::Geometry{}));

    EXPECT_EQ(30u, Request({ 1, 0 }).size());
}

TEST_F(CalibrationAccessTest, setting_the_axis_map_decodes_the_signed_axes)
{
    EXPECT_CALL(calibration, SetAxisMap(AxisMapIs(platform::AxisMap{ 1, 2, 0, -1.0f, 1.0f, -1.0f }))).WillOnce(testing::Return(calibration::Result::accepted));

    EXPECT_EQ((Bytes{ 2, 0, 0 }), Request({ 2, 0, 0xfe, 3, 0xff }));
}

TEST_F(CalibrationAccessTest, an_axis_outside_one_to_three_is_an_invalid_request)
{
    EXPECT_EQ((Bytes{ 2, 0, 3 }), Request({ 2, 0, 1, 0, 3 }));
}

TEST_F(CalibrationAccessTest, a_refusal_reports_its_reason)
{
    EXPECT_CALL(calibration, SetStillnessThreshold(0.05f)).WillOnce(testing::Return(calibration::Result::armed)).WillOnce(testing::Return(calibration::Result::restoring)).WillOnce(testing::Return(calibration::Result::invalid));

    EXPECT_EQ((Bytes{ 3, 0, 1 }), Request(Concatenate({ { 3, 0 }, Float(0.05f) })));
    EXPECT_EQ((Bytes{ 3, 0, 2 }), Request(Concatenate({ { 3, 0 }, Float(0.05f) })));
    EXPECT_EQ((Bytes{ 3, 0, 3 }), Request(Concatenate({ { 3, 0 }, Float(0.05f) })));
}

TEST_F(CalibrationAccessTest, setting_and_capturing_the_upright_offset)
{
    EXPECT_CALL(calibration, SetUprightOffset(-0.03f)).WillOnce(testing::Return(calibration::Result::accepted));
    EXPECT_CALL(calibration, CaptureUpright()).WillOnce(testing::Return(calibration::Result::accepted));

    EXPECT_EQ((Bytes{ 4, 0, 0 }), Request(Concatenate({ { 4, 0 }, Float(-0.03f) })));
    EXPECT_EQ((Bytes{ 5, 0, 0 }), Request({ 5, 0 }));
}

TEST_F(CalibrationAccessTest, setting_a_motor_and_an_encoder_sign_names_the_wheel)
{
    EXPECT_CALL(calibration, SetMotorInverted(motion::WheelSide::right, true)).WillOnce(testing::Return(calibration::Result::accepted));
    EXPECT_CALL(calibration, SetEncoderInverted(odometry::WheelSide::left, false)).WillOnce(testing::Return(calibration::Result::accepted));

    EXPECT_EQ((Bytes{ 6, 1, 0 }), Request({ 6, 1, 0xff }));
    EXPECT_EQ((Bytes{ 7, 0, 0 }), Request({ 7, 0, 1 }));
}

TEST_F(CalibrationAccessTest, a_sign_other_than_plus_or_minus_one_or_an_unknown_wheel_is_an_invalid_request)
{
    EXPECT_EQ((Bytes{ 6, 0, 3 }), Request({ 6, 0, 0 }));
    EXPECT_EQ((Bytes{ 7, 2, 3 }), Request({ 7, 2, 1 }));
}

TEST_F(CalibrationAccessTest, setting_the_geometry_changes_all_three_values)
{
    EXPECT_CALL(calibration, SetGeometry(GeometryIs(odometry::Geometry{ 0.04f, 0.15f, 2.0f }))).WillOnce(testing::Return(calibration::Result::accepted));

    EXPECT_EQ((Bytes{ 8, 0, 0 }), Request(Concatenate({ { 8, 0 }, Float(0.04f), Float(0.15f), Float(2.0f) })));
}

TEST_F(CalibrationAccessTest, malformed_requests_are_invalid)
{
    EXPECT_EQ((Bytes{ 0, 0, 3 }), Request({}));
    EXPECT_EQ((Bytes{ 9, 0, 3 }), Request({ 9, 0 }));
    EXPECT_EQ((Bytes{ 1, 0, 3 }), Request({ 1, 0, 0 }));
    EXPECT_EQ((Bytes{ 2, 0, 3 }), Request({ 2, 0, 1, 2 }));
    EXPECT_EQ((Bytes{ 3, 0, 3 }), Request({ 3, 0, 1 }));
    EXPECT_EQ((Bytes{ 5, 0, 3 }), Request({ 5, 0, 1 }));
    EXPECT_EQ((Bytes{ 6, 0, 3 }), Request({ 6, 0 }));
    EXPECT_EQ((Bytes{ 8, 0, 3 }), Request(Concatenate({ { 8, 0 }, Float(0.04f) })));
    EXPECT_EQ((Bytes{ 3, 1, 3 }), Request(Concatenate({ { 3, 1 }, Float(0.05f) })));
}

TEST_F(CalibrationAccessTest, a_request_while_a_response_is_outstanding_is_ignored)
{
    Request({ 9, 0 }, services::GattRequestStatus::invalidState);

    Write({ 9, 0 });
}

TEST_F(CalibrationAccessTest, disconnecting_ends_an_outstanding_response_and_forgets_the_mtu)
{
    access.AttMtuChanged(largeMtu);
    Request({ 9, 0 }, services::GattRequestStatus::invalidState);

    access.Disconnected();

    EXPECT_EQ((Bytes{ 1, 0, 4 }), Request({ 1, 0 }));
}
