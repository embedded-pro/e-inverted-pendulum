#include "core/balance_control/test_doubles/BalanceControlMock.hpp"
#include "core/calibration/implementations/CalibrationImpl.hpp"
#include "hal/interfaces/test_doubles/EepromStub.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "infra/util/Crc.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <array>
#include <chrono>
#include <optional>
#include <vector>

namespace
{
    using namespace std::chrono_literals;

    class InertialSensingMock
        : public sensing::InertialSensing
    {
    public:
        MOCK_METHOD(sensing::Measurement, Latest, (), (const, override));
        MOCK_METHOD(void, OnMeasurement, (const infra::Function<void(const sensing::Measurement&)>& onMeasurement), (override));
        MOCK_METHOD(sensing::InvalidCause, Cause, (), (const, override));
        MOCK_METHOD(void, StartCalibration, (), (override));
        MOCK_METHOD(sensing::CalibrationState, Calibration, (), (const, override));
        MOCK_METHOD(platform::InertialAxes, GyroscopeBias, (), (const, override));
        MOCK_METHOD(bool, SetAxisMap, (const platform::AxisMap& map), (override));
        MOCK_METHOD(platform::AxisMap, CurrentAxisMap, (), (const, override));
        MOCK_METHOD(bool, SetStillnessThreshold, (float threshold), (override));
        MOCK_METHOD(float, StillnessThreshold, (), (const, override));
    };

    class AttitudeEstimationMock
        : public estimation::AttitudeEstimation
    {
    public:
        MOCK_METHOD(estimation::Estimate, Update, (const sensing::Measurement& measurement), (override));
        MOCK_METHOD(estimation::Estimate, Latest, (), (const, override));
        MOCK_METHOD(void, Select, (estimation::Filter filter), (override));
        MOCK_METHOD(estimation::Filter, Selected, (), (const, override));
        MOCK_METHOD(bool, SetUprightOffset, (float offset), (override));
        MOCK_METHOD(float, UprightOffset, (), (const, override));
    };

    class MotionActuationMock
        : public motion::MotionActuation
    {
    public:
        MOCK_METHOD(void, Apply, (float effortLeft, float effortRight), (override));
        MOCK_METHOD(void, Disable, (motion::DisableState state), (override));
        MOCK_METHOD(motion::FaultCause, Fault, (), (const, override));
        MOCK_METHOD(void, ClearFault, (), (override));
        MOCK_METHOD(motion::DriverState, State, (), (const, override));
        MOCK_METHOD(void, SetMotorInverted, (motion::WheelSide side, bool inverted), (override));
        MOCK_METHOD(bool, MotorInverted, (motion::WheelSide side), (const, override));
    };

    class WheelOdometryMock
        : public odometry::WheelOdometry
    {
    public:
        MOCK_METHOD(odometry::WheelMotion, Left, (), (const, override));
        MOCK_METHOD(odometry::WheelMotion, Right, (), (const, override));
        MOCK_METHOD(odometry::ChassisMotion, Chassis, (), (const, override));
        MOCK_METHOD(void, SetEncoderInverted, (odometry::WheelSide side, bool inverted), (override));
        MOCK_METHOD(bool, EncoderInverted, (odometry::WheelSide side), (const, override));
        MOCK_METHOD(bool, SetGeometry, (const odometry::Geometry& geometry), (override));
        MOCK_METHOD(odometry::Geometry, CurrentGeometry, (), (const, override));
    };

    MATCHER_P(AxisMapIs, expected, "")
    {
        return arg.xFrom == expected.xFrom && arg.yFrom == expected.yFrom && arg.zFrom == expected.zFrom && arg.xSign == expected.xSign && arg.ySign == expected.ySign && arg.zSign == expected.zSign;
    }

    MATCHER_P(GeometryIs, expected, "")
    {
        return arg.wheelRadius == expected.wheelRadius && arg.trackWidth == expected.trackWidth && arg.gearRatio == expected.gearRatio;
    }

    constexpr platform::AxisMap tilted{ 0, 2, 1, 1.0f, -1.0f, 1.0f };
    constexpr odometry::Geometry measured{ 0.035f, 0.15f, 2.0f };

    class CalibrationImplTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        CalibrationImplTest()
        {
            EXPECT_CALL(control, Engaged()).WillRepeatedly(testing::ReturnPointee(&engaged));
        }

        void Start()
        {
            calibration.emplace(calibration::CalibrationImpl::Components{ sensing, estimation, actuation, odometry, control }, eeprom, access);
            ExecuteAllActions();
        }

        void ExpectCapture()
        {
            EXPECT_CALL(sensing, CurrentAxisMap()).WillOnce(testing::Return(tilted));
            EXPECT_CALL(sensing, StillnessThreshold()).WillOnce(testing::Return(0.05f));
            EXPECT_CALL(estimation, UprightOffset()).WillOnce(testing::Return(0.02f));
            EXPECT_CALL(actuation, MotorInverted(motion::WheelSide::left)).WillOnce(testing::Return(true));
            EXPECT_CALL(actuation, MotorInverted(motion::WheelSide::right)).WillOnce(testing::Return(false));
            EXPECT_CALL(odometry, EncoderInverted(odometry::WheelSide::left)).WillOnce(testing::Return(false));
            EXPECT_CALL(odometry, EncoderInverted(odometry::WheelSide::right)).WillOnce(testing::Return(true));
            EXPECT_CALL(odometry, CurrentGeometry()).WillOnce(testing::Return(measured));
        }

        void SaveAxisMap()
        {
            EXPECT_CALL(sensing, SetAxisMap(AxisMapIs(tilted))).WillOnce(testing::Return(true));
            EXPECT_EQ(calibration::Result::accepted, calibration->SetAxisMap(tilted));
            ExpectCapture();
            ForwardTime(2s);
        }

        void StorePayload(std::vector<uint8_t> payload)
        {
            std::vector<uint8_t> copy{ 0x52, 0x53, 1, 0, static_cast<uint8_t>(payload.size()), 0 };
            copy.insert(copy.end(), payload.begin(), payload.end());
            infra::Crc16CcittFalse crc;
            crc.Update(infra::MakeRange(copy));
            copy.insert(copy.begin() + 6, { static_cast<uint8_t>(crc.Result()), static_cast<uint8_t>(crc.Result() >> 8) });
            eeprom.WriteBuffer(infra::MakeRange(copy), calibration::CalibrationImpl::baseAddress, [] {});
            ExecuteAllActions();
        }

        void Store(std::initializer_list<std::pair<const char*, float>> values)
        {
            persistence::TuningRecord record;
            for (const auto& [name, value] : values)
                record.entries.push_back({ persistence::KeyOf("cal", name), value });
            std::array<uint8_t, persistence::TuningRecord::maxSize> payload{};
            const auto size = persistence::Encode(record, infra::MakeRange(payload));
            StorePayload(std::vector<uint8_t>(payload.begin(), payload.begin() + static_cast<std::ptrdiff_t>(size)));
        }

        bool RegionErased(uint32_t address)
        {
            std::array<uint8_t, 8> header{};
            eeprom.ReadBuffer(infra::MakeRange(header), address, [] {});
            ExecuteAllActions();
            return std::ranges::all_of(header, [](uint8_t byte)
                {
                    return byte == 0xff;
                });
        }

        testing::StrictMock<InertialSensingMock> sensing;
        testing::StrictMock<AttitudeEstimationMock> estimation;
        testing::StrictMock<MotionActuationMock> actuation;
        testing::StrictMock<WheelOdometryMock> odometry;
        testing::StrictMock<balance::BalanceControlMock> control;
        hal::EepromStub eeprom{ 512 };
        infra::ClaimableResource access;
        bool engaged{ false };
        std::optional<calibration::CalibrationImpl> calibration;
    };
}

TEST_F(CalibrationImplTest, the_startup_condition_holds_once_the_restore_ends)
{
    calibration.emplace(calibration::CalibrationImpl::Components{ sensing, estimation, actuation, odometry, control }, eeprom, access);
    EXPECT_FALSE(calibration->Satisfied());

    ExecuteAllActions();

    EXPECT_TRUE(calibration->Satisfied());
    EXPECT_EQ(persistence::RestoreResult::defaults, calibration->Restore());
}

TEST_F(CalibrationImplTest, every_change_is_refused_while_the_drive_is_engaged)
{
    Start();
    engaged = true;

    EXPECT_EQ(calibration::Result::armed, calibration->SetAxisMap(tilted));
    EXPECT_EQ(calibration::Result::armed, calibration->SetStillnessThreshold(0.05f));
    EXPECT_EQ(calibration::Result::armed, calibration->SetUprightOffset(0.01f));
    EXPECT_EQ(calibration::Result::armed, calibration->CaptureUpright());
    EXPECT_EQ(calibration::Result::armed, calibration->SetMotorInverted(motion::WheelSide::left, true));
    EXPECT_EQ(calibration::Result::armed, calibration->SetEncoderInverted(odometry::WheelSide::right, true));
    EXPECT_EQ(calibration::Result::armed, calibration->SetGeometry(measured));
    EXPECT_EQ(persistence::SaveResult::none, calibration->LastSave());
}

TEST_F(CalibrationImplTest, a_rejected_value_is_invalid_and_not_saved)
{
    Start();
    EXPECT_CALL(sensing, SetStillnessThreshold(5.0f)).WillOnce(testing::Return(false));
    EXPECT_CALL(odometry, SetGeometry(testing::_)).WillOnce(testing::Return(false));

    EXPECT_EQ(calibration::Result::invalid, calibration->SetStillnessThreshold(5.0f));
    EXPECT_EQ(calibration::Result::invalid, calibration->SetGeometry(odometry::Geometry{ 1.0f, 1.0f, 1.0f }));
    ForwardTime(5s);
    EXPECT_EQ(persistence::SaveResult::none, calibration->LastSave());
}

TEST_F(CalibrationImplTest, accepted_values_are_forwarded)
{
    Start();
    EXPECT_CALL(sensing, SetStillnessThreshold(0.05f)).WillOnce(testing::Return(true));
    EXPECT_CALL(estimation, SetUprightOffset(0.01f)).WillOnce(testing::Return(true));
    EXPECT_CALL(actuation, SetMotorInverted(motion::WheelSide::left, true));
    EXPECT_CALL(odometry, SetEncoderInverted(odometry::WheelSide::right, true));
    EXPECT_CALL(odometry, SetGeometry(GeometryIs(measured))).WillOnce(testing::Return(true));

    EXPECT_EQ(calibration::Result::accepted, calibration->SetStillnessThreshold(0.05f));
    EXPECT_EQ(calibration::Result::accepted, calibration->SetUprightOffset(0.01f));
    EXPECT_EQ(calibration::Result::accepted, calibration->SetMotorInverted(motion::WheelSide::left, true));
    EXPECT_EQ(calibration::Result::accepted, calibration->SetEncoderInverted(odometry::WheelSide::right, true));
    EXPECT_EQ(calibration::Result::accepted, calibration->SetGeometry(measured));
    EXPECT_EQ(persistence::SaveResult::pending, calibration->LastSave());
}

TEST_F(CalibrationImplTest, capturing_upright_adds_the_current_pitch_to_the_offset)
{
    Start();
    EXPECT_CALL(estimation, Latest()).WillOnce(testing::Return(estimation::Estimate{ 0.03f, 0.0f, true, estimation::InvalidCause::none }));
    EXPECT_CALL(estimation, UprightOffset()).WillOnce(testing::Return(0.01f));
    EXPECT_CALL(estimation, SetUprightOffset(testing::FloatNear(0.04f, 1e-6f))).WillOnce(testing::Return(true));

    EXPECT_EQ(calibration::Result::accepted, calibration->CaptureUpright());
}

TEST_F(CalibrationImplTest, capturing_upright_needs_a_valid_estimate)
{
    Start();
    EXPECT_CALL(estimation, Latest()).WillOnce(testing::Return(estimation::Estimate{}));

    EXPECT_EQ(calibration::Result::invalid, calibration->CaptureUpright());
}

TEST_F(CalibrationImplTest, a_change_is_saved_after_the_delay_in_its_own_region)
{
    Start();

    SaveAxisMap();

    EXPECT_EQ(persistence::SaveResult::saved, calibration->LastSave());
    EXPECT_TRUE(RegionErased(0));
    EXPECT_TRUE(RegionErased(128));
    EXPECT_FALSE(RegionErased(calibration::CalibrationImpl::baseAddress));
}

TEST_F(CalibrationImplTest, the_save_waits_while_the_drive_is_engaged)
{
    Start();
    EXPECT_CALL(sensing, SetAxisMap(AxisMapIs(tilted))).WillOnce(testing::Return(true));
    EXPECT_EQ(calibration::Result::accepted, calibration->SetAxisMap(tilted));
    engaged = true;

    ForwardTime(2s);
    EXPECT_EQ(persistence::SaveResult::pending, calibration->LastSave());

    engaged = false;
    ExpectCapture();
    ForwardTime(2s);
    EXPECT_EQ(persistence::SaveResult::saved, calibration->LastSave());
}

TEST_F(CalibrationImplTest, a_saved_calibration_is_restored_after_a_reset)
{
    Start();
    SaveAxisMap();
    calibration.reset();

    testing::InSequence order;
    EXPECT_CALL(sensing, SetAxisMap(AxisMapIs(tilted))).WillOnce(testing::Return(true));
    EXPECT_CALL(sensing, SetStillnessThreshold(0.05f)).WillOnce(testing::Return(true));
    EXPECT_CALL(estimation, SetUprightOffset(0.02f)).WillOnce(testing::Return(true));
    EXPECT_CALL(actuation, SetMotorInverted(motion::WheelSide::left, true));
    EXPECT_CALL(actuation, SetMotorInverted(motion::WheelSide::right, false));
    EXPECT_CALL(odometry, SetEncoderInverted(odometry::WheelSide::left, false));
    EXPECT_CALL(odometry, SetEncoderInverted(odometry::WheelSide::right, true));
    EXPECT_CALL(odometry, CurrentGeometry()).WillOnce(testing::Return(odometry::Geometry{}));
    EXPECT_CALL(odometry, SetGeometry(GeometryIs(measured))).WillOnce(testing::Return(true));
    Start();

    EXPECT_EQ(persistence::RestoreResult::restored, calibration->Restore());
    EXPECT_TRUE(calibration->Satisfied());
}

TEST_F(CalibrationImplTest, missing_keys_keep_their_defaults)
{
    Store({ { "still", 0.05f }, { "wheel.radius", 0.04f } });

    EXPECT_CALL(sensing, SetStillnessThreshold(0.05f)).WillOnce(testing::Return(true));
    EXPECT_CALL(odometry, CurrentGeometry()).WillOnce(testing::Return(odometry::Geometry{}));
    EXPECT_CALL(odometry, SetGeometry(GeometryIs(odometry::Geometry{ 0.04f, 0.145f, 1.0f }))).WillOnce(testing::Return(true));
    Start();

    EXPECT_EQ(persistence::RestoreResult::restored, calibration->Restore());
}

TEST_F(CalibrationImplTest, an_incomplete_axis_map_is_not_applied_and_reported_as_corrupt)
{
    Store({ { "axis.x", -1.0f }, { "axis.y", -3.0f } });

    EXPECT_CALL(odometry, CurrentGeometry()).WillOnce(testing::Return(odometry::Geometry{}));
    EXPECT_CALL(odometry, SetGeometry(testing::_)).WillOnce(testing::Return(true));
    Start();

    EXPECT_EQ(persistence::RestoreResult::corrupt, calibration->Restore());
}

TEST_F(CalibrationImplTest, an_axis_map_with_every_axis_inverted_is_restored)
{
    Store({ { "axis.x", -2.0f }, { "axis.y", -1.0f }, { "axis.z", -3.0f } });

    EXPECT_CALL(sensing, SetAxisMap(AxisMapIs(platform::AxisMap{ 1, 0, 2, -1.0f, -1.0f, -1.0f }))).WillOnce(testing::Return(true));
    EXPECT_CALL(odometry, CurrentGeometry()).WillOnce(testing::Return(odometry::Geometry{}));
    EXPECT_CALL(odometry, SetGeometry(testing::_)).WillOnce(testing::Return(true));
    Start();

    EXPECT_EQ(persistence::RestoreResult::restored, calibration->Restore());
}

TEST_F(CalibrationImplTest, a_rejected_restored_value_is_reported_as_corrupt)
{
    Store({ { "upright", 1.0f } });

    EXPECT_CALL(estimation, SetUprightOffset(1.0f)).WillOnce(testing::Return(false));
    EXPECT_CALL(odometry, CurrentGeometry()).WillOnce(testing::Return(odometry::Geometry{}));
    EXPECT_CALL(odometry, SetGeometry(testing::_)).WillOnce(testing::Return(true));
    Start();

    EXPECT_EQ(persistence::RestoreResult::corrupt, calibration->Restore());
    EXPECT_TRUE(calibration->Satisfied());
}

TEST_F(CalibrationImplTest, an_undecodable_record_is_reported_as_corrupt)
{
    StorePayload({ 0x7f, 0x00, 0x00, 0x00 });

    Start();

    EXPECT_EQ(persistence::RestoreResult::corrupt, calibration->Restore());
    EXPECT_TRUE(calibration->Satisfied());
}
