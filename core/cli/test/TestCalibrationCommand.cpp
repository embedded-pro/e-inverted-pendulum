#include "core/cli/CalibrationCommand.hpp"
#include "infra/stream/StringOutputStream.hpp"
#include "services/tracer/Tracer.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <numbers>
#include <string>

namespace
{
    constexpr float degree{ std::numbers::pi_v<float> / 180.0f };

    class CalibrationMock
        : public calibration::Calibration
    {
    public:
        MOCK_METHOD(calibration::Result, SetAxisMap, (const platform::AxisMap& map), (override));
        MOCK_METHOD(calibration::Result, SetStillnessThreshold, (float threshold), (override));
        MOCK_METHOD(calibration::Result, SetUprightOffset, (float offset), (override));
        MOCK_METHOD(calibration::Result, CaptureUpright, (), (override));
        MOCK_METHOD(calibration::Result, SetMotorInverted, (motion::WheelSide side, bool inverted), (override));
        MOCK_METHOD(calibration::Result, SetEncoderInverted, (odometry::WheelSide side, bool inverted), (override));
        MOCK_METHOD(calibration::Result, SetGeometry, (const odometry::Geometry& geometry), (override));
    };

    class StoreMock
        : public persistence::TuningStore
    {
    public:
        MOCK_METHOD(persistence::RestoreResult, Restore, (), (const, override));
        MOCK_METHOD(persistence::SaveResult, LastSave, (), (const, override));
    };

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

    class CalibrationCommandTest
        : public testing::Test
    {
    public:
        std::string Output() const
        {
            return std::string{ text.begin(), text.end() };
        }

        testing::StrictMock<CalibrationMock> calibration;
        testing::StrictMock<StoreMock> store;
        testing::StrictMock<InertialSensingMock> sensing;
        testing::StrictMock<AttitudeEstimationMock> estimation;
        testing::StrictMock<MotionActuationMock> actuation;
        testing::StrictMock<WheelOdometryMock> odometry;
        infra::BoundedString::WithStorage<2048> text;
        infra::StringOutputStream stream{ text };
        services::TracerToStream tracer{ stream };
        application::CalibrationCommand command{ tracer, { calibration, store, sensing, estimation, actuation, odometry } };
    };
}

TEST_F(CalibrationCommandTest, without_arguments_prints_every_value)
{
    EXPECT_CALL(sensing, CurrentAxisMap()).WillOnce(testing::Return(platform::AxisMap{ 0, 2, 1, 1.0f, -1.0f, 1.0f }));
    EXPECT_CALL(sensing, StillnessThreshold()).WillOnce(testing::Return(0.09f));
    EXPECT_CALL(estimation, UprightOffset()).WillOnce(testing::Return(0.0f));
    EXPECT_CALL(actuation, MotorInverted(motion::WheelSide::left)).WillOnce(testing::Return(true));
    EXPECT_CALL(actuation, MotorInverted(motion::WheelSide::right)).WillOnce(testing::Return(false));
    EXPECT_CALL(odometry, EncoderInverted(odometry::WheelSide::left)).WillOnce(testing::Return(false));
    EXPECT_CALL(odometry, EncoderInverted(odometry::WheelSide::right)).WillOnce(testing::Return(true));
    EXPECT_CALL(odometry, CurrentGeometry()).WillOnce(testing::Return(odometry::Geometry{}));
    EXPECT_CALL(store, Restore()).WillOnce(testing::Return(persistence::RestoreResult::restored));
    EXPECT_CALL(store, LastSave()).WillOnce(testing::Return(persistence::SaveResult::saved));

    command.Execute("");

    EXPECT_THAT(Output(), testing::HasSubstr("cal axes +x -z +y"));
    EXPECT_THAT(Output(), testing::HasSubstr("cal still 0.09"));
    EXPECT_THAT(Output(), testing::HasSubstr("cal upright 0"));
    EXPECT_THAT(Output(), testing::HasSubstr("cal motor left - right +"));
    EXPECT_THAT(Output(), testing::HasSubstr("cal encoder left + right -"));
    EXPECT_THAT(Output(), testing::HasSubstr("cal wheel 0.03"));
    EXPECT_THAT(Output(), testing::HasSubstr("cal store restore restored save saved"));
}

TEST_F(CalibrationCommandTest, axes_sets_the_axis_map_and_asks_for_calibrate)
{
    EXPECT_CALL(calibration, SetAxisMap(AxisMapIs(platform::AxisMap{ 0, 2, 1, 1.0f, -1.0f, 1.0f }))).WillOnce(testing::Return(calibration::Result::accepted));

    command.Execute("axes +x -z +y");

    EXPECT_THAT(Output(), testing::HasSubstr("run 'calibrate' before arming"));
}

TEST_F(CalibrationCommandTest, still_sets_the_stillness_limit)
{
    EXPECT_CALL(calibration, SetStillnessThreshold(testing::FloatNear(0.05f, 1e-6f))).WillOnce(testing::Return(calibration::Result::accepted));

    command.Execute("still 0.05");

    EXPECT_THAT(Output(), testing::HasSubstr("run 'calibrate' before arming"));
}

TEST_F(CalibrationCommandTest, upright_without_a_value_captures_the_pitch)
{
    EXPECT_CALL(calibration, CaptureUpright()).WillOnce(testing::Return(calibration::Result::accepted));

    command.Execute("upright");

    EXPECT_THAT(Output(), testing::HasSubstr("ok"));
}

TEST_F(CalibrationCommandTest, upright_with_a_value_sets_the_offset_in_degrees)
{
    EXPECT_CALL(calibration, SetUprightOffset(testing::FloatNear(1.5f * degree, 1e-6f))).WillOnce(testing::Return(calibration::Result::accepted));

    command.Execute("upright 1.5");
}

TEST_F(CalibrationCommandTest, a_negative_upright_offset_is_accepted)
{
    EXPECT_CALL(calibration, SetUprightOffset(testing::FloatNear(-1.5f * degree, 1e-6f))).WillOnce(testing::Return(calibration::Result::accepted));

    command.Execute("upright -1.5");
}

TEST_F(CalibrationCommandTest, repeated_spaces_between_arguments_are_ignored)
{
    EXPECT_CALL(calibration, SetAxisMap(AxisMapIs(platform::AxisMap{ 0, 2, 1, 1.0f, -1.0f, 1.0f }))).WillOnce(testing::Return(calibration::Result::accepted));

    command.Execute("axes  +x   -z +y ");
}

TEST_F(CalibrationCommandTest, motor_and_encoder_set_one_wheel_sign)
{
    EXPECT_CALL(calibration, SetMotorInverted(motion::WheelSide::left, true)).WillOnce(testing::Return(calibration::Result::accepted));
    EXPECT_CALL(calibration, SetEncoderInverted(odometry::WheelSide::right, false)).WillOnce(testing::Return(calibration::Result::accepted));

    command.Execute("motor left -");
    command.Execute("encoder right +");
}

TEST_F(CalibrationCommandTest, wheel_sets_the_geometry)
{
    EXPECT_CALL(calibration, SetGeometry(GeometryIs(odometry::Geometry{ 0.035f, 0.15f, 2.0f }))).WillOnce(testing::Return(calibration::Result::accepted));

    command.Execute("wheel 0.035 0.15 2.0");
}

TEST_F(CalibrationCommandTest, each_refusal_is_reported)
{
    EXPECT_CALL(calibration, CaptureUpright())
        .WillOnce(testing::Return(calibration::Result::armed))
        .WillOnce(testing::Return(calibration::Result::invalid))
        .WillOnce(testing::Return(calibration::Result::restoring));

    command.Execute("upright");
    command.Execute("upright");
    command.Execute("upright");

    EXPECT_THAT(Output(), testing::HasSubstr("need disarmed"));
    EXPECT_THAT(Output(), testing::HasSubstr("out of range"));
    EXPECT_THAT(Output(), testing::HasSubstr("still loading"));
}

TEST_F(CalibrationCommandTest, malformed_arguments_print_the_usage)
{
    for (const auto* line : { "axes +x -z", "axes +x -z +y +x", "axes +x -w +y", "axes +X -z +y", "axes x -z +y", "axes +xx -z +y", "still fast", "still 1", "motor middle -", "motor left", "encoder left x", "wheel 0.03 0.1", "wheel 0.03 0.1 2", "upright 1.5 2.0", "bogus" })
    {
        text.clear();
        command.Execute(line);
        EXPECT_THAT(Output(), testing::HasSubstr("usage: cal")) << line;
    }
}
