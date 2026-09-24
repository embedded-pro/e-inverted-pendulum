#include "core/attitude_estimation/interfaces/AttitudeEstimation.hpp"
#include "core/cli/Cli.hpp"
#include "core/control_loop/interfaces/ControlLoop.hpp"
#include "core/inertial_sensing/interfaces/InertialSensing.hpp"
#include "core/motion_actuation/interfaces/MotionActuation.hpp"
#include "core/platform_abstraction/test_doubles/PlatformMock.hpp"
#include "core/safety_supervisor/interfaces/SafetySupervisor.hpp"
#include "core/wheel_odometry/interfaces/WheelOdometry.hpp"
#include "hal/interfaces/test_doubles/SerialCommunicationMock.hpp"
#include "infra/stream/StringOutputStream.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "services/tracer/Tracer.hpp"
#include "gmock/gmock.h"
#include <chrono>
#include <string>

namespace
{
    class GpioStub final
        : public hal::GpioPin
    {
    public:
        bool Get() const override
        {
            return false;
        }

        void Set(bool) override
        {}

        bool GetOutputLatch() const override
        {
            return false;
        }

        void SetAsInput() override
        {}

        bool IsInput() const override
        {
            return false;
        }

        void Config(hal::PinConfigType) override
        {}

        void Config(hal::PinConfigType, bool) override
        {}

        void ResetConfig() override
        {}

        void EnableInterrupt(const infra::Function<void()>&, hal::InterruptTrigger, hal::InterruptType) override
        {}

        void DisableInterrupt() override
        {}
    };

    class MotionActuationMock
        : public motion::MotionActuation
    {
    public:
        virtual ~MotionActuationMock() = default;

        MOCK_METHOD(void, Apply, (float effortLeft, float effortRight), (override));
        MOCK_METHOD(void, Disable, (motion::DisableState state), (override));
        MOCK_METHOD(motion::FaultCause, Fault, (), (const, override));
        MOCK_METHOD(void, ClearFault, (), (override));
        MOCK_METHOD(motion::DriverState, State, (), (const, override));
    };

    class WheelOdometryMock
        : public odometry::WheelOdometry
    {
    public:
        virtual ~WheelOdometryMock() = default;

        MOCK_METHOD(odometry::WheelMotion, Left, (), (const, override));
        MOCK_METHOD(odometry::WheelMotion, Right, (), (const, override));
        MOCK_METHOD(odometry::ChassisMotion, Chassis, (), (const, override));
    };

    class InertialSensingMock
        : public sensing::InertialSensing
    {
    public:
        virtual ~InertialSensingMock() = default;

        MOCK_METHOD(sensing::Measurement, Latest, (), (const, override));
        MOCK_METHOD(void, OnMeasurement, (const infra::Function<void(const sensing::Measurement&)>& onMeasurement), (override));
        MOCK_METHOD(sensing::InvalidCause, Cause, (), (const, override));
        MOCK_METHOD(void, StartCalibration, (), (override));
        MOCK_METHOD(sensing::CalibrationState, Calibration, (), (const, override));
        MOCK_METHOD(platform::InertialAxes, GyroscopeBias, (), (const, override));
    };

    class AttitudeEstimationMock
        : public estimation::AttitudeEstimation
    {
    public:
        virtual ~AttitudeEstimationMock() = default;

        MOCK_METHOD(estimation::Estimate, Update, (const sensing::Measurement& measurement), (override));
        MOCK_METHOD(estimation::Estimate, Latest, (), (const, override));
        MOCK_METHOD(void, Select, (estimation::Filter filter), (override));
        MOCK_METHOD(estimation::Filter, Selected, (), (const, override));
    };

    class ControlLoopMock
        : public control::ControlLoop
    {
    public:
        virtual ~ControlLoopMock() = default;

        MOCK_METHOD(control::TimingStatistics, Statistics, (), (const, override));
        MOCK_METHOD(void, ResetStatistics, (), (override));
    };

    class SafetySupervisorMock
        : public safety::SafetySupervisor
    {
    public:
        virtual ~SafetySupervisorMock() = default;

        MOCK_METHOD(bool, Arm, (), (override));
        MOCK_METHOD(bool, Disarm, (), (override));
        MOCK_METHOD(bool, ClearFault, (), (override));
        MOCK_METHOD(bool, Calibrate, (), (override));
        MOCK_METHOD(safety::Mode, Current, (), (const, override));
        MOCK_METHOD(safety::FaultCause, LatchedCause, (), (const, override));
        MOCK_METHOD(bool, DrivePermitted, (), (const, override));
    };

    class CliTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        CliTest()
        {
            EXPECT_CALL(platform, StatusLed()).WillRepeatedly(testing::ReturnRef(led));
            EXPECT_CALL(platform, Communication()).WillRepeatedly(testing::ReturnRef(communication));
            EXPECT_CALL(platform, Tracer()).WillRepeatedly(testing::ReturnRef(tracer));

            EXPECT_CALL(communication, SendDataMock(testing::_)).Times(testing::AnyNumber());
        }

        std::string Output() const
        {
            return std::string{ text.begin(), text.end() };
        }

        void Send(const std::string& line)
        {
            const auto terminated = line + "\r\n";
            const infra::ConstByteRange data{ reinterpret_cast<const uint8_t*>(terminated.data()), reinterpret_cast<const uint8_t*>(terminated.data() + terminated.size()) };

            communication.dataReceived(data);

            for (int i = 0; i != 64 && communication.actionOnCompletion; ++i)
                communication.actionOnCompletion();

            ExecuteAllActions();
        }

        GpioStub led;
        testing::StrictMock<hal::SerialCommunicationMock> communication;
        infra::BoundedString::WithStorage<512> text;
        infra::StringOutputStream stream{ text };
        services::TracerToStream tracer{ stream };
        testing::StrictMock<platform::PlatformMock> platform;
        testing::StrictMock<MotionActuationMock> motionActuation;
        testing::StrictMock<WheelOdometryMock> wheelOdometry;
        testing::StrictMock<InertialSensingMock> inertialSensing;
        testing::StrictMock<AttitudeEstimationMock> attitudeEstimation;
        testing::StrictMock<ControlLoopMock> controlLoop;
        testing::StrictMock<SafetySupervisorMock> supervisor;
    };
}

TEST_F(CliTest, greets_and_shows_a_prompt_on_construction)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_THAT(Output(), testing::HasSubstr("ready"));
    EXPECT_THAT(Output(), testing::HasSubstr("arm"));
    EXPECT_THAT(Output(), testing::HasSubstr("> "));
}

TEST_F(CliTest, ping_replies_pong)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    Send("ping");

    EXPECT_THAT(Output(), testing::HasSubstr("pong"));
}

TEST_F(CliTest, id_prints_the_board_identifier)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    Send("id");

    EXPECT_THAT(Output(), testing::HasSubstr("inverted-pendulum-bot cli"));
}

TEST_F(CliTest, drive_applies_both_efforts)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(supervisor, DrivePermitted()).WillOnce(testing::Return(true));
    EXPECT_CALL(motionActuation, Apply(testing::FloatEq(0.3f), testing::FloatEq(-0.7f)));

    Send("drive 0.3 -0.7");

    EXPECT_THAT(Output(), testing::HasSubstr("driving"));
}

TEST_F(CliTest, drive_without_a_second_argument_prints_usage)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(supervisor, DrivePermitted()).WillOnce(testing::Return(true));

    Send("drive 0.3");

    EXPECT_THAT(Output(), testing::HasSubstr("usage: drive"));
}

TEST_F(CliTest, drive_is_refused_unless_armed)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(supervisor, DrivePermitted()).WillOnce(testing::Return(false));

    Send("drive 0.3 -0.7");

    EXPECT_THAT(Output(), testing::HasSubstr("refused: not armed"));
}

TEST_F(CliTest, tristate_releases_the_bridges)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(motionActuation, Disable(motion::DisableState::tristate));

    Send("tristate");

    EXPECT_THAT(Output(), testing::HasSubstr("tristated"));
}

TEST_F(CliTest, brake_shorts_the_motors)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(motionActuation, Disable(motion::DisableState::brake));

    Send("brake");

    EXPECT_THAT(Output(), testing::HasSubstr("braking"));
}

TEST_F(CliTest, drive_with_a_missing_right_argument_is_rejected)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(supervisor, DrivePermitted()).WillOnce(testing::Return(true));

    Send("drive 0.3 ");

    EXPECT_THAT(Output(), testing::HasSubstr("usage: drive"));
}

TEST_F(CliTest, drive_with_a_non_numeric_argument_is_rejected)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(supervisor, DrivePermitted()).WillOnce(testing::Return(true));

    Send("drive abc def");

    EXPECT_THAT(Output(), testing::HasSubstr("usage: drive"));
}

TEST_F(CliTest, drive_with_a_trailing_third_argument_is_rejected)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(supervisor, DrivePermitted()).WillOnce(testing::Return(true));

    Send("drive 0.3 0.7 0.9");

    EXPECT_THAT(Output(), testing::HasSubstr("usage: drive"));
}

TEST_F(CliTest, drive_with_a_partially_numeric_argument_is_rejected)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(supervisor, DrivePermitted()).WillOnce(testing::Return(true));

    Send("drive 0.3 0.7x");

    EXPECT_THAT(Output(), testing::HasSubstr("usage: drive"));
}

TEST_F(CliTest, drive_with_an_over_long_argument_is_rejected)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(supervisor, DrivePermitted()).WillOnce(testing::Return(true));

    Send("drive 0.3 0.70000000000000");

    EXPECT_THAT(Output(), testing::HasSubstr("usage: drive"));
}

TEST_F(CliTest, odom_reports_both_wheels_and_the_chassis)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(wheelOdometry, Left()).WillOnce(testing::Return(odometry::WheelMotion{ 120, 3.5f }));
    EXPECT_CALL(wheelOdometry, Right()).WillOnce(testing::Return(odometry::WheelMotion{ -40, -1.25f }));
    EXPECT_CALL(wheelOdometry, Chassis()).WillOnce(testing::Return(odometry::ChassisMotion{ 0.04f, 0.5f }));

    Send("odom");

    EXPECT_THAT(Output(), testing::HasSubstr("left 120 counts"));
    EXPECT_THAT(Output(), testing::HasSubstr("right -40 counts"));
    EXPECT_THAT(Output(), testing::HasSubstr("chassis"));
}

TEST_F(CliTest, imu_reports_the_latest_sample_and_its_validity)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    sensing::Measurement measurement;
    measurement.angularRate = { 0.5f, -0.25f, 0.125f };
    measurement.acceleration = { 0.0f, 0.0f, -9.80665f };
    measurement.valid = true;
    measurement.cause = sensing::InvalidCause::none;

    EXPECT_CALL(inertialSensing, Latest()).WillOnce(testing::Return(measurement));

    Send("imu");

    EXPECT_THAT(Output(), testing::HasSubstr("rate"));
    EXPECT_THAT(Output(), testing::HasSubstr("accel"));
    EXPECT_THAT(Output(), testing::HasSubstr("valid yes"));
}

TEST_F(CliTest, imu_reports_an_invalid_sample_as_such)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    sensing::Measurement stale;
    stale.cause = sensing::InvalidCause::stale;

    EXPECT_CALL(inertialSensing, Latest()).WillOnce(testing::Return(stale));

    Send("imu");

    EXPECT_THAT(Output(), testing::HasSubstr("valid no"));
}

TEST_F(CliTest, calibrate_asks_the_supervisor_to_recalibrate)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(supervisor, Calibrate()).WillOnce(testing::Return(true));

    Send("calibrate");

    EXPECT_THAT(Output(), testing::HasSubstr("calibrating"));
}

TEST_F(CliTest, calibrate_outside_idle_is_refused)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(supervisor, Calibrate()).WillOnce(testing::Return(false));

    Send("calibrate");

    EXPECT_THAT(Output(), testing::HasSubstr("refused: calibrate only from idle"));
}

TEST_F(CliTest, clear_releases_the_latched_fault)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(supervisor, ClearFault()).WillOnce(testing::Return(true));

    Send("clear");

    EXPECT_THAT(Output(), testing::HasSubstr("fault cleared"));
}

TEST_F(CliTest, clear_without_a_fault_is_refused)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(supervisor, ClearFault()).WillOnce(testing::Return(false));

    Send("clear");

    EXPECT_THAT(Output(), testing::HasSubstr("refused: no fault latched"));
}

TEST_F(CliTest, driver_reports_state_and_latched_fault)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(motionActuation, State()).WillOnce(testing::Return(motion::DriverState::failed));
    EXPECT_CALL(motionActuation, Fault()).WillOnce(testing::Return(motion::FaultCause::driverFault));

    Send("driver");

    EXPECT_THAT(Output(), testing::HasSubstr("driver failed fault driver"));
}

TEST_F(CliTest, attitude_reports_the_estimate_and_the_selected_filter)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(attitudeEstimation, Latest()).WillOnce(testing::Return(estimation::Estimate{ 0.25f, -0.5f, false, estimation::InvalidCause::converging }));
    EXPECT_CALL(attitudeEstimation, Selected()).WillOnce(testing::Return(estimation::Filter::kalman));

    Send("attitude");

    EXPECT_THAT(Output(), testing::HasSubstr("pitch 0.25"));
    EXPECT_THAT(Output(), testing::HasSubstr("rate -0.5"));
    EXPECT_THAT(Output(), testing::HasSubstr("filter kalman valid no cause converging"));
}

TEST_F(CliTest, filter_without_argument_reports_the_selected_filter)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(attitudeEstimation, Selected()).WillOnce(testing::Return(estimation::Filter::complementary));

    Send("filter");

    EXPECT_THAT(Output(), testing::HasSubstr("filter complementary"));
}

TEST_F(CliTest, filter_selects_kalman)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(attitudeEstimation, Select(estimation::Filter::kalman));

    Send("filter kalman");

    EXPECT_THAT(Output(), testing::HasSubstr("filter kalman selected"));
}

TEST_F(CliTest, filter_selects_complementary)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(attitudeEstimation, Select(estimation::Filter::complementary));

    Send("f complementary");

    EXPECT_THAT(Output(), testing::HasSubstr("filter complementary selected"));
}

TEST_F(CliTest, filter_rejects_an_unknown_name)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    Send("filter median");

    EXPECT_THAT(Output(), testing::HasSubstr("usage: filter"));
}

TEST_F(CliTest, loop_reports_timing_statistics)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(controlLoop, Statistics()).WillOnce(testing::Return(control::TimingStatistics{ 500, std::chrono::microseconds{ 120 }, 3 }));

    Send("loop");

    EXPECT_THAT(Output(), testing::HasSubstr("iterations 500 worst jitter 120 us late 3"));
}

TEST_F(CliTest, loop_reset_clears_timing_statistics)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(controlLoop, ResetStatistics());

    Send("loop reset");

    EXPECT_THAT(Output(), testing::HasSubstr("loop statistics reset"));
}

TEST_F(CliTest, arm_reports_an_accepted_request)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(supervisor, Arm()).WillOnce(testing::Return(true));

    Send("arm");

    EXPECT_THAT(Output(), testing::HasSubstr("armed"));
}

TEST_F(CliTest, arm_reports_a_refused_request)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(supervisor, Arm()).WillOnce(testing::Return(false));

    Send("arm");

    EXPECT_THAT(Output(), testing::HasSubstr("refused: arming needs idle"));
}

TEST_F(CliTest, disarm_reports_an_accepted_request)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(supervisor, Disarm()).WillOnce(testing::Return(true));

    Send("disarm");

    EXPECT_THAT(Output(), testing::HasSubstr("disarmed"));
}

TEST_F(CliTest, disarm_while_not_armed_is_refused)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(supervisor, Disarm()).WillOnce(testing::Return(false));

    Send("disarm");

    EXPECT_THAT(Output(), testing::HasSubstr("refused: not armed"));
}

TEST_F(CliTest, mode_reports_the_mode_and_latched_fault)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    EXPECT_CALL(supervisor, Current()).WillOnce(testing::Return(safety::Mode::fault));
    EXPECT_CALL(supervisor, LatchedCause()).WillOnce(testing::Return(safety::FaultCause::loopStalled));

    Send("mode");

    EXPECT_THAT(Output(), testing::HasSubstr("mode fault fault loop"));
}
