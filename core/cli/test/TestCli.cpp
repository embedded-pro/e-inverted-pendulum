#include "core/cli/Cli.hpp"
#include "core/inertial_sensing/interfaces/InertialSensing.hpp"
#include "core/motion_actuation/interfaces/MotionActuation.hpp"
#include "core/platform_abstraction/test_doubles/PlatformMock.hpp"
#include "core/wheel_odometry/interfaces/WheelOdometry.hpp"
#include "hal/interfaces/test_doubles/SerialCommunicationMock.hpp"
#include "infra/stream/StringOutputStream.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "services/tracer/Tracer.hpp"
#include "gmock/gmock.h"
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
        MOCK_METHOD(sensing::InvalidCause, Cause, (), (const, override));
        MOCK_METHOD(void, StartCalibration, (), (override));
        MOCK_METHOD(sensing::CalibrationState, Calibration, (), (const, override));
        MOCK_METHOD(platform::InertialAxes, GyroscopeBias, (), (const, override));
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
    };
}

TEST_F(CliTest, greets_and_shows_a_prompt_on_construction)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

    EXPECT_THAT(Output(), testing::HasSubstr("ready"));
    EXPECT_THAT(Output(), testing::HasSubstr("drive"));
    EXPECT_THAT(Output(), testing::HasSubstr("> "));
}

TEST_F(CliTest, ping_replies_pong)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

    Send("ping");

    EXPECT_THAT(Output(), testing::HasSubstr("pong"));
}

TEST_F(CliTest, id_prints_the_board_identifier)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

    Send("id");

    EXPECT_THAT(Output(), testing::HasSubstr("inverted-pendulum-bot cli"));
}

TEST_F(CliTest, drive_applies_both_efforts)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

    EXPECT_CALL(motionActuation, State()).WillOnce(testing::Return(motion::DriverState::ready));
    EXPECT_CALL(motionActuation, Fault()).WillOnce(testing::Return(motion::FaultCause::none));
    EXPECT_CALL(motionActuation, Apply(testing::FloatEq(0.3f), testing::FloatEq(-0.7f)));

    Send("drive 0.3 -0.7");

    EXPECT_THAT(Output(), testing::HasSubstr("driving"));
}

TEST_F(CliTest, drive_without_a_second_argument_prints_usage)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

    EXPECT_CALL(motionActuation, State()).WillOnce(testing::Return(motion::DriverState::ready));
    EXPECT_CALL(motionActuation, Fault()).WillOnce(testing::Return(motion::FaultCause::none));

    Send("drive 0.3");

    EXPECT_THAT(Output(), testing::HasSubstr("usage: drive"));
}

TEST_F(CliTest, drive_is_refused_while_a_fault_is_latched)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

    EXPECT_CALL(motionActuation, State()).WillOnce(testing::Return(motion::DriverState::ready));
    EXPECT_CALL(motionActuation, Fault()).WillOnce(testing::Return(motion::FaultCause::driverFault));

    Send("drive 0.3 -0.7");

    EXPECT_THAT(Output(), testing::HasSubstr("refused"));
}

TEST_F(CliTest, tristate_releases_the_bridges)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

    EXPECT_CALL(motionActuation, Disable(motion::DisableState::tristate));

    Send("tristate");

    EXPECT_THAT(Output(), testing::HasSubstr("tristated"));
}

TEST_F(CliTest, brake_shorts_the_motors)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

    EXPECT_CALL(motionActuation, Disable(motion::DisableState::brake));

    Send("brake");

    EXPECT_THAT(Output(), testing::HasSubstr("braking"));
}

TEST_F(CliTest, drive_with_a_missing_right_argument_is_rejected)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

    EXPECT_CALL(motionActuation, State()).WillOnce(testing::Return(motion::DriverState::ready));
    EXPECT_CALL(motionActuation, Fault()).WillOnce(testing::Return(motion::FaultCause::none));

    Send("drive 0.3 ");

    EXPECT_THAT(Output(), testing::HasSubstr("usage: drive"));
}

TEST_F(CliTest, drive_with_a_non_numeric_argument_is_rejected)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

    EXPECT_CALL(motionActuation, State()).WillOnce(testing::Return(motion::DriverState::ready));
    EXPECT_CALL(motionActuation, Fault()).WillOnce(testing::Return(motion::FaultCause::none));

    Send("drive abc def");

    EXPECT_THAT(Output(), testing::HasSubstr("usage: drive"));
}

TEST_F(CliTest, drive_with_a_trailing_third_argument_is_rejected)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

    EXPECT_CALL(motionActuation, State()).WillOnce(testing::Return(motion::DriverState::ready));
    EXPECT_CALL(motionActuation, Fault()).WillOnce(testing::Return(motion::FaultCause::none));

    Send("drive 0.3 0.7 0.9");

    EXPECT_THAT(Output(), testing::HasSubstr("usage: drive"));
}

TEST_F(CliTest, drive_with_a_partially_numeric_argument_is_rejected)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

    EXPECT_CALL(motionActuation, State()).WillOnce(testing::Return(motion::DriverState::ready));
    EXPECT_CALL(motionActuation, Fault()).WillOnce(testing::Return(motion::FaultCause::none));

    Send("drive 0.3 0.7x");

    EXPECT_THAT(Output(), testing::HasSubstr("usage: drive"));
}

TEST_F(CliTest, drive_with_an_over_long_argument_is_rejected)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

    EXPECT_CALL(motionActuation, State()).WillOnce(testing::Return(motion::DriverState::ready));
    EXPECT_CALL(motionActuation, Fault()).WillOnce(testing::Return(motion::FaultCause::none));

    Send("drive 0.3 0.70000000000000");

    EXPECT_THAT(Output(), testing::HasSubstr("usage: drive"));
}

TEST_F(CliTest, odom_reports_both_wheels_and_the_chassis)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

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
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

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
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

    sensing::Measurement stale;
    stale.cause = sensing::InvalidCause::stale;

    EXPECT_CALL(inertialSensing, Latest()).WillOnce(testing::Return(stale));

    Send("imu");

    EXPECT_THAT(Output(), testing::HasSubstr("valid no"));
}

TEST_F(CliTest, calibrate_starts_bias_calibration)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

    EXPECT_CALL(inertialSensing, StartCalibration());

    Send("calibrate");

    EXPECT_THAT(Output(), testing::HasSubstr("calibrating"));
}

TEST_F(CliTest, drive_is_refused_until_the_driver_is_ready)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

    EXPECT_CALL(motionActuation, State()).WillOnce(testing::Return(motion::DriverState::configuring));

    Send("drive 0.3 -0.7");

    EXPECT_THAT(Output(), testing::HasSubstr("refused: motor driver not ready"));
}

TEST_F(CliTest, clear_releases_a_latched_fault)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

    EXPECT_CALL(motionActuation, ClearFault());

    Send("clear");

    EXPECT_THAT(Output(), testing::HasSubstr("fault cleared"));
}

TEST_F(CliTest, driver_reports_state_and_latched_fault)
{
    application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

    EXPECT_CALL(motionActuation, State()).WillOnce(testing::Return(motion::DriverState::failed));
    EXPECT_CALL(motionActuation, Fault()).WillOnce(testing::Return(motion::FaultCause::driverFault));

    Send("driver");

    EXPECT_THAT(Output(), testing::HasSubstr("driver failed fault driver"));
}
