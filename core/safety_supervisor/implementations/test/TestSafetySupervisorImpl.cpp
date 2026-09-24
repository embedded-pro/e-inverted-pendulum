#include "core/safety_supervisor/implementations/SafetySupervisorImpl.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <chrono>
#include <optional>

namespace
{
    using namespace std::chrono_literals;

    constexpr float degree{ 3.14159265f / 180.0f };

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

    class SupervisedControlMock
        : public control::SupervisedControl
    {
    public:
        virtual ~SupervisedControlMock() = default;

        MOCK_METHOD(void, Balance, (const estimation::Estimate& estimate, infra::Duration interval), (override));
        MOCK_METHOD(void, Steer, (infra::Duration interval), (override));
        MOCK_METHOD(void, Engage, (), (override));
        MOCK_METHOD(void, Disengage, (), (override));
    };

    estimation::Estimate EstimateAt(float pitch, bool valid = true)
    {
        return estimation::Estimate{ pitch, 0.0f, valid, valid ? estimation::InvalidCause::none : estimation::InvalidCause::converging };
    }

    class SafetySupervisorImplTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        SafetySupervisorImplTest()
        {
            EXPECT_CALL(actuation, State()).WillRepeatedly(testing::ReturnPointee(&driverState));
            EXPECT_CALL(actuation, Fault()).WillRepeatedly(testing::ReturnPointee(&driverFault));
            EXPECT_CALL(sensing, Cause()).WillRepeatedly(testing::ReturnPointee(&sensingCause));
            EXPECT_CALL(sensing, Calibration()).WillRepeatedly(testing::ReturnPointee(&calibration));
        }

        void Construct()
        {
            supervisor.emplace(actuation, sensing, strategy);
        }

        void BringToIdle()
        {
            Construct();
            EXPECT_CALL(sensing, StartCalibration());
            ForwardTime(2ms);
            ASSERT_EQ(safety::Mode::calibrating, supervisor->Current());

            calibration = sensing::CalibrationState::calibrated;
            ForwardTime(2ms);
            ASSERT_EQ(safety::Mode::idle, supervisor->Current());
        }

        void Service(const estimation::Estimate& estimate)
        {
            supervisor->Balance(estimate, 2ms);
            ForwardTime(2ms);
        }

        void BringToArmed()
        {
            BringToIdle();
            Service(EstimateAt(0.0f));

            EXPECT_CALL(strategy, Engage());
            ASSERT_TRUE(supervisor->Arm());
        }

        void ExpectTristate()
        {
            EXPECT_CALL(actuation, Disable(motion::DisableState::tristate)).WillOnce([this](motion::DisableState)
                {
                    modeWhenDisabled = supervisor->Current();
                });
        }

        void ExpectLeavingArmed()
        {
            testing::InSequence leavingArmed;
            ExpectTristate();
            EXPECT_CALL(strategy, Disengage());
        }

        testing::StrictMock<MotionActuationMock> actuation;
        testing::StrictMock<InertialSensingMock> sensing;
        testing::StrictMock<SupervisedControlMock> strategy;
        std::optional<safety::SafetySupervisorImpl> supervisor;

        motion::DriverState driverState{ motion::DriverState::ready };
        motion::FaultCause driverFault{ motion::FaultCause::none };
        sensing::InvalidCause sensingCause{ sensing::InvalidCause::uncalibrated };
        sensing::CalibrationState calibration{ sensing::CalibrationState::calibrating };
        std::optional<safety::Mode> modeWhenDisabled;
    };
}

TEST_F(SafetySupervisorImplTest, starts_in_init_with_the_drive_not_permitted)
{
    Construct();

    EXPECT_EQ(safety::Mode::init, supervisor->Current());
    EXPECT_EQ(safety::FaultCause::none, supervisor->LatchedCause());
    EXPECT_FALSE(supervisor->DrivePermitted());
}

TEST_F(SafetySupervisorImplTest, a_passed_self_test_starts_calibration)
{
    Construct();

    EXPECT_CALL(sensing, StartCalibration());
    ForwardTime(2ms);

    EXPECT_EQ(safety::Mode::calibrating, supervisor->Current());
}

TEST_F(SafetySupervisorImplTest, waits_for_the_driver_until_the_self_test_deadline)
{
    driverState = motion::DriverState::configuring;
    Construct();

    ForwardTime(998ms);
    EXPECT_EQ(safety::Mode::init, supervisor->Current());

    ExpectTristate();
    ForwardTime(2ms);

    EXPECT_EQ(safety::Mode::fault, supervisor->Current());
    EXPECT_EQ(safety::FaultCause::selfTestFailed, supervisor->LatchedCause());
}

TEST_F(SafetySupervisorImplTest, a_sensor_that_never_samples_fails_the_self_test)
{
    sensingCause = sensing::InvalidCause::neverSampled;
    Construct();

    ExpectTristate();
    ForwardTime(1s);

    EXPECT_EQ(safety::FaultCause::selfTestFailed, supervisor->LatchedCause());
}

TEST_F(SafetySupervisorImplTest, a_successful_calibration_enters_idle)
{
    BringToIdle();

    EXPECT_EQ(safety::FaultCause::none, supervisor->LatchedCause());
    EXPECT_FALSE(supervisor->DrivePermitted());
}

TEST_F(SafetySupervisorImplTest, a_failed_calibration_latches_a_calibration_fault)
{
    Construct();
    EXPECT_CALL(sensing, StartCalibration());
    ForwardTime(2ms);

    calibration = sensing::CalibrationState::failed;
    ExpectTristate();
    ForwardTime(2ms);

    EXPECT_EQ(safety::Mode::fault, supervisor->Current());
    EXPECT_EQ(safety::FaultCause::calibrationFailed, supervisor->LatchedCause());
}

TEST_F(SafetySupervisorImplTest, calibrate_from_idle_recalibrates)
{
    BringToIdle();
    calibration = sensing::CalibrationState::calibrating;

    EXPECT_CALL(sensing, StartCalibration());
    EXPECT_TRUE(supervisor->Calibrate());

    EXPECT_EQ(safety::Mode::calibrating, supervisor->Current());
}

TEST_F(SafetySupervisorImplTest, calibrate_is_refused_outside_idle)
{
    BringToArmed();

    EXPECT_FALSE(supervisor->Calibrate());
    EXPECT_EQ(safety::Mode::armed, supervisor->Current());
}

TEST_F(SafetySupervisorImplTest, REQ_SAFE_001_arming_near_upright_with_a_valid_estimate_engages_control_first)
{
    BringToIdle();
    Service(EstimateAt(4.0f * degree));

    std::optional<bool> permittedAtEngage;
    EXPECT_CALL(strategy, Engage()).WillOnce([this, &permittedAtEngage]()
        {
            permittedAtEngage = supervisor->DrivePermitted();
        });

    EXPECT_TRUE(supervisor->Arm());

    EXPECT_EQ(safety::Mode::armed, supervisor->Current());
    EXPECT_TRUE(supervisor->DrivePermitted());
    EXPECT_EQ(std::optional<bool>{ false }, permittedAtEngage);
}

TEST_F(SafetySupervisorImplTest, REQ_SAFE_001_arming_is_refused_when_tilted)
{
    BringToIdle();
    Service(EstimateAt(-6.0f * degree));

    EXPECT_FALSE(supervisor->Arm());
    EXPECT_EQ(safety::Mode::idle, supervisor->Current());
}

TEST_F(SafetySupervisorImplTest, REQ_SAFE_001_arming_is_refused_with_an_invalid_estimate)
{
    BringToIdle();
    Service(EstimateAt(0.0f, false));

    EXPECT_FALSE(supervisor->Arm());
    EXPECT_EQ(safety::Mode::idle, supervisor->Current());
}

TEST_F(SafetySupervisorImplTest, arming_is_refused_before_the_loop_has_serviced_the_supervisor)
{
    BringToIdle();

    EXPECT_FALSE(supervisor->Arm());
}

TEST_F(SafetySupervisorImplTest, arming_is_refused_when_the_loop_has_stopped)
{
    BringToIdle();
    Service(EstimateAt(0.0f));
    ForwardTime(2ms);

    EXPECT_FALSE(supervisor->Arm());
}

TEST_F(SafetySupervisorImplTest, arming_is_refused_while_the_driver_is_not_ready)
{
    BringToIdle();
    Service(EstimateAt(0.0f));
    driverState = motion::DriverState::failed;

    EXPECT_FALSE(supervisor->Arm());
}

TEST_F(SafetySupervisorImplTest, balance_iterations_reach_the_strategy_only_while_armed)
{
    BringToIdle();
    supervisor->Balance(EstimateAt(0.0f), 2ms);

    EXPECT_CALL(strategy, Engage());
    ASSERT_TRUE(supervisor->Arm());

    EXPECT_CALL(strategy, Balance(testing::Field(&estimation::Estimate::pitch, 0.1f), infra::Duration{ 2ms }));
    supervisor->Balance(EstimateAt(0.1f), 2ms);
}

TEST_F(SafetySupervisorImplTest, REQ_SAFE_002_a_recoverable_tilt_stays_armed)
{
    BringToArmed();

    EXPECT_CALL(strategy, Balance(testing::_, testing::_));
    supervisor->Balance(EstimateAt(30.0f * degree), 2ms);

    EXPECT_EQ(safety::Mode::armed, supervisor->Current());
}

TEST_F(SafetySupervisorImplTest, REQ_SAFE_002_REQ_SAFE_003_a_fall_tristates_before_leaving_armed)
{
    BringToArmed();

    ExpectLeavingArmed();
    supervisor->Balance(EstimateAt(-36.0f * degree), 2ms);

    EXPECT_EQ(std::optional<safety::Mode>{ safety::Mode::armed }, modeWhenDisabled);
    EXPECT_EQ(safety::Mode::fault, supervisor->Current());
    EXPECT_EQ(safety::FaultCause::fall, supervisor->LatchedCause());
    EXPECT_FALSE(supervisor->DrivePermitted());
}

TEST_F(SafetySupervisorImplTest, REQ_SAFE_007_an_invalid_estimate_while_armed_is_a_fault)
{
    BringToArmed();

    ExpectLeavingArmed();
    supervisor->Balance(EstimateAt(0.0f, false), 2ms);

    EXPECT_EQ(safety::FaultCause::estimateInvalid, supervisor->LatchedCause());
}

TEST_F(SafetySupervisorImplTest, REQ_SAFE_006_a_driver_fault_while_armed_is_caught_on_the_next_iteration)
{
    BringToArmed();
    driverFault = motion::FaultCause::driverFault;

    ExpectLeavingArmed();
    supervisor->Balance(EstimateAt(0.0f), 2ms);

    EXPECT_EQ(safety::FaultCause::driverFault, supervisor->LatchedCause());
}

TEST_F(SafetySupervisorImplTest, REQ_SAFE_006_a_driver_fault_while_idle_is_latched)
{
    BringToIdle();
    driverFault = motion::FaultCause::driverFault;

    ExpectTristate();
    ForwardTime(2ms);

    EXPECT_EQ(safety::Mode::fault, supervisor->Current());
    EXPECT_EQ(safety::FaultCause::driverFault, supervisor->LatchedCause());
}

TEST_F(SafetySupervisorImplTest, REQ_SAFE_008_two_missed_periods_stay_armed)
{
    BringToArmed();
    ForwardTime(2ms);
    ForwardTime(2ms);

    EXPECT_EQ(safety::Mode::armed, supervisor->Current());
}

TEST_F(SafetySupervisorImplTest, REQ_SAFE_008_three_missed_periods_are_a_stalled_loop)
{
    BringToArmed();
    ForwardTime(4ms);

    ExpectLeavingArmed();
    ForwardTime(2ms);

    EXPECT_EQ(safety::Mode::fault, supervisor->Current());
    EXPECT_EQ(safety::FaultCause::loopStalled, supervisor->LatchedCause());
}

TEST_F(SafetySupervisorImplTest, REQ_SAFE_008_a_serviced_loop_is_never_stalled)
{
    BringToArmed();

    EXPECT_CALL(strategy, Balance(testing::_, testing::_)).Times(50);
    for (int32_t iteration = 0; iteration < 50; ++iteration)
        Service(EstimateAt(0.0f));

    EXPECT_EQ(safety::Mode::armed, supervisor->Current());
}

TEST_F(SafetySupervisorImplTest, REQ_SAFE_004_a_fault_stays_latched_after_returning_upright)
{
    BringToArmed();
    ExpectLeavingArmed();
    supervisor->Balance(EstimateAt(40.0f * degree), 2ms);

    Service(EstimateAt(0.0f));
    Service(EstimateAt(0.0f));

    EXPECT_EQ(safety::Mode::fault, supervisor->Current());
    EXPECT_EQ(safety::FaultCause::fall, supervisor->LatchedCause());
    EXPECT_FALSE(supervisor->Arm());
}

TEST_F(SafetySupervisorImplTest, REQ_SAFE_005_clearing_a_fault_enters_idle_and_arming_needs_a_new_command)
{
    BringToArmed();
    ExpectLeavingArmed();
    supervisor->Balance(EstimateAt(40.0f * degree), 2ms);

    EXPECT_CALL(actuation, ClearFault());
    EXPECT_TRUE(supervisor->ClearFault());

    EXPECT_EQ(safety::Mode::idle, supervisor->Current());
    EXPECT_EQ(safety::FaultCause::none, supervisor->LatchedCause());
    EXPECT_FALSE(supervisor->DrivePermitted());

    Service(EstimateAt(0.0f));
    EXPECT_EQ(safety::Mode::idle, supervisor->Current());

    EXPECT_CALL(strategy, Engage());
    EXPECT_TRUE(supervisor->Arm());
}

TEST_F(SafetySupervisorImplTest, REQ_SAFE_003_disarm_tristates_before_leaving_armed)
{
    BringToArmed();

    ExpectLeavingArmed();
    EXPECT_TRUE(supervisor->Disarm());

    EXPECT_EQ(std::optional<safety::Mode>{ safety::Mode::armed }, modeWhenDisabled);
    EXPECT_EQ(safety::Mode::idle, supervisor->Current());
}

TEST_F(SafetySupervisorImplTest, rejected_requests_change_nothing)
{
    BringToIdle();

    EXPECT_FALSE(supervisor->Disarm());
    EXPECT_FALSE(supervisor->ClearFault());
    EXPECT_FALSE(supervisor->Arm());

    EXPECT_EQ(safety::Mode::idle, supervisor->Current());
    EXPECT_EQ(safety::FaultCause::none, supervisor->LatchedCause());
}

TEST_F(SafetySupervisorImplTest, outer_iterations_reach_control_only_while_armed)
{
    BringToIdle();
    supervisor->Steer(20ms);

    Service(EstimateAt(0.0f));
    EXPECT_CALL(strategy, Engage());
    ASSERT_TRUE(supervisor->Arm());

    EXPECT_CALL(strategy, Steer(infra::Duration{ 20ms }));
    supervisor->Steer(20ms);
}
