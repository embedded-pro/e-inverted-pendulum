#pragma once

#include "core/attitude_estimation/implementations/AttitudeEstimationImpl.hpp"
#include "core/balance_control/implementations/BalanceControlImpl.hpp"
#include "core/balance_control/implementations/CascadedPidStrategy.hpp"
#include "core/balance_control/implementations/LqrStrategy.hpp"
#include "core/control_loop/implementations/ControlLoopImpl.hpp"
#include "core/inertial_sensing/implementations/InertialSensingImpl.hpp"
#include "core/motion_actuation/implementations/Drv8711DriverConfiguration.hpp"
#include "core/motion_actuation/implementations/MotionActuationImpl.hpp"
#include "core/safety_supervisor/implementations/SafetySupervisorImpl.hpp"
#include "core/wheel_odometry/implementations/WheelOdometryImpl.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "integration_tests/support/InertialSensorFake.hpp"
#include "integration_tests/support/MotorDriverFake.hpp"
#include "targets/platform_implementations/host/WheelEncodersStub.hpp"
#include <array>
#include <optional>

namespace integration
{
    class NoStoredCalibration final
        : public safety::StartupCondition
    {
    public:
        bool Satisfied() const override
        {
            return true;
        }
    };

    struct SafetyFixture
    {
        void Forward(infra::Duration duration);
        std::optional<infra::Duration> ForwardUntil(const infra::Function<bool()>& condition, infra::Duration limit);
        bool ModeHolds(safety::Mode mode, infra::Duration duration);
        void Calibrate();
        void Arm();
        void TipTo(float degrees);

        infra::ClockFixture clock;
        InertialSensorFake inertial;
        MotorDriverFake motors;
        application::WheelEncodersStub encoders;
        motion::Drv8711DriverConfiguration driverConfiguration{ motors.Controller() };
        motion::MotionActuationImpl motionActuation{ motors, driverConfiguration };
        odometry::WheelOdometryImpl wheelOdometry{ encoders };
        sensing::InertialSensingImpl inertialSensing{ inertial };
        estimation::AttitudeEstimationImpl attitudeEstimation;
        balance::CascadedPidStrategy cascadedPid;
        balance::LqrStrategy lqr;
        std::array<balance::ControlStrategy*, 2> strategies{ &cascadedPid, &lqr };
        balance::BalanceControlImpl balanceControl{ infra::MakeRange(strategies), motionActuation, wheelOdometry };
        NoStoredCalibration startup;
        safety::SafetySupervisorImpl supervisor{ motionActuation, inertialSensing, balanceControl, startup };
        control::ControlLoopImpl controlLoop{ inertialSensing, attitudeEstimation, supervisor, supervisor };
        std::optional<infra::TimePoint> thresholdCrossed;
        std::optional<safety::FaultCause> expectedCause;
    };
}
