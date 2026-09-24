#pragma once

#include "core/control_loop/interfaces/Stages.hpp"
#include "core/inertial_sensing/interfaces/InertialSensing.hpp"
#include "core/motion_actuation/interfaces/MotionActuation.hpp"
#include "core/safety_supervisor/interfaces/SafetySupervisor.hpp"
#include "infra/timer/Timer.hpp"
#include <chrono>

namespace safety
{
    class SafetySupervisorImpl final
        : public SafetySupervisor
        , public control::BalanceStage
        , public control::OuterStage
    {
    public:
        struct Config
        {
            Config();

            float fallThreshold{ 35.0f * 3.14159265f / 180.0f };
            float armWindow{ 5.0f * 3.14159265f / 180.0f };
            std::chrono::milliseconds servicePeriod{ 2 };
            uint32_t missedServiceLimit{ 3 };
            std::chrono::milliseconds selfTestDeadline{ 1000 };
        };

        SafetySupervisorImpl(motion::MotionActuation& actuation, sensing::InertialSensing& sensing, control::SupervisedControl& control, const Config& config = Config());

        bool Arm() override;
        bool Disarm() override;
        bool ClearFault() override;
        bool Calibrate() override;

        Mode Current() const override;
        FaultCause LatchedCause() const override;
        bool DrivePermitted() const override;

        void Balance(const estimation::Estimate& estimate, infra::Duration interval) override;
        void Steer(infra::Duration interval) override;

    private:
        void Supervise();
        void SuperviseSelfTest();
        void SuperviseCalibration();
        void SuperviseLiveness();
        bool SelfTestPassed() const;
        bool LoopServiced() const;
        bool DriverHealthy() const;
        bool Upright() const;
        void StartCalibration();
        void LeaveArmed();
        void EnterFault(FaultCause cause);

        motion::MotionActuation& actuation;
        sensing::InertialSensing& sensing;
        control::SupervisedControl& control;
        Config config;

        Mode mode{ Mode::init };
        FaultCause cause{ FaultCause::none };
        estimation::Estimate latest;
        bool serviced{ false };
        uint32_t missedServices{ 0 };
        infra::TimePoint started;
        infra::TimerRepeating supervision;
    };
}
