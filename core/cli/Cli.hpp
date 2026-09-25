#pragma once

#include "core/attitude_estimation/interfaces/AttitudeEstimation.hpp"
#include "core/balance_control/interfaces/BalanceControl.hpp"
#include "core/ble_link/interfaces/LinkStatus.hpp"
#include "core/control_loop/interfaces/ControlLoop.hpp"
#include "core/inertial_sensing/interfaces/InertialSensing.hpp"
#include "core/motion_actuation/interfaces/MotionActuation.hpp"
#include "core/platform_abstraction/Platform.hpp"
#include "core/safety_supervisor/interfaces/SafetySupervisor.hpp"
#include "core/wheel_odometry/interfaces/WheelOdometry.hpp"
#include "services/peripheral/DebugLed.hpp"
#include "services/util/Terminal.hpp"
#include <array>

namespace application
{
    class Cli
    {
    public:
        struct Dependencies
        {
            motion::MotionActuation& motionActuation;
            odometry::WheelOdometry& wheelOdometry;
            sensing::InertialSensing& inertialSensing;
            estimation::AttitudeEstimation& attitudeEstimation;
            control::ControlLoop& controlLoop;
            safety::SafetySupervisor& supervisor;
            balance::BalanceControl& balanceControl;
            const ble::LinkStatus& link;
        };

        Cli(platform::Platform& platform, const Dependencies& dependencies);

    private:
        class CliCommands final
            : public services::TerminalCommands
        {
        public:
            CliCommands(services::TerminalWithCommands& terminal, services::Tracer& tracer, const Dependencies& dependencies);

            infra::MemoryRange<const Command> Commands() override;

        private:
            void Ping(const infra::BoundedConstString& params);
            void Identify(const infra::BoundedConstString& params);
            void ReleaseBridges(const infra::BoundedConstString& params);
            void Brake(const infra::BoundedConstString& params);
            void Odometry(const infra::BoundedConstString& params);
            void Imu(const infra::BoundedConstString& params);
            void Calibrate(const infra::BoundedConstString& params);
            void ClearFault(const infra::BoundedConstString& params);
            void DriverStatus(const infra::BoundedConstString& params);
            void Attitude(const infra::BoundedConstString& params);
            void SelectFilter(const infra::BoundedConstString& params);
            void LoopTiming(const infra::BoundedConstString& params);
            void Arm(const infra::BoundedConstString& params);
            void Disarm(const infra::BoundedConstString& params);
            void ReportMode(const infra::BoundedConstString& params);
            void Move(const infra::BoundedConstString& params);
            void Strategy(const infra::BoundedConstString& params);
            void Param(const infra::BoundedConstString& params);
            void ListParameters();
            void Bluetooth(const infra::BoundedConstString& params);

            services::Tracer& tracer;
            motion::MotionActuation& motionActuation;
            odometry::WheelOdometry& wheelOdometry;
            sensing::InertialSensing& inertialSensing;
            estimation::AttitudeEstimation& attitudeEstimation;
            control::ControlLoop& controlLoop;
            safety::SafetySupervisor& supervisor;
            balance::BalanceControl& balanceControl;
            const ble::LinkStatus& link;

            std::array<Command, 19> commands;
        };

        services::DebugLed debugLed;
        services::TerminalWithCommandsImpl::WithMaxQueueAndMaxHistory<> terminal;
        CliCommands commands;
    };
}
