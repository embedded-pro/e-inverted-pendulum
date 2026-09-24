#pragma once

#include "core/inertial_sensing/interfaces/InertialSensing.hpp"
#include "core/motion_actuation/interfaces/MotionActuation.hpp"
#include "core/platform_abstraction/Platform.hpp"
#include "core/wheel_odometry/interfaces/WheelOdometry.hpp"
#include "services/peripheral/DebugLed.hpp"
#include "services/util/Terminal.hpp"
#include <array>

namespace application
{
    class Cli
    {
    public:
        Cli(platform::Platform& platform, motion::MotionActuation& motionActuation, odometry::WheelOdometry& wheelOdometry, sensing::InertialSensing& inertialSensing);

    private:
        class CliCommands final
            : public services::TerminalCommands
        {
        public:
            CliCommands(services::TerminalWithCommands& terminal, services::Tracer& tracer, motion::MotionActuation& motionActuation, odometry::WheelOdometry& wheelOdometry, sensing::InertialSensing& inertialSensing);

            infra::MemoryRange<const Command> Commands() override;

        private:
            void Ping(const infra::BoundedConstString& params);
            void Identify(const infra::BoundedConstString& params);
            void Drive(const infra::BoundedConstString& params);
            void ReleaseBridges(const infra::BoundedConstString& params);
            void Brake(const infra::BoundedConstString& params);
            void Odometry(const infra::BoundedConstString& params);
            void Imu(const infra::BoundedConstString& params);
            void Calibrate(const infra::BoundedConstString& params);
            void ClearFault(const infra::BoundedConstString& params);
            void DriverStatus(const infra::BoundedConstString& params);

            services::Tracer& tracer;
            motion::MotionActuation& motionActuation;
            odometry::WheelOdometry& wheelOdometry;
            sensing::InertialSensing& inertialSensing;

            std::array<Command, 10> commands;
        };

        services::DebugLed debugLed;
        services::TerminalWithCommandsImpl::WithMaxQueueAndMaxHistory<> terminal;
        CliCommands commands;
    };
}
