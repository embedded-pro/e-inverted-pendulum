#include "core/cli/Cli.hpp"
#include "core/cli/DecimalParser.hpp"
#include "infra/util/Tokenizer.hpp"
#include <array>
#include <chrono>
#include <optional>

namespace application
{
    namespace
    {
        const char* NameOf(motion::DriverState state)
        {
            switch (state)
            {
                case motion::DriverState::configuring:
                    return "configuring";
                case motion::DriverState::ready:
                    return "ready";
                default:
                    return "failed";
            }
        }

        const char* NameOf(motion::FaultCause cause)
        {
            if (cause == motion::FaultCause::driverFault)
                return "driver";

            return "none";
        }

        const char* NameOf(estimation::Filter filter)
        {
            if (filter == estimation::Filter::kalman)
                return "kalman";

            return "complementary";
        }

        const char* NameOf(estimation::InvalidCause cause)
        {
            switch (cause)
            {
                case estimation::InvalidCause::none:
                    return "none";
                case estimation::InvalidCause::converging:
                    return "converging";
                default:
                    return "sensing";
            }
        }

        const char* NameOf(safety::Mode mode)
        {
            switch (mode)
            {
                case safety::Mode::init:
                    return "init";
                case safety::Mode::calibrating:
                    return "calibrating";
                case safety::Mode::idle:
                    return "idle";
                case safety::Mode::armed:
                    return "armed";
                default:
                    return "fault";
            }
        }

        const char* NameOf(safety::FaultCause cause)
        {
            switch (cause)
            {
                case safety::FaultCause::fall:
                    return "fall";
                case safety::FaultCause::driverFault:
                    return "driver";
                case safety::FaultCause::estimateInvalid:
                    return "estimate";
                case safety::FaultCause::loopStalled:
                    return "loop";
                case safety::FaultCause::selfTestFailed:
                    return "selftest";
                case safety::FaultCause::calibrationFailed:
                    return "calibration";
                default:
                    return "none";
            }
        }

        std::optional<estimation::Filter> ParseFilter(const infra::BoundedConstString& name)
        {
            if (name == "complementary")
                return estimation::Filter::complementary;

            if (name == "kalman")
                return estimation::Filter::kalman;

            return std::nullopt;
        }

        uint32_t Microseconds(infra::Duration duration)
        {
            return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(duration).count());
        }
    }

    Cli::Cli(platform::Platform& platform, motion::MotionActuation& motionActuation, odometry::WheelOdometry& wheelOdometry, sensing::InertialSensing& inertialSensing, estimation::AttitudeEstimation& attitudeEstimation, control::ControlLoop& controlLoop, safety::SafetySupervisor& supervisor)
        : debugLed{ platform.StatusLed() }
        , terminal{ platform.Communication(), platform.Tracer() }
        , commands{ terminal, platform.Tracer(), motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor }
    {
        platform.Tracer().Trace() << "inverted-pendulum-bot ready - try 'mode', 'attitude', 'arm' or 'help'";
    }

    Cli::CliCommands::CliCommands(services::TerminalWithCommands& terminal, services::Tracer& tracer, motion::MotionActuation& motionActuation, odometry::WheelOdometry& wheelOdometry, sensing::InertialSensing& inertialSensing, estimation::AttitudeEstimation& attitudeEstimation, control::ControlLoop& controlLoop, safety::SafetySupervisor& supervisor)
        : services::TerminalCommands(terminal)
        , tracer(tracer)
        , motionActuation(motionActuation)
        , wheelOdometry(wheelOdometry)
        , inertialSensing(inertialSensing)
        , attitudeEstimation(attitudeEstimation)
        , controlLoop(controlLoop)
        , supervisor(supervisor)
        , commands{ {
              { { "ping", "p", "reply with pong" },
                  [this](const infra::BoundedConstString& params)
                  {
                      Ping(params);
                  } },
              { { "id", "i", "print the board identifier" },
                  [this](const infra::BoundedConstString& params)
                  {
                      Identify(params);
                  } },
              { { "drive", "d", "apply signed effort to both wheels, -1.0 to 1.0, while armed" },
                  [this](const infra::BoundedConstString& params)
                  {
                      Drive(params);
                  } },
              { { "tristate", "t", "release both bridges to high impedance" },
                  [this](const infra::BoundedConstString& params)
                  {
                      ReleaseBridges(params);
                  } },
              { { "brake", "b", "short both motors" },
                  [this](const infra::BoundedConstString& params)
                  {
                      Brake(params);
                  } },
              { { "odom", "o", "print wheel and chassis motion" },
                  [this](const infra::BoundedConstString& params)
                  {
                      Odometry(params);
                  } },
              { { "imu", "m", "print the latest inertial sample" },
                  [this](const infra::BoundedConstString& params)
                  {
                      Imu(params);
                  } },
              { { "calibrate", "c", "recalibrate the gyroscope bias, from idle only" },
                  [this](const infra::BoundedConstString& params)
                  {
                      Calibrate(params);
                  } },
              { { "clear", "x", "clear the latched fault and return to idle" },
                  [this](const infra::BoundedConstString& params)
                  {
                      ClearFault(params);
                  } },
              { { "driver", "v", "print the motor driver state and latched fault" },
                  [this](const infra::BoundedConstString& params)
                  {
                      DriverStatus(params);
                  } },
              { { "attitude", "a", "print the pitch estimate" },
                  [this](const infra::BoundedConstString& params)
                  {
                      Attitude(params);
                  } },
              { { "filter", "f", "select the attitude filter: complementary or kalman" },
                  [this](const infra::BoundedConstString& params)
                  {
                      SelectFilter(params);
                  } },
              { { "loop", "l", "print control loop timing, 'loop reset' clears it" },
                  [this](const infra::BoundedConstString& params)
                  {
                      LoopTiming(params);
                  } },
              { { "arm", "r", "arm the drive: needs idle, upright and a valid estimate" },
                  [this](const infra::BoundedConstString& params)
                  {
                      Arm(params);
                  } },
              { { "disarm", "s", "disarm the drive and tristate both bridges" },
                  [this](const infra::BoundedConstString& params)
                  {
                      Disarm(params);
                  } },
              { { "mode", "e", "print the operating mode and latched fault" },
                  [this](const infra::BoundedConstString& params)
                  {
                      ReportMode(params);
                  } },
          } }
    {}

    infra::MemoryRange<const services::TerminalCommands::Command> Cli::CliCommands::Commands()
    {
        return infra::MakeRange(commands);
    }

    void Cli::CliCommands::Ping(const infra::BoundedConstString&)
    {
        tracer.Trace() << "pong";
    }

    void Cli::CliCommands::Identify(const infra::BoundedConstString&)
    {
        tracer.Trace() << "inverted-pendulum-bot cli";
    }

    void Cli::CliCommands::Drive(const infra::BoundedConstString& params)
    {
        if (!supervisor.DrivePermitted())
        {
            tracer.Trace() << "refused: not armed";
            return;
        }

        const infra::Tokenizer tokenizer{ params, ' ' };
        std::optional<float> effortLeft;
        std::optional<float> effortRight;

        if (tokenizer.Size() == 2)
        {
            effortLeft = ParseDecimal(tokenizer.Token(0));
            effortRight = ParseDecimal(tokenizer.Token(1));
        }

        if (!effortLeft.has_value() || !effortRight.has_value())
        {
            tracer.Trace() << "usage: drive <left> <right>";
            return;
        }

        motionActuation.Apply(*effortLeft, *effortRight);
        tracer.Trace() << "driving";
    }

    void Cli::CliCommands::ReleaseBridges(const infra::BoundedConstString&)
    {
        motionActuation.Disable(motion::DisableState::tristate);
        tracer.Trace() << "tristated";
    }

    void Cli::CliCommands::Brake(const infra::BoundedConstString&)
    {
        motionActuation.Disable(motion::DisableState::brake);
        tracer.Trace() << "braking";
    }

    void Cli::CliCommands::Odometry(const infra::BoundedConstString&)
    {
        const auto left = wheelOdometry.Left();
        const auto right = wheelOdometry.Right();
        const auto chassis = wheelOdometry.Chassis();

        tracer.Trace() << "left " << left.position << " counts " << left.angularVelocity << " rad/s";
        tracer.Trace() << "right " << right.position << " counts " << right.angularVelocity << " rad/s";
        tracer.Trace() << "chassis " << chassis.forwardVelocity << " m/s " << chassis.yawRate << " rad/s";
    }

    void Cli::CliCommands::Imu(const infra::BoundedConstString&)
    {
        const auto measurement = inertialSensing.Latest();

        tracer.Trace() << "rate " << measurement.angularRate.x << " " << measurement.angularRate.y << " " << measurement.angularRate.z << " rad/s";
        tracer.Trace() << "accel " << measurement.acceleration.x << " " << measurement.acceleration.y << " " << measurement.acceleration.z << " m/s2";
        tracer.Trace() << "valid " << (measurement.valid ? "yes" : "no") << " cause " << static_cast<uint32_t>(measurement.cause);
    }

    void Cli::CliCommands::Calibrate(const infra::BoundedConstString&)
    {
        if (!supervisor.Calibrate())
        {
            tracer.Trace() << "refused: calibrate only from idle";
            return;
        }

        tracer.Trace() << "calibrating - hold the robot still";
    }

    void Cli::CliCommands::ClearFault(const infra::BoundedConstString&)
    {
        if (!supervisor.ClearFault())
        {
            tracer.Trace() << "refused: no fault latched";
            return;
        }

        tracer.Trace() << "fault cleared";
    }

    void Cli::CliCommands::DriverStatus(const infra::BoundedConstString&)
    {
        tracer.Trace() << "driver " << NameOf(motionActuation.State()) << " fault " << NameOf(motionActuation.Fault());
    }

    void Cli::CliCommands::Attitude(const infra::BoundedConstString&)
    {
        const auto estimate = attitudeEstimation.Latest();

        tracer.Trace() << "pitch " << estimate.pitch << " rad rate " << estimate.pitchRate << " rad/s";
        tracer.Trace() << "filter " << NameOf(attitudeEstimation.Selected()) << " valid " << (estimate.valid ? "yes" : "no") << " cause " << NameOf(estimate.cause);
    }

    void Cli::CliCommands::SelectFilter(const infra::BoundedConstString& params)
    {
        if (params.empty())
        {
            tracer.Trace() << "filter " << NameOf(attitudeEstimation.Selected());
            return;
        }

        const auto filter = ParseFilter(params);

        if (!filter.has_value())
        {
            tracer.Trace() << "usage: filter [complementary|kalman]";
            return;
        }

        attitudeEstimation.Select(*filter);
        tracer.Trace() << "filter " << NameOf(*filter) << " selected - estimate converging";
    }

    void Cli::CliCommands::LoopTiming(const infra::BoundedConstString& params)
    {
        if (params == "reset")
        {
            controlLoop.ResetStatistics();
            tracer.Trace() << "loop statistics reset";
            return;
        }

        const auto statistics = controlLoop.Statistics();

        tracer.Trace() << "iterations " << statistics.iterations << " worst jitter " << Microseconds(statistics.worstJitter) << " us late " << statistics.lateIterations;
    }

    void Cli::CliCommands::Arm(const infra::BoundedConstString&)
    {
        if (!supervisor.Arm())
        {
            tracer.Trace() << "refused: arming needs idle, upright within 5 deg, a valid estimate and a healthy driver";
            return;
        }

        tracer.Trace() << "armed";
    }

    void Cli::CliCommands::Disarm(const infra::BoundedConstString&)
    {
        if (!supervisor.Disarm())
        {
            tracer.Trace() << "refused: not armed";
            return;
        }

        tracer.Trace() << "disarmed";
    }

    void Cli::CliCommands::ReportMode(const infra::BoundedConstString&)
    {
        tracer.Trace() << "mode " << NameOf(supervisor.Current()) << " fault " << NameOf(supervisor.LatchedCause());
    }
}
