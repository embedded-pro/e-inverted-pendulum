#include "core/cli/Cli.hpp"
#include "infra/util/Tokenizer.hpp"
#include <algorithm>
#include <array>
#include <cstdlib>
#include <optional>

namespace application
{
    namespace
    {
        std::optional<float> ParseEffort(const infra::BoundedConstString& token)
        {
            std::array<char, 16> buffer{};
            if (token.size() >= buffer.size())
                return std::nullopt;

            std::copy_n(token.begin(), token.size(), buffer.begin());

            char* end = nullptr;
            const auto value = std::strtof(buffer.data(), &end);

            if (end != buffer.data() + token.size())
                return std::nullopt;

            return value;
        }

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
    }

    Cli::Cli(platform::Platform& platform, motion::MotionActuation& motionActuation, odometry::WheelOdometry& wheelOdometry, sensing::InertialSensing& inertialSensing)
        : debugLed{ platform.StatusLed() }
        , terminal{ platform.Communication(), platform.Tracer() }
        , commands{ terminal, platform.Tracer(), motionActuation, wheelOdometry, inertialSensing }
    {
        platform.Tracer().Trace() << "inverted-pendulum-bot ready - try 'ping', 'id', 'drive <left> <right>' or 'odom'";
    }

    Cli::CliCommands::CliCommands(services::TerminalWithCommands& terminal, services::Tracer& tracer, motion::MotionActuation& motionActuation, odometry::WheelOdometry& wheelOdometry, sensing::InertialSensing& inertialSensing)
        : services::TerminalCommands(terminal)
        , tracer(tracer)
        , motionActuation(motionActuation)
        , wheelOdometry(wheelOdometry)
        , inertialSensing(inertialSensing)
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
              { { "drive", "d", "apply signed effort to both wheels, -1.0 to 1.0" },
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
              { { "calibrate", "c", "start gyroscope bias calibration" },
                  [this](const infra::BoundedConstString& params)
                  {
                      Calibrate(params);
                  } },
              { { "clear", "x", "clear a latched driver fault" },
                  [this](const infra::BoundedConstString& params)
                  {
                      ClearFault(params);
                  } },
              { { "driver", "v", "print the motor driver state and latched fault" },
                  [this](const infra::BoundedConstString& params)
                  {
                      DriverStatus(params);
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
        if (motionActuation.State() != motion::DriverState::ready)
        {
            tracer.Trace() << "refused: motor driver not ready";
            return;
        }

        if (motionActuation.Fault() != motion::FaultCause::none)
        {
            tracer.Trace() << "refused: driver fault latched, clear it first";
            return;
        }

        const infra::Tokenizer tokenizer{ params, ' ' };
        std::optional<float> effortLeft;
        std::optional<float> effortRight;

        if (tokenizer.Size() == 2)
        {
            effortLeft = ParseEffort(tokenizer.Token(0));
            effortRight = ParseEffort(tokenizer.Token(1));
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
        inertialSensing.StartCalibration();
        tracer.Trace() << "calibrating - hold the robot still";
    }

    void Cli::CliCommands::ClearFault(const infra::BoundedConstString&)
    {
        motionActuation.ClearFault();
        tracer.Trace() << "fault cleared";
    }

    void Cli::CliCommands::DriverStatus(const infra::BoundedConstString&)
    {
        tracer.Trace() << "driver " << NameOf(motionActuation.State()) << " fault " << NameOf(motionActuation.Fault());
    }
}
