#pragma once

#include "core/motion_actuation/interfaces/DriverConfiguration.hpp"
#include "drivers/motor_controller/DirectPwmStepperMotorDrv8711Decorator.hpp"
#include "infra/timer/Timer.hpp"
#include <cstdint>

namespace motion
{
    struct CurrentScaling
    {
        drivers::StepperMotorControllerDrv8711::Isgain senseGain;
        uint8_t torque;

        bool operator==(const CurrentScaling& other) const = default;
    };

    CurrentScaling ScaleCurrent(uint32_t senseResistanceMilliOhm, uint32_t tripCurrentMilliAmpere);

    class Drv8711DriverConfiguration final
        : public DriverConfiguration
    {
    public:
        struct Config
        {
            Config();

            uint32_t senseResistanceMilliOhm{ 50 };
            uint32_t tripCurrentMilliAmpere{ 1000 };
            drivers::StepperMotorControllerDrv8711::Dtime deadTime{ drivers::StepperMotorControllerDrv8711::Dtime::ns850 };
            drivers::StepperMotorControllerDrv8711::DecayMode currentDecay{ drivers::StepperMotorControllerDrv8711::DecayMode::slowMixed };
            infra::Duration wakeUpDelay{ std::chrono::milliseconds{ 1 } };
        };

        explicit Drv8711DriverConfiguration(drivers::DirectPwmStepperMotorDrv8711Decorator& driver, const Config& config = Config());

        void Configure(const infra::Function<void(bool verified)>& onDone) override;

        static drivers::StepperMotorControllerDrv8711::Configuration DriverConfigurationFor(const Config& config);

    private:
        drivers::DirectPwmStepperMotorDrv8711Decorator& driver;
        drivers::StepperMotorControllerDrv8711::Configuration configuration;
        infra::Duration wakeUpDelay;
        infra::TimerSingleShot wakeUpTimer;
        infra::Function<void(bool verified)> onDone;
    };
}
