#include "targets/platform_implementations/st/MotorBridgeStm.hpp"
#include "core/platform_abstraction/SignMagnitude.hpp"

namespace application
{
    MotorBridgeStm::MotorBridgeStm(uint8_t timerOneBasedIndex, hal::GpioPinStm& pwmPin, hal::GpioPinStm& breakPin, hal::GpioPin& directionPin, const hal::PwmStmBase::Config& config)
        : channels{ { { 1, pwmPin } } }
        , pwm(timerOneBasedIndex, channels, breakPin, config)
        , direction(directionPin)
    {}

    void MotorBridgeStm::SetBaseFrequency(hal::Hertz baseFrequency)
    {
        pwm.SetBaseFrequency(baseFrequency);
    }

    void MotorBridgeStm::Start(hal::DutyCycle input1, hal::DutyCycle input2)
    {
        const auto command = platform::AsSignMagnitude(input1, input2);

        if (command.secondInputHigh != directionHigh)
        {
            pwm.Start(hal::DutyCycle::FromPercent(0));
            direction.Set(command.secondInputHigh);
            directionHigh = command.secondInputHigh;
        }

        pwm.Start(command.dutyCycle);
    }

    void MotorBridgeStm::Stop()
    {
        Start(hal::DutyCycle::FromPercent(0), hal::DutyCycle::FromPercent(0));
    }
}
