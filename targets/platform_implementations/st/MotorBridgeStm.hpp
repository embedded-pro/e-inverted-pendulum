#pragma once

#include "core/platform_abstraction/MotorBridge.hpp"
#include "hal/interfaces/Gpio.hpp"
#include "hal_st/stm32fxxx/GpioStm.hpp"
#include "hal_st/synchronous_stm32fxxx/SynchronousPwmStm.hpp"
#include <array>

namespace application
{
    class MotorBridgeStm final
        : public platform::MotorBridge
    {
    public:
        MotorBridgeStm(uint8_t timerOneBasedIndex, hal::GpioPinStm& pwmPin, hal::GpioPinStm& breakPin, hal::GpioPin& directionPin, const hal::PwmStmBase::Config& config);

        void SetBaseFrequency(hal::Hertz baseFrequency) override;
        void Start(hal::DutyCycle input1, hal::DutyCycle input2) override;
        void Stop() override;

    private:
        std::array<hal::PwmStmBase::ChannelConfig, 1> channels;
        hal::SynchronousPwmStm pwm;
        hal::OutputPin direction;
        bool directionHigh{ false };
    };
}
