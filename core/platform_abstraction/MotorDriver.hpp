#pragma once

#include "drivers/motor_controller/DirectPwmStepperMotorDrv8711Decorator.hpp"
#include "hal/synchronous_interfaces/SynchronousPwm.hpp"
#include "infra/util/Function.hpp"

namespace platform
{
    struct BridgeInputs
    {
        hal::DutyCycle input1;
        hal::DutyCycle input2;

        bool operator==(const BridgeInputs& other) const = default;
    };

    class MotorDriver
    {
    public:
        MotorDriver() = default;
        MotorDriver(const MotorDriver& other) = delete;
        MotorDriver& operator=(const MotorDriver& other) = delete;

        virtual void SetBaseFrequency(hal::Hertz baseFrequency) = 0;
        virtual void Drive(const BridgeInputs& left, const BridgeInputs& right) = 0;

        virtual drivers::DirectPwmStepperMotorDrv8711Decorator& Controller() = 0;

        virtual void EnableFaultNotification(const infra::Function<void()>& onFault) = 0;
        virtual void DisableFaultNotification() = 0;

    protected:
        ~MotorDriver() = default;
    };
}
