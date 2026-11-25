#pragma once

#include "core/platform_abstraction/MotorDriver.hpp"
#include "core/platform_abstraction/UnusedAnalogToDigitalPin.hpp"
#include "targets/platform_implementations/host/Drv8711Emulator.hpp"

namespace integration
{
    class MotorDriverFake final
        : public platform::MotorDriver
    {
    public:
        void SetBaseFrequency(hal::Hertz baseFrequency) override;
        void Drive(const platform::BridgeInputs& left, const platform::BridgeInputs& right) override;
        drivers::DirectPwmStepperMotorDrv8711Decorator& Controller() override;
        void EnableFaultNotification(const infra::Function<void()>& onFault) override;
        void DisableFaultNotification() override;

        void RaiseFault();
        bool Released() const;
        bool Braked() const;
        bool AnyBraked() const;
        bool EnergisedSinceMark() const;
        void Mark();

    private:
        application::Drv8711Emulator registers;
        platform::UnusedAnalogToDigitalPin backEmf;
        drivers::DirectPwmStepperMotorDrv8711Decorator controller{ registers, hal::dummyPin, hal::dummyPin, backEmf };
        infra::Function<void()> onFault;
        platform::BridgeInputs left{};
        platform::BridgeInputs right{};
        bool energised{ false };
    };
}
