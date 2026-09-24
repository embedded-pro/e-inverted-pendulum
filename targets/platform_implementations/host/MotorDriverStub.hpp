#pragma once

#include "core/platform_abstraction/MotorDriver.hpp"
#include "targets/platform_implementations/host/Drv8711Emulator.hpp"

namespace application
{
    class MotorDriverStub final
        : public platform::MotorDriver
    {
    public:
        void SetBaseFrequency(hal::Hertz) override
        {}

        void Drive(const platform::BridgeInputs& left, const platform::BridgeInputs& right) override
        {
            lastLeft = left;
            lastRight = right;
        }

        hal::SpiMaster& ConfigurationChannel() override
        {
            return driver;
        }

        void EnableFaultNotification(const infra::Function<void()>& onFault) override
        {
            this->onFault = onFault;
        }

        void DisableFaultNotification() override
        {
            onFault = nullptr;
        }

        const platform::BridgeInputs& LastLeft() const
        {
            return lastLeft;
        }

        const platform::BridgeInputs& LastRight() const
        {
            return lastRight;
        }

    private:
        Drv8711Emulator driver;
        platform::BridgeInputs lastLeft{};
        platform::BridgeInputs lastRight{};
        infra::Function<void()> onFault;
    };
}
