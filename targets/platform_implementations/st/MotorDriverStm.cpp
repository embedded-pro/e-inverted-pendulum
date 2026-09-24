#include "targets/platform_implementations/st/MotorDriverStm.hpp"

namespace application
{
    hal::PwmStmBase::Config MotorDriverStm::PwmConfig()
    {
        hal::PwmStmBase::Config config;

        config.alignment = hal::PwmStmBase::Alignment::centerAlignedUpCounting;
        config.preloadEnabled = true;

        hal::PwmStmBase::BreakInput breakInput;
        breakInput.activeHigh = false;
        config.breakInput = breakInput;

        return config;
    }

    hal::SpiMasterStm::Config MotorDriverStm::SpiConfig()
    {
        hal::SpiMasterStm::Config config;

        config.msbFirst = true;
        config.polarityLow = true;
        config.phase1st = true;
        config.baudRatePrescaler = SPI_BAUDRATEPRESCALER_128;

        return config;
    }

    MotorDriverStm::MotorDriverStm()
        : channels{ { { 1, leftInput1 }, { 2, leftInput2 }, { 3, rightInput1 }, { 4, rightInput2 } } }
        , pwm{ 1, channels, faultBreak, PwmConfig() }
    {}

    void MotorDriverStm::SetBaseFrequency(hal::Hertz baseFrequency)
    {
        pwm.SetBaseFrequency(baseFrequency);
    }

    void MotorDriverStm::Drive(const platform::BridgeInputs& left, const platform::BridgeInputs& right)
    {
        pwm.Start(left.input1, left.input2, right.input1, right.input2);
    }

    hal::SpiMaster& MotorDriverStm::ConfigurationChannel()
    {
        return configurationChannel;
    }

    void MotorDriverStm::EnableFaultNotification(const infra::Function<void()>& onFault)
    {
        faultInterrupt.EnableInterrupt(onFault, hal::InterruptTrigger::fallingEdge);
    }

    void MotorDriverStm::DisableFaultNotification()
    {
        faultInterrupt.DisableInterrupt();
    }
}
