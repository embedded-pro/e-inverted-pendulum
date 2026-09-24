#pragma once

#include "core/platform_abstraction/MotorDriver.hpp"
#include "hal/interfaces/Gpio.hpp"
#include "hal_st/stm32fxxx/GpioStm.hpp"
#include "hal_st/stm32fxxx/SpiMasterStm.hpp"
#include "hal_st/synchronous_stm32fxxx/SynchronousPwmStm.hpp"
#include "services/peripheral/GpioPinInverted.hpp"
#include "services/peripheral/SpiMasterWithChipSelect.hpp"
#include <array>

namespace application
{
    class MotorDriverStm final
        : public platform::MotorDriver
    {
    public:
        MotorDriverStm();

        void SetBaseFrequency(hal::Hertz baseFrequency) override;
        void Drive(const platform::BridgeInputs& left, const platform::BridgeInputs& right) override;
        hal::SpiMaster& ConfigurationChannel() override;
        void EnableFaultNotification(const infra::Function<void()>& onFault) override;
        void DisableFaultNotification() override;

    private:
        static hal::PwmStmBase::Config PwmConfig();
        static hal::SpiMasterStm::Config SpiConfig();

        hal::GpioPinStm sleepPin{ hal::Port::B, 8 };
        hal::GpioPinStm resetPin{ hal::Port::B, 9 };
        hal::OutputPin awake{ sleepPin, true };
        hal::OutputPin reset{ resetPin, false };

        hal::GpioPinStm leftInput1{ hal::Port::A, 8 };
        hal::GpioPinStm leftInput2{ hal::Port::A, 9 };
        hal::GpioPinStm rightInput1{ hal::Port::A, 10 };
        hal::GpioPinStm rightInput2{ hal::Port::A, 11 };
        hal::GpioPinStm faultBreak{ hal::Port::A, 6 };
        hal::GpioPinStm faultInterrupt{ hal::Port::C, 4 };

        std::array<hal::PwmStmBase::ChannelConfig, 4> channels;
        hal::SynchronousPwmStm pwm;

        hal::GpioPinStm spiClock{ hal::Port::B, 3 };
        hal::GpioPinStm spiMiso{ hal::Port::B, 4 };
        hal::GpioPinStm spiMosi{ hal::Port::B, 5 };
        hal::GpioPinStm spiSelect{ hal::Port::A, 4 };
        services::GpioPinInverted spiSelectActiveHigh{ spiSelect };
        hal::SpiMasterStm spi{ 1, spiClock, spiMiso, spiMosi, SpiConfig() };
        services::SpiMasterWithChipSelect configurationChannel{ spi, spiSelectActiveHigh };
    };
}
