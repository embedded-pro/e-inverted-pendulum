#pragma once

#include "core/platform_abstraction/WheelEncoders.hpp"
#include "hal_st/stm32fxxx/GpioStm.hpp"
#include "hal_st/synchronous_stm32fxxx/SynchronousQuadratureEncoderLpTimStm.hpp"
#include "hal_st/synchronous_stm32fxxx/SynchronousQuadratureEncoderStm.hpp"

namespace application
{
    class WheelEncodersStm final
        : public platform::WheelEncoders
    {
    public:
        WheelEncodersStm();

        hal::SynchronousQuadratureEncoder& Left() override;
        hal::SynchronousQuadratureEncoder& Right() override;

    private:
        static hal::SynchronousQuadratureEncoderLpTimStm::Config LeftEncoderConfig();
        static hal::SynchronousQuadratureEncoderStm::Config RightEncoderConfig();

        hal::GpioPinStm leftPhaseA{ hal::Port::C, 0 };
        hal::GpioPinStm leftPhaseB{ hal::Port::C, 2 };
        hal::GpioPinStm leftIndex{ hal::Port::C, 5 };

        hal::GpioPinStm rightPhaseA{ hal::Port::A, 0 };
        hal::GpioPinStm rightPhaseB{ hal::Port::A, 1 };
        hal::GpioPinStm rightIndex{ hal::Port::C, 3 };

        hal::SynchronousQuadratureEncoderLpTimStm left;
        hal::SynchronousQuadratureEncoderStm right;
    };
}
