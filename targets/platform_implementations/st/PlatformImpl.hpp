#pragma once

#include "core/platform_abstraction/Platform.hpp"
#include "hal_st/instantiations/StmEventInfrastructure.hpp"
#include INVERTED_PENDULUM_BOT_ST_CLOCK_HEADER
#include "hal_st/stm32fxxx/GpioStm.hpp"
#include "hal_st/stm32fxxx/UartStm.hpp"
#include "infra/stream/OutputStream.hpp"
#include "services/tracer/StreamWriterOnSerialCommunication.hpp"
#include "services/tracer/Tracer.hpp"
#include "targets/platform_implementations/st/InertialSensorStm.hpp"
#include "targets/platform_implementations/st/MotorDriverStm.hpp"
#include "targets/platform_implementations/st/WheelEncodersStm.hpp"

namespace application
{
    class PlatformImpl final
        : public platform::Platform
    {
    public:
        PlatformImpl() = default;

        hal::GpioPin& StatusLed() override;
        hal::SerialCommunication& Communication() override;
        services::Tracer& Tracer() override;
        platform::MotorDriver& Motors() override;
        platform::WheelEncoders& Encoders() override;
        platform::InertialSensor& Inertial() override;
        void StartBluetooth(const infra::Function<void(platform::Bluetooth& bluetooth)>& onReady) override;
        void Run() override;

    private:
        struct ClockInit
        {
            ClockInit()
            {
                HAL_Init();
                INVERTED_PENDULUM_BOT_ST_CLOCK_INIT();
            }
        };

        ClockInit clockInit;
        main_::StmEventInfrastructure eventInfrastructure;

        hal::GpioPinStm statusLed{ hal::Port::B, 0 };

        hal::GpioPinStm consoleTx{ hal::Port::B, 6 };
        hal::GpioPinStm consoleRx{ hal::Port::B, 7 };
        hal::UartStm console{ 1, consoleTx, consoleRx };

        MotorDriverStm motors;
        WheelEncodersStm encoders;
        InertialSensorStm inertial{ platform::AxisMap{} };

        services::StreamWriterOnSerialCommunication::WithStorage<256> streamWriter{ console };
        infra::TextOutputStream::WithErrorPolicy stream{ streamWriter };
        services::TracerToStream tracer{ stream };
    };
}
