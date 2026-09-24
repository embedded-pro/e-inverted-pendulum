#pragma once

#include "core/platform_abstraction/InertialFrame.hpp"
#include "drivers/imu/mpu9250/Mpu9250BusAccessSpi.hpp"
#include "drivers/imu/mpu9250/Mpu9250Core.hpp"
#include "hal_st/stm32fxxx/GpioStm.hpp"
#include "hal_st/stm32fxxx/SpiMasterStm.hpp"
#include "services/peripheral/SpiMasterWithChipSelect.hpp"

namespace application
{
    class InertialSensorStm final
        : public platform::InertialSensor
    {
    public:
        explicit InertialSensorStm(const platform::AxisMap& axisMap);

        void Start(const infra::Function<void(const platform::InertialSample&)>& onSample) override;
        void Stop() override;

        bool Identified() const;

    private:
        static drivers::Mpu9250Core::Config DeviceConfig();
        static hal::SpiMasterStm::Config BusConfig();

        void StartSampling();
        void OnAcceleration(drivers::Mpu9250Core::Accelerometer::Samples samples);
        void OnAngularVelocity(drivers::Mpu9250Core::Gyroscope::Samples samples);

        hal::GpioPinStm clock{ hal::Port::B, 13 };
        hal::GpioPinStm miso{ hal::Port::B, 14 };
        hal::GpioPinStm mosi{ hal::Port::B, 15 };
        hal::GpioPinStm chipSelect{ hal::Port::B, 12 };
        hal::GpioPinStm dataReady{ hal::Port::C, 6 };

        hal::SpiMasterStm spi{ 2, clock, miso, mosi, BusConfig() };
        services::SpiMasterWithChipSelect spiWithChipSelect{ spi, chipSelect };
        drivers::Mpu9250BusAccessSpi busAccess{ spiWithChipSelect };
        drivers::Mpu9250Core device{ busAccess, dataReady };

        platform::AxisMap axisMap;
        bool sampling{ false };
        infra::Function<void(const platform::InertialSample&)> onSample;
        platform::InertialSample pending;
        bool accelerationReceived{ false };
        bool identified{ false };
        bool initializing{ false };
        bool stopping{ false };
    };
}
