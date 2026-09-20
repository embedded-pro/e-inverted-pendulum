#pragma once

#include "core/platform_abstraction/InertialSensor.hpp"
#include "core/platform_abstraction/MotorDriver.hpp"
#include "core/platform_abstraction/WheelEncoders.hpp"
#include "hal/interfaces/Gpio.hpp"
#include "hal/interfaces/SerialCommunication.hpp"
#include "services/tracer/Tracer.hpp"

namespace platform
{
    class Platform
    {
    public:
        Platform() = default;
        Platform(const Platform& other) = delete;
        Platform& operator=(const Platform& other) = delete;

        virtual hal::GpioPin& StatusLed() = 0;
        virtual hal::SerialCommunication& Communication() = 0;
        virtual services::Tracer& Tracer() = 0;
        virtual MotorDriver& Motors() = 0;
        virtual WheelEncoders& Encoders() = 0;
        virtual InertialSensor& Inertial() = 0;
        virtual void Run() = 0;

    protected:
        ~Platform() = default;
    };
}
