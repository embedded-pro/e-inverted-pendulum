#pragma once

#include "core/platform_abstraction/Platform.hpp"
#include "hal/generic/TimerServiceGeneric.hpp"
#include "infra/event/EventDispatcherWithWeakPtr.hpp"
#include "infra/stream/StringOutputStream.hpp"
#include "services/peripheral/SerialCommunicationLoopback.hpp"
#include "services/tracer/Tracer.hpp"
#include "targets/platform_implementations/host/GpioStub.hpp"
#include "targets/platform_implementations/host/InertialSensorStub.hpp"
#include "targets/platform_implementations/host/MotorDriverStub.hpp"
#include "targets/platform_implementations/host/WheelEncodersStub.hpp"

namespace application
{
    class PlatformImpl final
        : public platform::Platform
    {
    public:
        hal::GpioPin& StatusLed() override;
        hal::SerialCommunication& Communication() override;
        services::Tracer& Tracer() override;
        platform::MotorDriver& Motors() override;
        platform::WheelEncoders& Encoders() override;
        platform::InertialSensor& Inertial() override;
        void Run() override;

    private:
        infra::EventDispatcherWithWeakPtr::WithSize<50> eventDispatcher;
        hal::TimerServiceGeneric timerService;
        GpioStub led;
        services::SerialCommunicationLoopback loopback;
        infra::StringOutputStream::WithStorage<1024> stream;
        services::TracerToStream tracer{ stream };
        MotorDriverStub motors;
        WheelEncodersStub encoders;
        InertialSensorStub inertial;
    };
}
