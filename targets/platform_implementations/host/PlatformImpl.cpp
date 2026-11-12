#include "targets/platform_implementations/host/PlatformImpl.hpp"

namespace application
{
    hal::GpioPin& PlatformImpl::StatusLed()
    {
        return led;
    }

    hal::SerialCommunication& PlatformImpl::Communication()
    {
        return loopback.Client();
    }

    services::Tracer& PlatformImpl::Tracer()
    {
        return tracer;
    }

    platform::MotorDriver& PlatformImpl::Motors()
    {
        return motors;
    }

    platform::WheelEncoders& PlatformImpl::Encoders()
    {
        return encoders;
    }

    platform::InertialSensor& PlatformImpl::Inertial()
    {
        return inertial;
    }

    platform::ParameterStore PlatformImpl::ParameterStorage()
    {
        return { parameterStoreFirst, parameterStoreSecond };
    }

    void PlatformImpl::StartBluetooth(infra::BoundedConstString, const infra::Function<void(platform::Bluetooth& bluetooth)>&)
    {}

    void PlatformImpl::Run()
    {
        eventDispatcher.Run();
    }
}
