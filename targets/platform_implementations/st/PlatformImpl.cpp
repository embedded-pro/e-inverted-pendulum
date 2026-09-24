#include "targets/platform_implementations/st/PlatformImpl.hpp"

unsigned int hse_value = 32'000'000;

namespace application
{
    hal::GpioPin& PlatformImpl::StatusLed()
    {
        return statusLed;
    }

    hal::SerialCommunication& PlatformImpl::Communication()
    {
        return console;
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

    void PlatformImpl::StartBluetooth(const infra::Function<void(platform::Bluetooth& bluetooth)>&)
    {}

    void PlatformImpl::Run()
    {
        eventInfrastructure.Run();
    }
}
