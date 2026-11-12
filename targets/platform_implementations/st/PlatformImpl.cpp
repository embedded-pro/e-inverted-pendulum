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

    platform::ParameterStore PlatformImpl::ParameterStorage()
    {
        return { tuningFirst, tuningSecond };
    }

    void PlatformImpl::StartBluetooth(infra::BoundedConstString deviceName, const infra::Function<void(platform::Bluetooth& bluetooth)>& onReady)
    {
        bluetoothName = deviceName;
        onBluetoothReady = onReady;
        StartBluetoothWhenReady();
    }

    void PlatformImpl::BondsRecovered()
    {
        auto& record = bondStore.Configuration();
        record.stackBonds.resize(hal::SystemTransportLayerWb::bondBlobSize, 0);
        stackBonds = infra::MakeRange(record.stackBonds);
        bondStorage.emplace(services::ConfigurationStoreAccess<infra::BoundedVector<uint8_t>>{ bondStore, record.addresses });

        flash.WirelessStackStarting();
        transport.emplace(
            services::ConfigurationStoreAccess<infra::ByteRange>{ bondStore, stackBonds },
            bondStorageSynchronizerCreator,
            hal::SystemTransportLayerWb::Configuration{ maxAttMtuSize, hal::SystemTransportLayerWb::RfWakeupClock::lowSpeedExternal, numberOfLinks },
            [this](services::BondStorageSynchronizer& synchronizer)
            {
                BluetoothStackRunning(synchronizer);
            },
            tracer);
    }

    void PlatformImpl::BluetoothStackRunning(services::BondStorageSynchronizer& synchronizer)
    {
        flash.WirelessStackReady();
        bondStorageSynchronizer = &synchronizer;
        StartBluetoothWhenReady();
    }

    void PlatformImpl::StartBluetoothWhenReady()
    {
        if (bondStorageSynchronizer == nullptr || !onBluetoothReady || bluetooth)
            return;

        bluetooth.emplace(*transport, *bondStorageSynchronizer, bluetoothName, tracer);
        onBluetoothReady(*bluetooth);
    }

    void PlatformImpl::Run()
    {
        eventInfrastructure.Run();
    }
}
