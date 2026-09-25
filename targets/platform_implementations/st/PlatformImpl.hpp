#pragma once

#include "core/platform_abstraction/Platform.hpp"
#include "hal_st/instantiations/StmEventInfrastructure.hpp"
#include INVERTED_PENDULUM_BOT_ST_CLOCK_HEADER
#include "hal_st/middlewares/ble_middleware/BondStorageSt.hpp"
#include "hal_st/middlewares/ble_middleware/TracingSystemTransportLayerWb.hpp"
#include "hal_st/stm32fxxx/GpioStm.hpp"
#include "hal_st/stm32fxxx/UartStm.hpp"
#include "infra/stream/OutputStream.hpp"
#include "infra/util/ProxyCreator.hpp"
#include "services/ble/BondStorageSynchronizer.hpp"
#include "services/ble/VolatileBondStorage.hpp"
#include "services/tracer/StreamWriterOnSerialCommunication.hpp"
#include "services/tracer/Tracer.hpp"
#include "services/util/ConfigurationStore.hpp"
#include "targets/platform_implementations/st/BluetoothPeripheralStm.hpp"
#include "targets/platform_implementations/st/InertialSensorStm.hpp"
#include "targets/platform_implementations/st/MotorDriverStm.hpp"
#include "targets/platform_implementations/st/WheelEncodersStm.hpp"
#include <array>
#include <optional>

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
        void StartBluetooth(infra::BoundedConstString deviceName, const infra::Function<void(platform::Bluetooth& bluetooth)>& onReady) override;
        void Run() override;

    private:
        static constexpr uint16_t maxAttMtuSize{ 251 };
        static constexpr uint8_t numberOfLinks{ 1 };
        static constexpr uint32_t maxNumberOfBonds{ 10 };

        void BluetoothStackRunning(services::BondStorageSynchronizer& synchronizer);
        void StartBluetoothWhenReady();

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

        services::StreamWriterOnSerialCommunication::WithStorage<1024> streamWriter{ console };
        infra::TextOutputStream::WithErrorPolicy stream{ streamWriter };
        services::TracerToStream tracer{ stream };

        services::ConfigurationStoreStub bondConfigurationStore;
        std::array<uint8_t, hal::SystemTransportLayerWb::bondBlobSize> bondBlob{};
        infra::ByteRange bondBlobRange{ infra::MakeRange(bondBlob) };
        services::VolatileBondStorage::WithMaxBonds<maxNumberOfBonds> volatileBondStorage;
        hal::BondStorageSt bondStorageSt{ maxNumberOfBonds };
        infra::Creator<services::BondStorageSynchronizer, services::BondStorageSynchronizerImpl, void()> bondStorageSynchronizerCreator{ [this](std::optional<services::BondStorageSynchronizerImpl>& synchronizer)
            {
                synchronizer.emplace(volatileBondStorage, bondStorageSt);
            } };

        hal::TracingSystemTransportLayerWb transport{
            services::ConfigurationStoreAccess<infra::ByteRange>{ bondConfigurationStore, bondBlobRange },
            bondStorageSynchronizerCreator,
            { maxAttMtuSize, hal::SystemTransportLayerWb::RfWakeupClock::lowSpeedExternal, numberOfLinks },
            [this](services::BondStorageSynchronizer& synchronizer)
            {
                BluetoothStackRunning(synchronizer);
            },
            tracer
        };

        services::BondStorageSynchronizer* bondStorageSynchronizer{ nullptr };
        infra::BoundedConstString bluetoothName;
        infra::Function<void(platform::Bluetooth& bluetooth)> onBluetoothReady;
        std::optional<BluetoothPeripheralStm> bluetooth;
    };
}
