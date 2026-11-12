#pragma once

#include "core/platform_abstraction/Platform.hpp"
#include "hal_st/instantiations/StmEventInfrastructure.hpp"
#include INVERTED_PENDULUM_BOT_ST_CLOCK_HEADER
#include "generated/echo/BondRecord.pb.hpp"
#include "hal_st/middlewares/ble_middleware/BondStorageSt.hpp"
#include "hal_st/middlewares/ble_middleware/TracingSystemTransportLayerWb.hpp"
#include "hal_st/stm32fxxx/FlashCoordinatedWithWirelessStack.hpp"
#include "hal_st/stm32fxxx/FlashInternalStm.hpp"
#include "hal_st/stm32fxxx/GpioStm.hpp"
#include "hal_st/stm32fxxx/UartStm.hpp"
#include "hal_st/stm32fxxx/WatchDogStm.hpp"
#include "infra/stream/OutputStream.hpp"
#include "infra/util/ProxyCreator.hpp"
#include "services/ble/BondStorageSynchronizer.hpp"
#include "services/ble/PersistentBondStorage.hpp"
#include "services/crypto/Sha256Software.hpp"
#include "services/flash/FlashMultipleAccess.hpp"
#include "services/flash/FlashRegion.hpp"
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
        platform::ParameterStore ParameterStorage() override;
        void StartBluetooth(infra::BoundedConstString deviceName, const infra::Function<void(platform::Bluetooth& bluetooth)>& onReady) override;
        void Run() override;

    private:
        static constexpr uint16_t maxAttMtuSize{ 251 };
        static constexpr uint8_t numberOfLinks{ 1 };
        static constexpr uint32_t maxNumberOfBonds{ 10 };
        static constexpr uint32_t firstPersistenceSector{ 128 };

        void BondsRecovered();
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

        hal::WatchDogStm watchdog{ []()
            {
                HAL_NVIC_SystemReset();
            } };
        hal::FlashHomogeneousInternalStm internalFlash{ FLASH_PAGE_NB, FLASH_PAGE_SIZE, infra::ConstByteRange{ reinterpret_cast<const uint8_t*>(FLASH_BASE), reinterpret_cast<const uint8_t*>(FLASH_BASE + FLASH_SIZE) } };
        hal::FlashCoordinatedWithWirelessStack flash{ internalFlash, watchdog, hal::FlashCoordinatedWithWirelessStack::WirelessStack::stopped };
        services::FlashMultipleAccessMaster flashMaster{ flash };
        services::FlashMultipleAccess tuningFlash{ flashMaster };
        services::FlashMultipleAccess bondFlash{ flashMaster };
        services::FlashRegion tuningFirst{ tuningFlash, firstPersistenceSector, 1 };
        services::FlashRegion tuningSecond{ tuningFlash, firstPersistenceSector + 1, 1 };
        services::FlashRegion bondFirst{ bondFlash, firstPersistenceSector + 2, 1 };
        services::FlashRegion bondSecond{ bondFlash, firstPersistenceSector + 3, 1 };
        services::Sha256Software sha256;

        services::ConfigurationStoreImpl<bonds::BondRecord>::WithBlobs<> bondStore{ bondFirst, bondSecond, sha256, [this](bool)
            {
                BondsRecovered();
            } };
        infra::ByteRange stackBonds;
        std::optional<services::PersistentBondStorage> bondStorage;
        hal::BondStorageSt bondStorageSt{ maxNumberOfBonds };
        infra::Creator<services::BondStorageSynchronizer, services::BondStorageSynchronizerImpl, void()> bondStorageSynchronizerCreator{ [this](std::optional<services::BondStorageSynchronizerImpl>& synchronizer)
            {
                synchronizer.emplace(*bondStorage, bondStorageSt);
            } };
        std::optional<hal::TracingSystemTransportLayerWb> transport;

        services::BondStorageSynchronizer* bondStorageSynchronizer{ nullptr };
        infra::BoundedConstString bluetoothName;
        infra::Function<void(platform::Bluetooth& bluetooth)> onBluetoothReady;
        std::optional<BluetoothPeripheralStm> bluetooth;
    };
}
