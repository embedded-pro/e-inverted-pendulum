#pragma once

#include "core/platform_abstraction/Bluetooth.hpp"
#include "hal_st/middlewares/ble_middleware/BondStorageSt.hpp"
#include "hal_st/middlewares/ble_middleware/TracingSystemTransportLayerWb.hpp"
#include "infra/util/BoundedString.hpp"
#include "infra/util/Function.hpp"
#include "infra/util/ProxyCreator.hpp"
#include "services/ble/BondStorageSynchronizer.hpp"
#include "services/util/ConfigurationStore.hpp"
#include "targets/platform_implementations/st/BluetoothPeripheralStm.hpp"
#include "targets/platform_implementations/st/VolatileBondStorage.hpp"
#include <array>
#include <optional>

namespace application
{
    class BluetoothStm
    {
    public:
        explicit BluetoothStm(services::Tracer& tracer);

        void Start(infra::BoundedConstString deviceName, const infra::Function<void(platform::Bluetooth& bluetooth)>& onReady);

    private:
        void StackRunning(services::BondStorageSynchronizer& bondStorageSynchronizer);

        static constexpr uint8_t numberOfLinks{ 1 };

        services::Tracer& tracer;
        infra::BoundedConstString deviceName;
        infra::Function<void(platform::Bluetooth& bluetooth)> onReady;

        services::ConfigurationStoreStub configurationStore;
        std::array<uint8_t, hal::SystemTransportLayerWb::bondBlobSize> bondBlob{};
        infra::ByteRange bondBlobRange{ infra::MakeRange(bondBlob) };

        VolatileBondStorage volatileBondStorage;
        hal::BondStorageSt bondStorageSt{ VolatileBondStorage::maxNumberOfBonds };
        infra::Creator<services::BondStorageSynchronizer, services::BondStorageSynchronizerImpl, void()> bondStorageSynchronizerCreator;

        std::optional<hal::TracingSystemTransportLayerWb> transport;
        std::optional<BluetoothPeripheralStm> peripheral;
    };
}
