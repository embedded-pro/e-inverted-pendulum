#include "targets/platform_implementations/st/BluetoothStm.hpp"
#include "infra/util/ReallyAssert.hpp"

namespace application
{
    BluetoothStm::BluetoothStm(services::Tracer& tracer)
        : tracer{ tracer }
        , bondStorageSynchronizerCreator{ [this](std::optional<services::BondStorageSynchronizerImpl>& synchronizer)
            {
                synchronizer.emplace(volatileBondStorage, bondStorageSt);
            } }
    {}

    void BluetoothStm::Start(infra::BoundedConstString name, const infra::Function<void(platform::Bluetooth& bluetooth)>& ready)
    {
        really_assert(!transport);

        deviceName = name;
        onReady = ready;

        transport.emplace(
            services::ConfigurationStoreAccess<infra::ByteRange>{ configurationStore, bondBlobRange },
            bondStorageSynchronizerCreator,
            hal::SystemTransportLayerWb::Configuration{ LinkGattServerStm::largestAttMtu, hal::SystemTransportLayerWb::RfWakeupClock::lowSpeedExternal, numberOfLinks },
            [this](services::BondStorageSynchronizer& bondStorageSynchronizer)
            {
                StackRunning(bondStorageSynchronizer);
            },
            tracer);
    }

    void BluetoothStm::StackRunning(services::BondStorageSynchronizer& bondStorageSynchronizer)
    {
        peripheral.emplace(*transport, bondStorageSynchronizer, deviceName, tracer);
        onReady(*peripheral);
    }
}
