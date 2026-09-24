#include "targets/platform_implementations/st/BluetoothPeripheralStm.hpp"
#include "stm32wbxx_ll_system.h"
#include "stm32wbxx_ll_utils.h"

namespace application
{
    namespace
    {
        ble::FactoryIdentity ReadFactoryIdentity()
        {
            return {
                LL_FLASH_GetUDN(),
                static_cast<uint8_t>(LL_FLASH_GetDeviceID()),
                LL_FLASH_GetSTCompanyID(),
                { LL_GetUID_Word0(), LL_GetUID_Word1(), LL_GetUID_Word2() }
            };
        }
    }

    BluetoothPeripheralStm::BluetoothPeripheralStm(hal::HciEventSource& hciEventSource, services::BondStorageSynchronizer& bondStorageSynchronizer, infra::BoundedConstString deviceName, services::Tracer& tracer)
        : identity{ ble::DeriveDeviceIdentity(ReadFactoryIdentity()) }
        , rootKeys{ identity.identityRoot, identity.encryptionRoot }
        , gapService{ deviceName, unknownAppearance }
        , gapConfiguration{ identity.address, gapService, rootKeys, hal::GapSt::encrypted, zeroDbmPowerLevel, false }
        , gap{ hciEventSource, bondStorageSynchronizer, gapConfiguration, tracer }
        , gattServer{ hciEventSource, tracer }
        , confirmIndication{ hciEventSource }
    {}

    services::GapPeripheral& BluetoothPeripheralStm::Gap()
    {
        return gap;
    }

    services::GattServer& BluetoothPeripheralStm::GattServer()
    {
        return gattServer;
    }

    void BluetoothPeripheralStm::SetLinkObserver(platform::BluetoothLinkObserver& observer)
    {
        gattServer.SetLinkObserver(observer);
    }
}
