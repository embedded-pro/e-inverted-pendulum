#pragma once

#include "core/platform_abstraction/Bluetooth.hpp"
#include "hal_st/middlewares/ble_middleware/TracingGattServerSt.hpp"
#include <cstdint>

namespace application
{
    class LinkGattServerStm
        : public hal::TracingGattServerSt
    {
    public:
        static constexpr uint16_t largestAttMtu{ 251 };

        LinkGattServerStm(hal::HciEventSource& hciEventSource, services::Tracer& tracer);

        void SetLinkObserver(platform::BluetoothLinkObserver& observer);

        void HciEvent(hci_event_pckt& event) override;

    protected:
        void HandleGattAttributeModified(aci_gatt_attribute_modified_event_rp0& event) override;

    private:
        void HandleLeMetaEvent(const evt_le_meta_event& metaEvent);
        void HandleVendorEvent(const evt_blecore_aci& vendorEvent);
        void RequestLargestAttMtu(uint16_t connectionHandle) const;

        platform::BluetoothLinkObserver* observer{ nullptr };
    };
}
