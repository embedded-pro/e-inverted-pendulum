#include "targets/platform_implementations/st/LinkGattServerStm.hpp"
#include <algorithm>

namespace application
{
    LinkGattServerStm::LinkGattServerStm(hal::HciEventSource& hciEventSource, services::Tracer& tracer)
        : hal::TracingGattServerSt{ hciEventSource, tracer }
    {}

    void LinkGattServerStm::SetLinkObserver(platform::BluetoothLinkObserver& linkObserver)
    {
        observer = &linkObserver;
    }

    void LinkGattServerStm::HciEvent(hci_event_pckt& event)
    {
        hal::TracingGattServerSt::HciEvent(event);

        if (event.evt == HCI_LE_META_EVT_CODE)
            HandleLeMetaEvent(*reinterpret_cast<const evt_le_meta_event*>(event.data));
        else if (event.evt == HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE)
            HandleVendorEvent(*reinterpret_cast<const evt_blecore_aci*>(event.data));
    }

    void LinkGattServerStm::HandleGattAttributeModified(aci_gatt_attribute_modified_event_rp0& event)
    {
        hal::TracingGattServerSt::HandleGattAttributeModified(event);

        if (observer == nullptr || event.Attr_Data_Length != sizeof(uint16_t))
            return;

        const auto value = static_cast<uint16_t>(event.Attr_Data[0] | (event.Attr_Data[1] << 8));
        observer->ClientConfigurationWritten(event.Attr_Handle, value);
    }

    void LinkGattServerStm::HandleLeMetaEvent(const evt_le_meta_event& metaEvent)
    {
        if (metaEvent.subevent == HCI_LE_ENHANCED_CONNECTION_COMPLETE_SUBEVT_CODE)
        {
            const auto& connection = *reinterpret_cast<const hci_le_enhanced_connection_complete_event_rp0*>(metaEvent.data);

            if (connection.Status == BLE_STATUS_SUCCESS)
                RequestLargestAttMtu(connection.Connection_Handle);
        }
        else if (metaEvent.subevent == HCI_LE_CONNECTION_COMPLETE_SUBEVT_CODE)
        {
            const auto& connection = *reinterpret_cast<const hci_le_connection_complete_event_rp0*>(metaEvent.data);

            if (connection.Status == BLE_STATUS_SUCCESS)
                RequestLargestAttMtu(connection.Connection_Handle);
        }
    }

    void LinkGattServerStm::HandleVendorEvent(const evt_blecore_aci& vendorEvent)
    {
        if (vendorEvent.ecode != ACI_ATT_EXCHANGE_MTU_RESP_VSEVT_CODE || observer == nullptr)
            return;

        const auto& exchange = *reinterpret_cast<const aci_att_exchange_mtu_resp_event_rp0*>(vendorEvent.data);
        observer->AttMtuChanged(std::min(exchange.Server_RX_MTU, largestAttMtu));
    }

    void LinkGattServerStm::RequestLargestAttMtu(uint16_t connectionHandle) const
    {
        const auto status = aci_gatt_exchange_config(connectionHandle);

        if (status != BLE_STATUS_SUCCESS)
            ReportError(status);
    }
}
