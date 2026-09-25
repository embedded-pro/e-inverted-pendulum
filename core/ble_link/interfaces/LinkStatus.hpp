#pragma once

#include "hal/interfaces/MacAddress.hpp"
#include "services/ble/Att.hpp"
#include "services/ble/GapPeripheral.hpp"
#include <cstdint>
#include <optional>

namespace ble
{
    struct LinkReport
    {
        services::GapPeripheralState state{ services::GapPeripheralState::standby };
        hal::MacAddress address{};
        uint16_t mtu{ services::attDefaultMaxMtuSize };
    };

    class LinkStatus
    {
    public:
        LinkStatus() = default;
        LinkStatus(const LinkStatus& other) = delete;
        LinkStatus& operator=(const LinkStatus& other) = delete;

        virtual std::optional<LinkReport> Report() const = 0;

    protected:
        ~LinkStatus() = default;
    };
}
