#pragma once

#include "hal/interfaces/MacAddress.hpp"
#include "services/ble/Att.hpp"
#include <cstdint>

namespace ble
{
    enum class RadioState : uint8_t
    {
        starting,
        advertising,
        connected
    };

    struct LinkReport
    {
        RadioState radio{ RadioState::starting };
        hal::MacAddress address{};
        uint16_t mtu{ services::attDefaultMaxMtuSize };
        bool telemetrySubscribed{ false };
    };

    class LinkStatus
    {
    public:
        LinkStatus() = default;
        LinkStatus(const LinkStatus& other) = delete;
        LinkStatus& operator=(const LinkStatus& other) = delete;

        virtual LinkReport Report() const = 0;

    protected:
        ~LinkStatus() = default;
    };
}
