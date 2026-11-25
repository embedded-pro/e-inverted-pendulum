#pragma once

#include "hal/interfaces/Eeprom.hpp"
#include "infra/event/EventDispatcher.hpp"
#include <algorithm>
#include <array>
#include <cstdint>

namespace application
{
    class EepromStub final
        : public hal::Eeprom
    {
    public:
        EepromStub()
        {
            storage.fill(erased);
        }

        uint32_t Size() const override
        {
            return static_cast<uint32_t>(storage.size());
        }

        void WriteBuffer(infra::ConstByteRange buffer, uint32_t address, infra::Function<void()> onDone) override
        {
            std::copy(buffer.begin(), buffer.end(), storage.begin() + address);
            infra::EventDispatcher::Instance().Schedule(onDone);
        }

        void ReadBuffer(infra::ByteRange buffer, uint32_t address, infra::Function<void()> onDone) override
        {
            std::copy(storage.begin() + address, storage.begin() + address + buffer.size(), buffer.begin());
            infra::EventDispatcher::Instance().Schedule(onDone);
        }

        void Erase(infra::Function<void()> onDone) override
        {
            storage.fill(erased);
            infra::EventDispatcher::Instance().Schedule(onDone);
        }

    private:
        static constexpr uint8_t erased{ 0xff };

        std::array<uint8_t, 512> storage{};
    };
}
