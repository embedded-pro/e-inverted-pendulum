#pragma once

#include "hal/interfaces/Flash.hpp"
#include "infra/event/EventDispatcher.hpp"
#include <algorithm>
#include <array>
#include <cstdint>

namespace application
{
    class FlashStub final
        : public hal::Flash
    {
    public:
        static constexpr uint32_t sectorSize{ 4096 };

        uint32_t NumberOfSectors() const override
        {
            return 1;
        }

        uint32_t SizeOfSector(uint32_t) const override
        {
            return sectorSize;
        }

        uint32_t SectorOfAddress(uint32_t address) const override
        {
            return address / sectorSize;
        }

        uint32_t AddressOfSector(uint32_t sectorIndex) const override
        {
            return sectorIndex * sectorSize;
        }

        void WriteBuffer(infra::ConstByteRange buffer, uint32_t address, infra::Function<void()> onDone) override
        {
            std::transform(buffer.begin(), buffer.end(), memory.begin() + address, memory.begin() + address, [](uint8_t written, uint8_t stored)
                {
                    return static_cast<uint8_t>(written & stored);
                });
            infra::EventDispatcher::Instance().Schedule(onDone);
        }

        void ReadBuffer(infra::ByteRange buffer, uint32_t address, infra::Function<void()> onDone) override
        {
            std::copy(memory.begin() + address, memory.begin() + address + buffer.size(), buffer.begin());
            infra::EventDispatcher::Instance().Schedule(onDone);
        }

        void EraseSectors(uint32_t beginIndex, uint32_t endIndex, infra::Function<void()> onDone) override
        {
            std::fill(memory.begin() + AddressOfSector(beginIndex), memory.begin() + AddressOfSector(endIndex), erased);
            infra::EventDispatcher::Instance().Schedule(onDone);
        }

    private:
        static constexpr uint8_t erased{ 0xff };

        std::array<uint8_t, sectorSize> memory{ MakeErased() };

        static constexpr std::array<uint8_t, sectorSize> MakeErased()
        {
            std::array<uint8_t, sectorSize> result{};
            result.fill(erased);
            return result;
        }
    };
}
