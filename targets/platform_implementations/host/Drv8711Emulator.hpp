#pragma once

#include "hal/interfaces/Spi.hpp"
#include "infra/event/EventDispatcher.hpp"
#include <array>
#include <cstdint>

namespace application
{
    class Drv8711Emulator final
        : public hal::SpiMaster
    {
    public:
        void SendAndReceive(infra::ConstByteRange sendData, infra::ByteRange receiveData, hal::SpiAction, const infra::Function<void()>& onDone) override
        {
            if (sendData.size() == frameSize)
                Transfer(static_cast<uint16_t>((sendData[0] << 8) | sendData[1]), receiveData);

            infra::EventDispatcher::Instance().Schedule(onDone);
        }

        void SetChipSelectConfigurator(hal::ChipSelectConfigurator&) override
        {}

        void SetCommunicationConfigurator(hal::CommunicationConfigurator&) override
        {}

        void ResetCommunicationConfigurator() override
        {}

    private:
        static constexpr std::size_t frameSize{ 2 };
        static constexpr uint16_t readRequest{ 0x8000 };
        static constexpr uint16_t dataMask{ 0x0fff };

        void Transfer(uint16_t frame, infra::ByteRange receiveData)
        {
            const auto address = (frame >> 12) & 0x7;

            if ((frame & readRequest) == 0)
            {
                registers[address] = frame & dataMask;
                return;
            }

            if (receiveData.size() == frameSize)
            {
                receiveData[0] = static_cast<uint8_t>(registers[address] >> 8);
                receiveData[1] = static_cast<uint8_t>(registers[address] & 0xff);
            }
        }

        std::array<uint16_t, 8> registers{};
    };
}
