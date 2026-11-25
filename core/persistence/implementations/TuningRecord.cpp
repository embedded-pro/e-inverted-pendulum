#include "core/persistence/implementations/TuningRecord.hpp"
#include "infra/util/Crc.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <bit>

namespace persistence
{
    namespace
    {
        constexpr uint16_t magic{ 0x5455 };
        constexpr uint8_t formatVersion{ 1 };
        constexpr char keySeparator{ '/' };

        void Put16(infra::ByteRange buffer, std::size_t offset, uint16_t value)
        {
            buffer[offset] = static_cast<uint8_t>(value);
            buffer[offset + 1] = static_cast<uint8_t>(value >> 8);
        }

        void Put32(infra::ByteRange buffer, std::size_t offset, uint32_t value)
        {
            Put16(buffer, offset, static_cast<uint16_t>(value));
            Put16(buffer, offset + 2, static_cast<uint16_t>(value >> 16));
        }

        uint16_t Get16(infra::ConstByteRange data, std::size_t offset)
        {
            return static_cast<uint16_t>(data[offset] | (data[offset + 1] << 8));
        }

        uint32_t Get32(infra::ConstByteRange data, std::size_t offset)
        {
            return Get16(data, offset) | (static_cast<uint32_t>(Get16(data, offset + 2)) << 16);
        }

        uint16_t CrcOf(infra::ConstByteRange data)
        {
            infra::Crc16CcittFalseBitwise crc;
            crc.Update(data);
            return crc.Result();
        }

        std::size_t SizeFor(std::size_t entries)
        {
            return TuningRecord::headerSize + entries * TuningRecord::entrySize + TuningRecord::crcSize;
        }

        bool Sealed(infra::ConstByteRange data, std::size_t size)
        {
            return CrcOf(infra::Head(data, size - TuningRecord::crcSize)) == Get16(data, size - TuningRecord::crcSize);
        }
    }

    uint16_t KeyOf(infra::BoundedConstString strategy, infra::BoundedConstString parameter)
    {
        infra::Crc16CcittFalseBitwise crc;
        crc.Update(infra::StringAsByteRange(strategy));
        crc.Update(static_cast<uint8_t>(keySeparator));
        crc.Update(infra::StringAsByteRange(parameter));
        return crc.Result();
    }

    std::size_t Encode(const TuningRecord& record, infra::ByteRange buffer)
    {
        const auto size = SizeFor(record.entries.size());
        really_assert(buffer.size() >= size);

        Put16(buffer, 0, magic);
        buffer[2] = formatVersion;
        Put16(buffer, 3, record.sequence);
        Put16(buffer, 5, record.activeStrategy);
        buffer[7] = static_cast<uint8_t>(record.entries.size());

        for (std::size_t i = 0; i != record.entries.size(); ++i)
        {
            const auto offset = TuningRecord::headerSize + i * TuningRecord::entrySize;
            Put16(buffer, offset, record.entries[i].key);
            Put32(buffer, offset + 2, std::bit_cast<uint32_t>(record.entries[i].value));
        }

        Put16(buffer, size - TuningRecord::crcSize, CrcOf(infra::Head(infra::ConstByteRange(buffer), size - TuningRecord::crcSize)));
        return size;
    }

    std::optional<TuningRecord> Decode(infra::ConstByteRange data)
    {
        if (data.size() < SizeFor(0) || Get16(data, 0) != magic || data[2] != formatVersion || data[7] > TuningRecord::maxEntries)
            return std::nullopt;

        const auto size = SizeFor(data[7]);
        if (data.size() < size || !Sealed(data, size))
            return std::nullopt;

        TuningRecord record;
        record.sequence = Get16(data, 3);
        record.activeStrategy = Get16(data, 5);

        for (std::size_t i = 0; i != data[7]; ++i)
        {
            const auto offset = TuningRecord::headerSize + i * TuningRecord::entrySize;
            record.entries.push_back({ Get16(data, offset), std::bit_cast<float>(Get32(data, offset + 2)) });
        }

        return record;
    }
}
