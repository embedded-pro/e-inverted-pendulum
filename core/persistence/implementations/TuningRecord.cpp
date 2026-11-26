#include "core/persistence/implementations/TuningRecord.hpp"
#include "infra/util/Crc.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <bit>

namespace persistence
{
    namespace
    {
        constexpr uint8_t formatVersion{ 2 };
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

        std::size_t SizeFor(std::size_t entries)
        {
            return TuningRecord::headerSize + entries * TuningRecord::entrySize;
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

        buffer[0] = formatVersion;
        Put16(buffer, 1, record.activeStrategy);
        buffer[3] = static_cast<uint8_t>(record.entries.size());

        for (std::size_t i = 0; i != record.entries.size(); ++i)
        {
            const auto offset = TuningRecord::headerSize + i * TuningRecord::entrySize;
            Put16(buffer, offset, record.entries[i].key);
            Put32(buffer, offset + 2, std::bit_cast<uint32_t>(record.entries[i].value));
        }

        return size;
    }

    std::optional<TuningRecord> Decode(infra::ConstByteRange data)
    {
        if (data.size() < SizeFor(0) || data[0] != formatVersion || data[3] > TuningRecord::maxEntries || data.size() != SizeFor(data[3]))
            return std::nullopt;

        TuningRecord record;
        record.activeStrategy = Get16(data, 1);

        for (std::size_t i = 0; i != data[3]; ++i)
        {
            const auto offset = TuningRecord::headerSize + i * TuningRecord::entrySize;
            record.entries.push_back({ Get16(data, offset), std::bit_cast<float>(Get32(data, offset + 2)) });
        }

        return record;
    }
}
