#pragma once

#include "infra/util/BoundedString.hpp"
#include "infra/util/BoundedVector.hpp"
#include "infra/util/ByteRange.hpp"
#include <cstddef>
#include <cstdint>
#include <optional>

namespace persistence
{
    struct TuningEntry
    {
        uint16_t key{ 0 };
        float value{ 0.0f };
    };

    struct TuningRecord
    {
        static constexpr std::size_t maxEntries{ 16 };
        static constexpr std::size_t headerSize{ 4 };
        static constexpr std::size_t entrySize{ 6 };
        static constexpr std::size_t maxSize{ headerSize + maxEntries * entrySize };

        uint16_t activeStrategy{ 0 };
        infra::BoundedVector<TuningEntry>::WithMaxSize<maxEntries> entries;
    };

    uint16_t KeyOf(infra::BoundedConstString strategy, infra::BoundedConstString parameter);
    std::size_t Encode(const TuningRecord& record, infra::ByteRange buffer);
    std::optional<TuningRecord> Decode(infra::ConstByteRange data);
}
