#include "core/ble_link/WireFormat.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <bit>
#include <cstdint>

namespace ble
{
    void PutFloat(infra::ByteRange destination, std::size_t offset, float value)
    {
        really_assert(offset + sizeof(uint32_t) <= destination.size());

        const auto bits = std::bit_cast<uint32_t>(value);

        for (std::size_t byte = 0; byte != sizeof(uint32_t); ++byte)
            destination[offset + byte] = static_cast<uint8_t>(bits >> (8 * byte));
    }

    float GetFloat(infra::ConstByteRange source, std::size_t offset)
    {
        really_assert(offset + sizeof(uint32_t) <= source.size());

        uint32_t bits{ 0 };

        for (std::size_t byte = 0; byte != sizeof(uint32_t); ++byte)
            bits |= static_cast<uint32_t>(source[offset + byte]) << (8 * byte);

        return std::bit_cast<float>(bits);
    }
}
