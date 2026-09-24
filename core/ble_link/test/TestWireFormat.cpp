#include "core/ble_link/WireFormat.hpp"
#include "gtest/gtest.h"
#include <array>
#include <cstdint>

TEST(WireFormatTest, floats_are_written_little_endian)
{
    std::array<uint8_t, 6> buffer{};

    ble::PutFloat(infra::MakeRange(buffer), 1, 1.0f);

    EXPECT_EQ((std::array<uint8_t, 6>{ 0x00, 0x00, 0x00, 0x80, 0x3f, 0x00 }), buffer);
}

TEST(WireFormatTest, floats_are_read_little_endian)
{
    const std::array<uint8_t, 5> buffer{ 0xaa, 0x00, 0x00, 0x20, 0xc1 };

    EXPECT_FLOAT_EQ(-10.0f, ble::GetFloat(infra::MakeRange(buffer), 1));
}

TEST(WireFormatTest, a_float_survives_a_round_trip)
{
    std::array<uint8_t, 4> buffer{};

    ble::PutFloat(infra::MakeRange(buffer), 0, -0.123456f);

    EXPECT_EQ(-0.123456f, ble::GetFloat(infra::MakeRange(buffer), 0));
}
