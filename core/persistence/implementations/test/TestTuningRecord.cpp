#include "core/persistence/implementations/TuningRecord.hpp"
#include "gtest/gtest.h"
#include <array>
#include <cstdint>

namespace
{
    persistence::TuningRecord Example()
    {
        persistence::TuningRecord record;
        record.activeStrategy = persistence::KeyOf("lqr", "");
        record.entries.push_back({ persistence::KeyOf("pid", "pitch.kp"), 2.5f });
        record.entries.push_back({ persistence::KeyOf("lqr", "k.pitch"), -12.0f });
        return record;
    }

    class TuningRecordTest
        : public testing::Test
    {
    public:
        std::array<uint8_t, persistence::TuningRecord::maxSize> buffer{};
    };
}

TEST(TuningRecordKeyTest, keys_differ_by_strategy_and_by_parameter)
{
    EXPECT_NE(persistence::KeyOf("pid", "yaw.kp"), persistence::KeyOf("lqr", "yaw.kp"));
    EXPECT_NE(persistence::KeyOf("pid", "yaw.kp"), persistence::KeyOf("pid", "yaw.ki"));
    EXPECT_EQ(persistence::KeyOf("pid", "yaw.kp"), persistence::KeyOf("pid", "yaw.kp"));
}

TEST_F(TuningRecordTest, a_record_survives_an_encode_and_decode)
{
    const auto size = persistence::Encode(Example(), infra::MakeRange(buffer));
    const auto decoded = persistence::Decode(infra::Head(infra::MakeConstByteRange(buffer), size));

    ASSERT_TRUE(decoded.has_value());
    EXPECT_EQ(persistence::KeyOf("lqr", ""), decoded->activeStrategy);
    ASSERT_EQ(2u, decoded->entries.size());
    EXPECT_EQ(persistence::KeyOf("lqr", "k.pitch"), decoded->entries[1].key);
    EXPECT_FLOAT_EQ(-12.0f, decoded->entries[1].value);
}

TEST_F(TuningRecordTest, a_full_record_fits_the_maximum_size)
{
    persistence::TuningRecord record;
    for (uint16_t key = 0; key != persistence::TuningRecord::maxEntries; ++key)
        record.entries.push_back({ key, 1.0f });

    EXPECT_EQ(100u, persistence::Encode(record, infra::MakeRange(buffer)));
}

TEST_F(TuningRecordTest, erased_storage_is_rejected)
{
    buffer.fill(0xff);

    EXPECT_FALSE(persistence::Decode(infra::MakeConstByteRange(buffer)).has_value());
}

TEST_F(TuningRecordTest, an_unknown_format_version_is_rejected)
{
    const auto size = persistence::Encode(Example(), infra::MakeRange(buffer));
    buffer[0] = 0x7f;

    EXPECT_FALSE(persistence::Decode(infra::Head(infra::MakeConstByteRange(buffer), size)).has_value());
}

TEST_F(TuningRecordTest, an_entry_count_beyond_the_maximum_is_rejected)
{
    persistence::Encode(Example(), infra::MakeRange(buffer));
    buffer[3] = persistence::TuningRecord::maxEntries + 1;

    EXPECT_FALSE(persistence::Decode(infra::MakeConstByteRange(buffer)).has_value());
}

TEST_F(TuningRecordTest, a_size_that_does_not_match_the_entry_count_is_rejected)
{
    const auto size = persistence::Encode(Example(), infra::MakeRange(buffer));

    EXPECT_FALSE(persistence::Decode(infra::Head(infra::MakeConstByteRange(buffer), size - 3)).has_value());
    EXPECT_FALSE(persistence::Decode(infra::Head(infra::MakeConstByteRange(buffer), size + 1)).has_value());
}
