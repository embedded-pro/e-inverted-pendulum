#include "core/balance_control/test_doubles/BalanceControlMock.hpp"
#include "core/persistence/implementations/PersistentBalanceControl.hpp"
#include "hal/interfaces/test_doubles/EepromStub.hpp"
#include "infra/event/EventDispatcher.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <array>
#include <chrono>
#include <optional>

namespace
{
    using namespace std::chrono_literals;

    constexpr std::array<balance::ParameterDescriptor, 2> pidDescriptors{ {
        { "a", 0.0f, 1.0f },
        { "b", 0.0f, 1.0f },
    } };

    constexpr std::array<balance::ParameterDescriptor, 1> lqrDescriptors{ {
        { "c", -2.0f, 2.0f },
    } };

    class DroppingEeprom
        : public hal::Eeprom
    {
    public:
        uint32_t Size() const override
        {
            return 512;
        }

        void WriteBuffer(infra::ConstByteRange, uint32_t, infra::Function<void()> onDone) override
        {
            infra::EventDispatcher::Instance().Schedule(onDone);
        }

        void ReadBuffer(infra::ByteRange buffer, uint32_t, infra::Function<void()> onDone) override
        {
            std::fill(buffer.begin(), buffer.end(), 0xff);
            infra::EventDispatcher::Instance().Schedule(onDone);
        }

        void Erase(infra::Function<void()> onDone) override
        {
            infra::EventDispatcher::Instance().Schedule(onDone);
        }
    };

    class PersistentBalanceControlTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        PersistentBalanceControlTest()
        {
            EXPECT_CALL(control, StrategyCount()).WillRepeatedly(testing::Return(2));
            EXPECT_CALL(control, StrategyName(0)).WillRepeatedly(testing::Return("pid"));
            EXPECT_CALL(control, StrategyName(1)).WillRepeatedly(testing::Return("lqr"));
            EXPECT_CALL(control, StrategyParameters(0)).WillRepeatedly(testing::Return(infra::MakeRange(pidDescriptors)));
            EXPECT_CALL(control, StrategyParameters(1)).WillRepeatedly(testing::Return(infra::MakeRange(lqrDescriptors)));
        }

        void Store(uint32_t address, const persistence::TuningRecord& record)
        {
            std::array<uint8_t, persistence::PersistentBalanceControl::regionSize> buffer{};
            buffer.fill(0xff);
            persistence::Encode(record, infra::MakeRange(buffer));
            eeprom.WriteBuffer(infra::MakeConstByteRange(buffer), address, [] {});
            ExecuteAllActions();
        }

        std::optional<persistence::TuningRecord> Stored(uint32_t address)
        {
            std::array<uint8_t, persistence::PersistentBalanceControl::regionSize> buffer{};
            eeprom.ReadBuffer(infra::MakeRange(buffer), address, [] {});
            ExecuteAllActions();
            return persistence::Decode(infra::MakeConstByteRange(buffer));
        }

        persistence::TuningRecord Record(uint16_t sequence, float a)
        {
            persistence::TuningRecord record;
            record.sequence = sequence;
            record.activeStrategy = persistence::KeyOf("pid", "");
            record.entries.push_back({ persistence::KeyOf("pid", "a"), a });
            return record;
        }

        void Start(hal::Eeprom& storage)
        {
            store.emplace(control, storage);
            ExecuteAllActions();
        }

        void ExpectCapture(float a, float b, float c)
        {
            EXPECT_CALL(control, Engaged()).WillOnce(testing::Return(false));
            EXPECT_CALL(control, ActiveStrategy()).WillOnce(testing::Return(1));
            EXPECT_CALL(control, StrategyParameter(0, 0)).WillOnce(testing::Return(a));
            EXPECT_CALL(control, StrategyParameter(0, 1)).WillOnce(testing::Return(b));
            EXPECT_CALL(control, StrategyParameter(1, 0)).WillOnce(testing::Return(c));
        }

        void AcceptParameter()
        {
            EXPECT_CALL(control, SetParameter(0, 0.5f)).WillOnce(testing::Return(true));
            EXPECT_TRUE(store->SetParameter(0, 0.5f));
        }

        testing::StrictMock<balance::BalanceControlMock> control;
        hal::EepromStub eeprom{ 512 };
        std::optional<persistence::PersistentBalanceControl> store;
    };
}

TEST_F(PersistentBalanceControlTest, blank_storage_keeps_the_defaults)
{
    Start(eeprom);

    EXPECT_EQ(persistence::RestoreResult::defaults, store->Restore());
    EXPECT_EQ(persistence::SaveResult::none, store->LastSave());
}

TEST_F(PersistentBalanceControlTest, a_stored_record_is_restored)
{
    auto record = Record(1, 0.25f);
    record.activeStrategy = persistence::KeyOf("lqr", "");
    record.entries.push_back({ persistence::KeyOf("lqr", "c"), -1.0f });
    Store(0, record);

    EXPECT_CALL(control, SetStrategyParameter(0, 0, 0.25f)).WillOnce(testing::Return(true));
    EXPECT_CALL(control, SetStrategyParameter(1, 0, -1.0f)).WillOnce(testing::Return(true));
    EXPECT_CALL(control, Select(1)).WillOnce(testing::Return(true));
    Start(eeprom);

    EXPECT_EQ(persistence::RestoreResult::restored, store->Restore());
}

TEST_F(PersistentBalanceControlTest, the_newer_copy_is_restored)
{
    Store(0, Record(1, 0.1f));
    Store(persistence::PersistentBalanceControl::regionSize, Record(2, 0.2f));

    EXPECT_CALL(control, SetStrategyParameter(0, 0, 0.2f)).WillOnce(testing::Return(true));
    EXPECT_CALL(control, Select(0)).WillOnce(testing::Return(true));
    Start(eeprom);
}

TEST_F(PersistentBalanceControlTest, a_torn_newer_copy_falls_back_to_the_older_copy)
{
    Store(0, Record(1, 0.1f));
    Store(persistence::PersistentBalanceControl::regionSize, Record(2, 0.2f));
    std::array<uint8_t, 1> torn{ 0x00 };
    eeprom.WriteBuffer(infra::MakeConstByteRange(torn), persistence::PersistentBalanceControl::regionSize + 9, [] {});

    EXPECT_CALL(control, SetStrategyParameter(0, 0, 0.1f)).WillOnce(testing::Return(true));
    EXPECT_CALL(control, Select(0)).WillOnce(testing::Return(true));
    Start(eeprom);

    EXPECT_EQ(persistence::RestoreResult::restored, store->Restore());
}

TEST_F(PersistentBalanceControlTest, unreadable_copies_are_reported_as_corrupt)
{
    std::array<uint8_t, 4> garbage{ 0x01, 0x02, 0x03, 0x04 };
    eeprom.WriteBuffer(infra::MakeConstByteRange(garbage), 0, [] {});

    Start(eeprom);

    EXPECT_EQ(persistence::RestoreResult::corrupt, store->Restore());
}

TEST_F(PersistentBalanceControlTest, unknown_keys_are_ignored)
{
    auto record = Record(1, 0.1f);
    record.entries.push_back({ persistence::KeyOf("gone", "x"), 9.0f });
    Store(0, record);

    EXPECT_CALL(control, SetStrategyParameter(0, 0, 0.1f)).WillOnce(testing::Return(true));
    EXPECT_CALL(control, Select(0)).WillOnce(testing::Return(true));
    Start(eeprom);
}

TEST_F(PersistentBalanceControlTest, an_accepted_change_is_saved_after_the_delay)
{
    Start(eeprom);
    AcceptParameter();
    EXPECT_EQ(persistence::SaveResult::pending, store->LastSave());

    ForwardTime(1999ms);
    EXPECT_FALSE(Stored(0).has_value());

    ExpectCapture(0.5f, 0.75f, -1.5f);
    ForwardTime(1ms);

    const auto saved = Stored(0);
    ASSERT_TRUE(saved.has_value());
    EXPECT_EQ(1, saved->sequence);
    EXPECT_EQ(persistence::KeyOf("lqr", ""), saved->activeStrategy);
    ASSERT_EQ(3u, saved->entries.size());
    EXPECT_FLOAT_EQ(-1.5f, saved->entries[2].value);
    EXPECT_EQ(persistence::SaveResult::saved, store->LastSave());
}

TEST_F(PersistentBalanceControlTest, a_burst_of_changes_makes_one_save)
{
    Start(eeprom);
    AcceptParameter();
    ForwardTime(1s);
    AcceptParameter();

    ForwardTime(1999ms);
    ExpectCapture(0.5f, 0.5f, 0.0f);
    ForwardTime(1ms);
    ForwardTime(5s);
}

TEST_F(PersistentBalanceControlTest, a_rejected_change_is_not_saved)
{
    Start(eeprom);
    EXPECT_CALL(control, Select(1)).WillOnce(testing::Return(false));

    EXPECT_FALSE(store->Select(1));
    ForwardTime(5s);
    EXPECT_EQ(persistence::SaveResult::none, store->LastSave());
}

TEST_F(PersistentBalanceControlTest, the_save_waits_while_the_drive_is_engaged)
{
    Start(eeprom);
    EXPECT_CALL(control, SetStrategyParameter(1, 0, 1.0f)).WillOnce(testing::Return(true));
    EXPECT_TRUE(store->SetStrategyParameter(1, 0, 1.0f));

    EXPECT_CALL(control, Engaged()).WillOnce(testing::Return(true));
    ForwardTime(2s);
    EXPECT_FALSE(Stored(0).has_value());

    ExpectCapture(0.0f, 0.0f, 1.0f);
    ForwardTime(2s);
    EXPECT_TRUE(Stored(0).has_value());
}

TEST_F(PersistentBalanceControlTest, saves_alternate_between_the_copies)
{
    Start(eeprom);
    AcceptParameter();
    ExpectCapture(0.5f, 0.0f, 0.0f);
    ForwardTime(2s);

    EXPECT_CALL(control, Select(0)).WillOnce(testing::Return(true));
    EXPECT_TRUE(store->Select(0));
    ExpectCapture(0.5f, 0.0f, 0.0f);
    ForwardTime(2s);

    ASSERT_TRUE(Stored(0).has_value());
    ASSERT_TRUE(Stored(persistence::PersistentBalanceControl::regionSize).has_value());
    EXPECT_EQ(2, Stored(persistence::PersistentBalanceControl::regionSize)->sequence);
}

TEST_F(PersistentBalanceControlTest, a_save_that_does_not_read_back_is_reported_as_failed)
{
    DroppingEeprom dropping;
    Start(dropping);
    AcceptParameter();

    ExpectCapture(0.5f, 0.0f, 0.0f);
    ForwardTime(2s);

    EXPECT_EQ(persistence::SaveResult::failed, store->LastSave());
}

TEST_F(PersistentBalanceControlTest, other_calls_are_forwarded)
{
    Start(eeprom);

    EXPECT_CALL(control, Move(balance::Setpoints{ 0.1f, 0.0f })).WillOnce(testing::Return(true));
    EXPECT_TRUE(store->Move(balance::Setpoints{ 0.1f, 0.0f }));

    EXPECT_CALL(control, CancelMotion());
    store->CancelMotion();

    EXPECT_CALL(control, AppliedEffort()).WillOnce(testing::Return(balance::Effort{ 0.2f, -0.2f }));
    EXPECT_EQ((balance::Effort{ 0.2f, -0.2f }), store->AppliedEffort());

    EXPECT_CALL(control, Parameter(1)).WillOnce(testing::Return(0.3f));
    EXPECT_FLOAT_EQ(0.3f, store->Parameter(1));
}
