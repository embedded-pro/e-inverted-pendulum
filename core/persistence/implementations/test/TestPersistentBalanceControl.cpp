#include "core/balance_control/test_doubles/BalanceControlMock.hpp"
#include "core/persistence/implementations/PersistentBalanceControl.hpp"
#include "hal/interfaces/test_doubles/EepromStub.hpp"
#include "infra/event/ClaimableResource.hpp"
#include "infra/event/EventDispatcher.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "infra/util/Crc.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <array>
#include <chrono>
#include <optional>
#include <vector>

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

    class ControllableEeprom
        : public hal::Eeprom
    {
    public:
        uint32_t Size() const override
        {
            return static_cast<uint32_t>(storage.size());
        }

        void WriteBuffer(infra::ConstByteRange buffer, uint32_t address, infra::Function<void()> onDone) override
        {
            std::copy(buffer.begin(), buffer.end(), storage.begin() + address);
            Complete(false, onDone);
        }

        void ReadBuffer(infra::ByteRange buffer, uint32_t address, infra::Function<void()> onDone) override
        {
            std::copy(storage.begin() + address, storage.begin() + address + buffer.size(), buffer.begin());
            Complete(holdReads, onDone);
        }

        void Erase(infra::Function<void()> onDone) override
        {
            storage.fill(0xff);
            Complete(false, onDone);
        }

        void Release()
        {
            auto onDone = held;
            held = nullptr;
            infra::EventDispatcher::Instance().Schedule(onDone);
        }

        bool holdReads{ false };

    private:
        void Complete(bool hold, const infra::Function<void()>& onDone)
        {
            if (hold)
                held = onDone;
            else
                infra::EventDispatcher::Instance().Schedule(onDone);
        }

        std::array<uint8_t, 512> storage = []
        {
            std::array<uint8_t, 512> erased{};
            erased.fill(0xff);
            return erased;
        }();
        infra::Function<void()> held;
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

        void Store(uint32_t address, const persistence::TuningRecord& record, uint16_t sequence = 1)
        {
            std::array<uint8_t, persistence::TuningRecord::maxSize> payload{};
            const auto size = persistence::Encode(record, infra::MakeRange(payload));
            StorePayload(address, std::vector<uint8_t>(payload.begin(), payload.begin() + static_cast<std::ptrdiff_t>(size)), sequence);
        }

        void StorePayload(uint32_t address, std::vector<uint8_t> payload, uint16_t sequence = 1)
        {
            std::vector<uint8_t> copy{ 0x52, 0x53, static_cast<uint8_t>(sequence), static_cast<uint8_t>(sequence >> 8), static_cast<uint8_t>(payload.size()), 0 };
            copy.insert(copy.end(), payload.begin(), payload.end());
            infra::Crc16CcittFalse crc;
            crc.Update(infra::MakeRange(copy));
            copy.insert(copy.begin() + 6, { static_cast<uint8_t>(crc.Result()), static_cast<uint8_t>(crc.Result() >> 8) });
            eeprom.WriteBuffer(infra::MakeRange(copy), address, [] {});
            ExecuteAllActions();
        }

        std::optional<persistence::TuningRecord> Stored(uint32_t address)
        {
            std::array<uint8_t, persistence::PersistentBalanceControl::regionSize> buffer{};
            eeprom.ReadBuffer(infra::MakeRange(buffer), address, [] {});
            ExecuteAllActions();
            if (buffer[0] != 0x52 || buffer[1] != 0x53)
                return std::nullopt;
            return persistence::Decode(infra::Head(infra::DiscardHead(infra::MakeConstByteRange(buffer), 8), buffer[4]));
        }

        persistence::TuningRecord Record(float a)
        {
            persistence::TuningRecord record;
            record.activeStrategy = persistence::KeyOf("pid", "");
            record.entries.push_back({ persistence::KeyOf("pid", "a"), a });
            return record;
        }

        void Start(hal::Eeprom& storage)
        {
            store.emplace(control, storage, access);
            ExecuteAllActions();
        }

        void ExpectCapture(float a, float b, float c)
        {
            EXPECT_CALL(control, Engaged()).Times(2).WillRepeatedly(testing::Return(false));
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
        infra::ClaimableResource access;
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
    auto record = Record(0.25f);
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
    Store(0, Record(0.1f), 1);
    Store(persistence::PersistentBalanceControl::regionSize, Record(0.2f), 2);

    EXPECT_CALL(control, SetStrategyParameter(0, 0, 0.2f)).WillOnce(testing::Return(true));
    EXPECT_CALL(control, Select(0)).WillOnce(testing::Return(true));
    Start(eeprom);
}

TEST_F(PersistentBalanceControlTest, unreadable_copies_are_reported_as_corrupt)
{
    std::array<uint8_t, 4> garbage{ 0x01, 0x02, 0x03, 0x04 };
    eeprom.WriteBuffer(infra::MakeConstByteRange(garbage), 0, [] {});

    Start(eeprom);

    EXPECT_EQ(persistence::RestoreResult::corrupt, store->Restore());
}

TEST_F(PersistentBalanceControlTest, a_valid_copy_with_an_unknown_format_is_reported_as_corrupt)
{
    StorePayload(0, { 0x7f, 0x00, 0x00, 0x00 });

    Start(eeprom);

    EXPECT_EQ(persistence::RestoreResult::corrupt, store->Restore());
}

TEST_F(PersistentBalanceControlTest, unknown_keys_are_ignored)
{
    auto record = Record(0.1f);
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
    EXPECT_EQ(persistence::KeyOf("lqr", ""), saved->activeStrategy);
    ASSERT_EQ(3u, saved->entries.size());
    EXPECT_FLOAT_EQ(-1.5f, saved->entries[2].value);
    EXPECT_EQ(persistence::SaveResult::saved, store->LastSave());
}

TEST_F(PersistentBalanceControlTest, a_saved_record_is_restored_after_a_reset)
{
    Start(eeprom);
    AcceptParameter();
    ExpectCapture(0.5f, 0.75f, -1.5f);
    ForwardTime(2s);

    store.reset();
    EXPECT_CALL(control, SetStrategyParameter(0, 0, 0.5f)).WillOnce(testing::Return(true));
    EXPECT_CALL(control, SetStrategyParameter(0, 1, 0.75f)).WillOnce(testing::Return(true));
    EXPECT_CALL(control, SetStrategyParameter(1, 0, -1.5f)).WillOnce(testing::Return(true));
    EXPECT_CALL(control, Select(1)).WillOnce(testing::Return(true));
    Start(eeprom);
    EXPECT_EQ(persistence::RestoreResult::restored, store->Restore());
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

TEST_F(PersistentBalanceControlTest, the_save_waits_for_shared_access)
{
    Start(eeprom);
    infra::ClaimableResource::Claimer other{ access };
    other.Claim([] {});
    ExecuteAllActions();
    AcceptParameter();

    EXPECT_CALL(control, Engaged()).WillOnce(testing::Return(false));
    ForwardTime(2s);
    EXPECT_FALSE(Stored(0).has_value());

    EXPECT_CALL(control, Engaged()).WillOnce(testing::Return(false));
    EXPECT_CALL(control, ActiveStrategy()).WillOnce(testing::Return(0));
    EXPECT_CALL(control, StrategyParameter(0, 0)).WillOnce(testing::Return(0.5f));
    EXPECT_CALL(control, StrategyParameter(0, 1)).WillOnce(testing::Return(0.0f));
    EXPECT_CALL(control, StrategyParameter(1, 0)).WillOnce(testing::Return(0.0f));
    other.Release();
    ExecuteAllActions();
    EXPECT_TRUE(Stored(0).has_value());
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

TEST_F(PersistentBalanceControlTest, the_save_waits_for_the_restore)
{
    ControllableEeprom controllable;
    controllable.holdReads = true;
    Start(controllable);
    EXPECT_EQ(persistence::RestoreResult::pending, store->Restore());

    AcceptParameter();
    ForwardTime(2s);

    controllable.Release();
    ExecuteAllActions();
    controllable.holdReads = false;
    controllable.Release();
    ExecuteAllActions();
    EXPECT_EQ(persistence::RestoreResult::defaults, store->Restore());

    ExpectCapture(0.5f, 0.0f, 0.0f);
    ForwardTime(2s);
    EXPECT_EQ(persistence::SaveResult::saved, store->LastSave());
}
