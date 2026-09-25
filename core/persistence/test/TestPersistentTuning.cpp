#include "core/persistence/PersistentTuning.hpp"
#include "core/persistence/test/Mocks.hpp"
#include "hal/interfaces/test_doubles/FlashStub.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "services/crypto/Sha256Software.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <array>
#include <chrono>
#include <optional>

namespace
{
    using namespace std::chrono_literals;

    constexpr std::array<balance::ParameterDescriptor, 2> pidParameters{ {
        { "pitch.kp", 0.0f, 50.0f },
        { "pitch.kd", 0.0f, 5.0f },
    } };

    constexpr std::array<balance::ParameterDescriptor, 1> lqrParameters{ {
        { "k.pitch", -100.0f, 100.0f },
    } };

    class PersistentTuningTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        PersistentTuningTest()
        {
            EXPECT_CALL(control, StrategyCount()).WillRepeatedly(testing::Return(2));
            EXPECT_CALL(control, StrategyName(0)).WillRepeatedly(testing::Return("cascaded-pid"));
            EXPECT_CALL(control, StrategyName(1)).WillRepeatedly(testing::Return("lqr"));
            EXPECT_CALL(control, StrategyParameters(0)).WillRepeatedly(testing::Return(infra::MakeRange(pidParameters)));
            EXPECT_CALL(control, StrategyParameters(1)).WillRepeatedly(testing::Return(infra::MakeRange(lqrParameters)));
        }

        void PowerOn()
        {
            tuning.reset();
            tuning.emplace(first, second, sha256, control, supervisor);
            ExecuteAllActions();
        }

        hal::FlashStub first{ 1, 4096 };
        hal::FlashStub second{ 1, 4096 };
        services::Sha256Software sha256;
        testing::StrictMock<persistence::BalanceControlMock> control;
        testing::StrictMock<persistence::SafetySupervisorMock> supervisor;
        std::optional<persistence::PersistentTuning> tuning;
    };
}

TEST_F(PersistentTuningTest, an_erased_store_leaves_the_defaults_in_place)
{
    PowerOn();
}

TEST_F(PersistentTuningTest, tuning_saved_before_a_reset_is_restored_after_it)
{
    PowerOn();

    EXPECT_CALL(control, SetParameter(0, 12.5f)).WillOnce(testing::Return(true));
    tuning->Control().SetParameter(0, 12.5f);

    EXPECT_CALL(supervisor, Current()).WillRepeatedly(testing::Return(safety::Mode::idle));
    EXPECT_CALL(control, ActiveStrategy()).WillRepeatedly(testing::Return(1));
    EXPECT_CALL(control, StrategyParameter(0, 0)).WillRepeatedly(testing::Return(12.5f));
    EXPECT_CALL(control, StrategyParameter(0, 1)).WillRepeatedly(testing::Return(0.5f));
    EXPECT_CALL(control, StrategyParameter(1, 0)).WillRepeatedly(testing::Return(-30.0f));
    ForwardTime(2s);

    EXPECT_CALL(control, SetStrategyParameter(0, 0, 12.5f)).WillOnce(testing::Return(true));
    EXPECT_CALL(control, SetStrategyParameter(0, 1, 0.5f)).WillOnce(testing::Return(true));
    EXPECT_CALL(control, SetStrategyParameter(1, 0, -30.0f)).WillOnce(testing::Return(true));
    EXPECT_CALL(control, Select(1)).WillOnce(testing::Return(true));
    PowerOn();
}
