#include "core/persistence/PersistingBalanceControl.hpp"
#include "core/persistence/test/Mocks.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "services/util/test_doubles/ConfigurationStoreMock.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <array>
#include <bit>
#include <chrono>

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

    persistence::StoredParameter Stored(const char* name, float value)
    {
        persistence::StoredParameter parameter;
        parameter.name = name;
        parameter.value = std::bit_cast<uint32_t>(value);
        return parameter;
    }

    class PersistingBalanceControlTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        PersistingBalanceControlTest()
        {
            EXPECT_CALL(control, StrategyCount()).WillRepeatedly(testing::Return(2));
            EXPECT_CALL(control, StrategyName(0)).WillRepeatedly(testing::Return("cascaded-pid"));
            EXPECT_CALL(control, StrategyName(1)).WillRepeatedly(testing::Return("lqr"));
            EXPECT_CALL(control, StrategyParameters(0)).WillRepeatedly(testing::Return(infra::MakeRange(pidParameters)));
            EXPECT_CALL(control, StrategyParameters(1)).WillRepeatedly(testing::Return(infra::MakeRange(lqrParameters)));
        }

        void ExpectCapture()
        {
            EXPECT_CALL(control, ActiveStrategy()).WillRepeatedly(testing::Return(1));
            EXPECT_CALL(control, StrategyParameter(0, 0)).WillRepeatedly(testing::Return(12.5f));
            EXPECT_CALL(control, StrategyParameter(0, 1)).WillRepeatedly(testing::Return(0.5f));
            EXPECT_CALL(control, StrategyParameter(1, 0)).WillRepeatedly(testing::Return(-30.0f));
        }

        void ExpectDisarmed()
        {
            EXPECT_CALL(supervisor, Current()).WillRepeatedly(testing::Return(safety::Mode::idle));
        }

        testing::StrictMock<persistence::BalanceControlMock> control;
        testing::StrictMock<persistence::SafetySupervisorMock> supervisor;
        testing::StrictMock<services::ConfigurationStoreInterfaceMock> storeInterface;
        persistence::Tuning tuning;
        persistence::PersistingBalanceControl persisting{ control, supervisor, services::ConfigurationStoreAccess<persistence::Tuning>{ storeInterface, tuning } };
    };
}

TEST_F(PersistingBalanceControlTest, restoring_an_empty_store_changes_nothing)
{
    persisting.Restore();
}

TEST_F(PersistingBalanceControlTest, restoring_applies_stored_values_by_strategy_and_parameter_name)
{
    tuning.activeStrategy = "lqr";
    tuning.strategies.emplace_back();
    auto& pid = tuning.strategies.back();
    pid.name = "cascaded-pid";
    pid.parameters.push_back(Stored("pitch.kd", 1.25f));
    pid.parameters.push_back(Stored("pitch.kp", 20.0f));
    tuning.strategies.emplace_back();
    auto& lqr = tuning.strategies.back();
    lqr.name = "lqr";
    lqr.parameters.push_back(Stored("k.pitch", -42.0f));

    EXPECT_CALL(control, SetStrategyParameter(0, 1, 1.25f)).WillOnce(testing::Return(true));
    EXPECT_CALL(control, SetStrategyParameter(0, 0, 20.0f)).WillOnce(testing::Return(true));
    EXPECT_CALL(control, SetStrategyParameter(1, 0, -42.0f)).WillOnce(testing::Return(true));
    EXPECT_CALL(control, Select(1)).WillOnce(testing::Return(true));

    persisting.Restore();
    ForwardTime(10s);
}

TEST_F(PersistingBalanceControlTest, restoring_ignores_unknown_strategies_and_parameters)
{
    tuning.activeStrategy = "retired";
    tuning.strategies.emplace_back();
    auto& retired = tuning.strategies.back();
    retired.name = "retired";
    retired.parameters.push_back(Stored("pitch.kp", 1.0f));
    tuning.strategies.emplace_back();
    auto& pid = tuning.strategies.back();
    pid.name = "cascaded-pid";
    pid.parameters.push_back(Stored("pitch.ki", 3.0f));

    persisting.Restore();
}

TEST_F(PersistingBalanceControlTest, an_accepted_parameter_write_saves_every_strategy_two_seconds_later)
{
    EXPECT_CALL(control, SetParameter(0, 12.5f)).WillOnce(testing::Return(true));
    EXPECT_TRUE(persisting.SetParameter(0, 12.5f));

    ForwardTime(1999ms);

    ExpectDisarmed();
    ExpectCapture();
    EXPECT_CALL(storeInterface, Write()).WillOnce(testing::Return(1));
    ForwardTime(1ms);

    EXPECT_EQ("lqr", tuning.activeStrategy);
    ASSERT_EQ(2u, tuning.strategies.size());
    EXPECT_EQ("cascaded-pid", tuning.strategies[0].name);
    ASSERT_EQ(2u, tuning.strategies[0].parameters.size());
    EXPECT_EQ("pitch.kp", tuning.strategies[0].parameters[0].name);
    EXPECT_EQ(12.5f, std::bit_cast<float>(tuning.strategies[0].parameters[0].value));
    EXPECT_EQ(0.5f, std::bit_cast<float>(tuning.strategies[0].parameters[1].value));
    EXPECT_EQ("lqr", tuning.strategies[1].name);
    EXPECT_EQ(-30.0f, std::bit_cast<float>(tuning.strategies[1].parameters[0].value));
}

TEST_F(PersistingBalanceControlTest, a_refused_change_does_not_save)
{
    EXPECT_CALL(control, SetParameter(0, 99.0f)).WillOnce(testing::Return(false));
    EXPECT_CALL(control, Select(3)).WillOnce(testing::Return(false));

    EXPECT_FALSE(persisting.SetParameter(0, 99.0f));
    EXPECT_FALSE(persisting.Select(3));

    ForwardTime(10s);
}

TEST_F(PersistingBalanceControlTest, a_burst_of_changes_saves_once_after_the_last)
{
    EXPECT_CALL(control, Select(1)).WillOnce(testing::Return(true));
    EXPECT_CALL(control, SetStrategyParameter(1, 0, -30.0f)).WillOnce(testing::Return(true));

    persisting.Select(1);
    ForwardTime(1500ms);
    persisting.SetStrategyParameter(1, 0, -30.0f);
    ForwardTime(1999ms);

    ExpectDisarmed();
    ExpectCapture();
    EXPECT_CALL(storeInterface, Write()).WillOnce(testing::Return(1));
    ForwardTime(1ms);
    ForwardTime(10s);
}

TEST_F(PersistingBalanceControlTest, a_save_that_falls_due_while_armed_waits_until_disarmed)
{
    EXPECT_CALL(control, SetParameter(1, 0.5f)).WillOnce(testing::Return(true));
    persisting.SetParameter(1, 0.5f);

    EXPECT_CALL(supervisor, Current()).WillRepeatedly(testing::Return(safety::Mode::armed));
    ForwardTime(6s);

    testing::Mock::VerifyAndClearExpectations(&supervisor);
    ExpectDisarmed();
    ExpectCapture();
    EXPECT_CALL(storeInterface, Write()).WillOnce(testing::Return(1));
    ForwardTime(2s);
}

TEST_F(PersistingBalanceControlTest, everything_else_is_forwarded)
{
    EXPECT_CALL(control, Move(balance::Setpoints{ 0.2f, 0.1f })).WillOnce(testing::Return(true));
    EXPECT_CALL(control, CancelMotion());
    EXPECT_CALL(control, Engaged()).WillOnce(testing::Return(true));
    EXPECT_CALL(control, AppliedEffort()).WillOnce(testing::Return(balance::Effort{ 0.3f, -0.3f }));
    EXPECT_CALL(control, Parameters()).WillOnce(testing::Return(infra::MakeRange(pidParameters)));
    EXPECT_CALL(control, Parameter(1)).WillOnce(testing::Return(0.5f));
    EXPECT_CALL(control, ActiveStrategy()).WillOnce(testing::Return(0));

    EXPECT_TRUE(persisting.Move(balance::Setpoints{ 0.2f, 0.1f }));
    persisting.CancelMotion();
    EXPECT_TRUE(persisting.Engaged());
    EXPECT_EQ((balance::Effort{ 0.3f, -0.3f }), persisting.AppliedEffort());
    EXPECT_EQ(2u, persisting.Parameters().size());
    EXPECT_EQ(0.5f, persisting.Parameter(1));
    EXPECT_EQ(0u, persisting.ActiveStrategy());
    EXPECT_EQ(2u, persisting.StrategyCount());
    EXPECT_STREQ("lqr", persisting.StrategyName(1));
    EXPECT_EQ(1u, persisting.StrategyParameters(1).size());
}
