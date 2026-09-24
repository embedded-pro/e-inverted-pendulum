#include "core/balance_control/implementations/BalanceControlImpl.hpp"
#include "infra/timer/test_helper/ClockFixture.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <array>
#include <chrono>
#include <cmath>
#include <limits>

namespace
{
    using namespace std::chrono_literals;

    class ControlStrategyMock
        : public balance::ControlStrategy
    {
    public:
        virtual ~ControlStrategyMock() = default;

        MOCK_METHOD(const char*, Name, (), (const, override));
        MOCK_METHOD(void, Reset, (), (override));
        MOCK_METHOD(void, Steer, (const balance::Setpoints& setpoints, const odometry::ChassisMotion& chassis, float intervalSeconds), (override));
        MOCK_METHOD(balance::Effort, Balance, (const estimation::Estimate& estimate, float intervalSeconds), (override));
        MOCK_METHOD(void, Saturated, (const balance::Effort& applied), (override));
        MOCK_METHOD(infra::MemoryRange<const balance::ParameterDescriptor>, Parameters, (), (const, override));
        MOCK_METHOD(float, Parameter, (std::size_t index), (const, override));
        MOCK_METHOD(void, SetParameter, (std::size_t index, float value), (override));
    };

    class MotionActuationMock
        : public motion::MotionActuation
    {
    public:
        virtual ~MotionActuationMock() = default;

        MOCK_METHOD(void, Apply, (float effortLeft, float effortRight), (override));
        MOCK_METHOD(void, Disable, (motion::DisableState state), (override));
        MOCK_METHOD(motion::FaultCause, Fault, (), (const, override));
        MOCK_METHOD(void, ClearFault, (), (override));
        MOCK_METHOD(motion::DriverState, State, (), (const, override));
    };

    class WheelOdometryMock
        : public odometry::WheelOdometry
    {
    public:
        virtual ~WheelOdometryMock() = default;

        MOCK_METHOD(odometry::WheelMotion, Left, (), (const, override));
        MOCK_METHOD(odometry::WheelMotion, Right, (), (const, override));
        MOCK_METHOD(odometry::ChassisMotion, Chassis, (), (const, override));
    };

    MATCHER_P2(SetpointsNear, velocity, yawRate, "")
    {
        return std::fabs(arg.velocity - velocity) < 1e-5f && std::fabs(arg.yawRate - yawRate) < 1e-5f;
    }

    MATCHER_P2(ChassisIs, velocity, yawRate, "")
    {
        return arg.forwardVelocity == velocity && arg.yawRate == yawRate;
    }

    constexpr std::array<balance::ParameterDescriptor, 2> descriptors{ {
        { "gain.a", 0.0f, 1.0f },
        { "gain.b", -2.0f, 2.0f },
    } };

    const estimation::Estimate upright{ 0.0f, 0.0f, true, estimation::InvalidCause::none };

    class BalanceControlImplTest
        : public testing::Test
        , public infra::ClockFixture
    {
    public:
        void Engage()
        {
            EXPECT_CALL(first, Reset());
            control.Engage();
        }

        void ExpectSteer(float velocity, float yawRate)
        {
            EXPECT_CALL(odometry, Chassis()).WillOnce(testing::Return(odometry::ChassisMotion{}));
            EXPECT_CALL(first, Steer(SetpointsNear(velocity, yawRate), testing::_, testing::_));
        }

        testing::StrictMock<ControlStrategyMock> first;
        testing::StrictMock<ControlStrategyMock> second;
        std::array<balance::ControlStrategy*, 2> strategies{ &first, &second };
        testing::StrictMock<MotionActuationMock> actuation;
        testing::StrictMock<WheelOdometryMock> odometry;
        balance::BalanceControlImpl control{ infra::MakeRange(strategies), actuation, odometry };
    };
}

TEST_F(BalanceControlImplTest, starts_disengaged_with_the_first_strategy_active)
{
    EXPECT_FALSE(control.Engaged());
    EXPECT_EQ(2u, control.StrategyCount());
    EXPECT_EQ(0u, control.ActiveStrategy());
}

TEST_F(BalanceControlImplTest, reports_strategy_names)
{
    EXPECT_CALL(second, Name()).WillOnce(testing::Return("second"));

    EXPECT_STREQ("second", control.StrategyName(1));
}

TEST_F(BalanceControlImplTest, iterations_do_nothing_while_disengaged)
{
    control.Balance(upright, 2ms);
    control.Steer(20ms);
}

TEST_F(BalanceControlImplTest, engaging_resets_the_active_strategy)
{
    Engage();

    EXPECT_TRUE(control.Engaged());
}

TEST_F(BalanceControlImplTest, an_engaged_iteration_applies_the_strategy_effort)
{
    Engage();

    EXPECT_CALL(first, Balance(testing::_, testing::FloatEq(0.002f))).WillOnce(testing::Return(balance::Effort{ 0.4f, -0.2f }));
    EXPECT_CALL(actuation, Apply(testing::FloatEq(0.4f), testing::FloatEq(-0.2f)));
    control.Balance(upright, 2ms);
}

TEST_F(BalanceControlImplTest, effort_beyond_the_actuator_range_is_clamped_and_reported)
{
    Engage();

    EXPECT_CALL(first, Balance(testing::_, testing::_)).WillOnce(testing::Return(balance::Effort{ 1.5f, -0.3f }));
    EXPECT_CALL(first, Saturated(balance::Effort{ 1.0f, -0.3f }));
    EXPECT_CALL(actuation, Apply(testing::FloatEq(1.0f), testing::FloatEq(-0.3f)));
    control.Balance(upright, 2ms);
}

TEST_F(BalanceControlImplTest, an_invalid_estimate_produces_zero_effort)
{
    Engage();

    EXPECT_CALL(actuation, Apply(0.0f, 0.0f));
    control.Balance(estimation::Estimate{}, 2ms);
}

TEST_F(BalanceControlImplTest, steering_passes_the_setpoints_and_measured_chassis_motion)
{
    Engage();
    EXPECT_TRUE(control.Move(balance::Setpoints{ 0.5f, -0.2f }));

    EXPECT_CALL(odometry, Chassis()).WillOnce(testing::Return(odometry::ChassisMotion{ 0.1f, 0.3f }));
    EXPECT_CALL(first, Steer(balance::Setpoints{ 0.5f, -0.2f }, ChassisIs(0.1f, 0.3f), testing::FloatEq(0.02f)));
    control.Steer(20ms);
}

TEST_F(BalanceControlImplTest, a_command_holds_for_the_command_timeout)
{
    Engage();
    EXPECT_TRUE(control.Move(balance::Setpoints{ 0.5f, 0.8f }));

    ForwardTime(500ms);
    ExpectSteer(0.5f, 0.8f);
    control.Steer(20ms);
}

TEST_F(BalanceControlImplTest, after_the_timeout_setpoints_decay_at_a_fixed_deceleration)
{
    Engage();
    EXPECT_TRUE(control.Move(balance::Setpoints{ 0.5f, -0.8f }));
    ForwardTime(501ms);

    ExpectSteer(0.49f, -0.768f);
    control.Steer(20ms);

    ExpectSteer(0.47f, -0.704f);
    control.Steer(40ms);
}

TEST_F(BalanceControlImplTest, decay_stops_at_zero)
{
    Engage();
    EXPECT_TRUE(control.Move(balance::Setpoints{ -0.005f, 0.01f }));
    ForwardTime(501ms);

    ExpectSteer(0.0f, 0.0f);
    control.Steer(20ms);
}

TEST_F(BalanceControlImplTest, cancelling_motion_starts_the_decay_at_once)
{
    Engage();
    EXPECT_TRUE(control.Move(balance::Setpoints{ 0.5f, 0.0f }));

    control.CancelMotion();

    ExpectSteer(0.49f, 0.0f);
    control.Steer(20ms);
}

TEST_F(BalanceControlImplTest, a_new_command_stops_the_decay)
{
    Engage();
    EXPECT_TRUE(control.Move(balance::Setpoints{ 0.5f, 0.0f }));
    control.CancelMotion();
    ExpectSteer(0.49f, 0.0f);
    control.Steer(20ms);

    EXPECT_TRUE(control.Move(balance::Setpoints{ 0.3f, 0.0f }));

    ExpectSteer(0.3f, 0.0f);
    control.Steer(20ms);
}

TEST_F(BalanceControlImplTest, the_applied_effort_is_reported)
{
    EXPECT_EQ((balance::Effort{}), control.AppliedEffort());
    Engage();

    EXPECT_CALL(first, Balance(testing::_, testing::_)).WillOnce(testing::Return(balance::Effort{ 1.5f, -0.3f }));
    EXPECT_CALL(first, Saturated(testing::_));
    EXPECT_CALL(actuation, Apply(testing::_, testing::_));
    control.Balance(upright, 2ms);

    EXPECT_EQ((balance::Effort{ 1.0f, -0.3f }), control.AppliedEffort());
}

TEST_F(BalanceControlImplTest, no_effort_is_reported_for_an_invalid_estimate_or_after_disengaging)
{
    Engage();
    EXPECT_CALL(first, Balance(testing::_, testing::_)).WillOnce(testing::Return(balance::Effort{ 0.5f, 0.5f }));
    EXPECT_CALL(actuation, Apply(testing::_, testing::_)).Times(2);
    control.Balance(upright, 2ms);

    control.Balance(estimation::Estimate{}, 2ms);
    EXPECT_EQ((balance::Effort{}), control.AppliedEffort());

    EXPECT_CALL(first, Balance(testing::_, testing::_)).WillOnce(testing::Return(balance::Effort{ 0.5f, 0.5f }));
    EXPECT_CALL(actuation, Apply(testing::_, testing::_));
    control.Balance(upright, 2ms);
    control.Disengage();
    EXPECT_EQ((balance::Effort{}), control.AppliedEffort());
}

TEST_F(BalanceControlImplTest, move_is_refused_while_disengaged)
{
    EXPECT_FALSE(control.Move(balance::Setpoints{ 0.1f, 0.0f }));
}

TEST_F(BalanceControlImplTest, move_is_refused_out_of_range)
{
    Engage();

    EXPECT_FALSE(control.Move(balance::Setpoints{ 1.01f, 0.0f }));
    EXPECT_FALSE(control.Move(balance::Setpoints{ 0.0f, -1.61f }));
    EXPECT_TRUE(control.Move(balance::Setpoints{ -1.0f, 1.6f }));
}

TEST_F(BalanceControlImplTest, disengaging_clears_the_setpoints)
{
    Engage();
    EXPECT_TRUE(control.Move(balance::Setpoints{ 0.5f, 0.5f }));

    control.Disengage();
    Engage();

    EXPECT_CALL(odometry, Chassis()).WillOnce(testing::Return(odometry::ChassisMotion{}));
    EXPECT_CALL(first, Steer(balance::Setpoints{}, testing::_, testing::_));
    control.Steer(20ms);
}

TEST_F(BalanceControlImplTest, selecting_a_strategy_switches_the_one_that_runs)
{
    EXPECT_TRUE(control.Select(1));
    EXPECT_EQ(1u, control.ActiveStrategy());

    EXPECT_CALL(second, Reset());
    control.Engage();
}

TEST_F(BalanceControlImplTest, selection_is_refused_while_engaged)
{
    Engage();

    EXPECT_FALSE(control.Select(1));
    EXPECT_EQ(0u, control.ActiveStrategy());
}

TEST_F(BalanceControlImplTest, selecting_an_unknown_strategy_is_refused)
{
    EXPECT_FALSE(control.Select(2));
}

TEST_F(BalanceControlImplTest, parameters_are_those_of_the_active_strategy)
{
    EXPECT_CALL(first, Parameters()).WillOnce(testing::Return(infra::MakeRange(descriptors)));
    EXPECT_EQ(2u, control.Parameters().size());

    EXPECT_CALL(first, Parameter(1)).WillOnce(testing::Return(0.75f));
    EXPECT_FLOAT_EQ(0.75f, control.Parameter(1));
}

TEST_F(BalanceControlImplTest, a_parameter_within_range_is_written)
{
    EXPECT_CALL(first, Parameters()).WillOnce(testing::Return(infra::MakeRange(descriptors)));
    EXPECT_CALL(first, SetParameter(1, -1.5f));

    EXPECT_TRUE(control.SetParameter(1, -1.5f));
}

TEST_F(BalanceControlImplTest, a_parameter_outside_its_range_is_rejected)
{
    EXPECT_CALL(first, Parameters()).WillRepeatedly(testing::Return(infra::MakeRange(descriptors)));

    EXPECT_FALSE(control.SetParameter(0, 1.5f));
    EXPECT_FALSE(control.SetParameter(1, -2.5f));
    EXPECT_FALSE(control.SetParameter(2, 0.0f));
}

TEST_F(BalanceControlImplTest, parameter_writes_are_refused_while_engaged)
{
    Engage();
    EXPECT_CALL(first, Parameters()).WillOnce(testing::Return(infra::MakeRange(descriptors)));

    EXPECT_FALSE(control.SetParameter(0, 0.5f));
}

TEST_F(BalanceControlImplTest, a_move_that_is_not_a_number_is_refused)
{
    Engage();

    EXPECT_FALSE(control.Move(balance::Setpoints{ std::numeric_limits<float>::quiet_NaN(), 0.0f }));
    EXPECT_FALSE(control.Move(balance::Setpoints{ 0.0f, std::numeric_limits<float>::infinity() }));
}

TEST_F(BalanceControlImplTest, a_parameter_that_is_not_a_number_is_rejected)
{
    EXPECT_CALL(first, Parameters()).WillOnce(testing::Return(infra::MakeRange(descriptors)));

    EXPECT_FALSE(control.SetParameter(0, std::numeric_limits<float>::quiet_NaN()));
}
