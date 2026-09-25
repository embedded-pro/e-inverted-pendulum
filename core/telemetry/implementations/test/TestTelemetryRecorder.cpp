#include "core/telemetry/implementations/TelemetryRecorder.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <chrono>

namespace
{
    using namespace std::chrono_literals;

    class BalanceStageMock
        : public control::BalanceStage
    {
    public:
        virtual ~BalanceStageMock() = default;

        MOCK_METHOD(void, Balance, (const estimation::Estimate& estimate, infra::Duration interval), (override));
    };

    class BalanceControlMock
        : public balance::BalanceControl
    {
    public:
        virtual ~BalanceControlMock() = default;

        MOCK_METHOD(std::size_t, StrategyCount, (), (const, override));
        MOCK_METHOD(const char*, StrategyName, (std::size_t index), (const, override));
        MOCK_METHOD(std::size_t, ActiveStrategy, (), (const, override));
        MOCK_METHOD(bool, Select, (std::size_t index), (override));
        MOCK_METHOD(infra::MemoryRange<const balance::ParameterDescriptor>, Parameters, (), (const, override));
        MOCK_METHOD(float, Parameter, (std::size_t index), (const, override));
        MOCK_METHOD(bool, SetParameter, (std::size_t index, float value), (override));
        MOCK_METHOD(infra::MemoryRange<const balance::ParameterDescriptor>, StrategyParameters, (std::size_t strategy), (const, override));
        MOCK_METHOD(float, StrategyParameter, (std::size_t strategy, std::size_t index), (const, override));
        MOCK_METHOD(bool, SetStrategyParameter, (std::size_t strategy, std::size_t index, float value), (override));
        MOCK_METHOD(bool, Move, (const balance::Setpoints& setpoints), (override));
        MOCK_METHOD(void, CancelMotion, (), (override));
        MOCK_METHOD(bool, Engaged, (), (const, override));
        MOCK_METHOD(balance::Effort, AppliedEffort, (), (const, override));
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

    class SafetySupervisorMock
        : public safety::SafetySupervisor
    {
    public:
        virtual ~SafetySupervisorMock() = default;

        MOCK_METHOD(bool, Arm, (), (override));
        MOCK_METHOD(bool, Disarm, (), (override));
        MOCK_METHOD(bool, ClearFault, (), (override));
        MOCK_METHOD(bool, Calibrate, (), (override));
        MOCK_METHOD(safety::Mode, Current, (), (const, override));
        MOCK_METHOD(safety::FaultCause, LatchedCause, (), (const, override));
        MOCK_METHOD(bool, DrivePermitted, (), (const, override));
    };

    class TelemetryRecorderTest
        : public testing::Test
    {
    public:
        testing::StrictMock<BalanceStageMock> next;
        testing::StrictMock<BalanceControlMock> balance;
        testing::StrictMock<WheelOdometryMock> odometry;
        testing::StrictMock<SafetySupervisorMock> supervisor;
        telemetry::TelemetryRecorder recorder{ next, balance, odometry, supervisor };
    };
}

TEST_F(TelemetryRecorderTest, nothing_recorded_yet_reads_as_an_empty_sample)
{
    EXPECT_EQ(telemetry::Sample{}, recorder.Latest());
}

TEST_F(TelemetryRecorderTest, the_iteration_runs_first_and_its_outcome_is_recorded_as_one_sample)
{
    const estimation::Estimate estimate{ 0.1f, -0.2f, true, estimation::InvalidCause::none };

    testing::InSequence iterationThenCapture;
    EXPECT_CALL(next, Balance(testing::Field(&estimation::Estimate::pitch, 0.1f), infra::Duration{ 2ms }));
    EXPECT_CALL(odometry, Chassis()).WillOnce(testing::Return(odometry::ChassisMotion{ 0.3f, 0.4f }));
    EXPECT_CALL(balance, AppliedEffort()).WillOnce(testing::Return(balance::Effort{ 0.5f, -0.6f }));
    EXPECT_CALL(supervisor, Current()).WillOnce(testing::Return(safety::Mode::fault));
    EXPECT_CALL(supervisor, LatchedCause()).WillOnce(testing::Return(safety::FaultCause::fall));

    recorder.Balance(estimate, 2ms);

    EXPECT_EQ((telemetry::Sample{ 0.1f, -0.2f, 0.3f, 0.4f, 0.5f, -0.6f, safety::Mode::fault, safety::FaultCause::fall }), recorder.Latest());
}
