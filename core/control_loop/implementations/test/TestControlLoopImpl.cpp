#include "core/control_loop/implementations/ControlLoopImpl.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include <chrono>

namespace
{
    using namespace std::chrono_literals;

    class InertialSensingMock
        : public sensing::InertialSensing
    {
    public:
        virtual ~InertialSensingMock() = default;

        MOCK_METHOD(sensing::Measurement, Latest, (), (const, override));
        MOCK_METHOD(void, OnMeasurement, (const infra::Function<void(const sensing::Measurement&)>& onMeasurement), (override));
        MOCK_METHOD(sensing::InvalidCause, Cause, (), (const, override));
        MOCK_METHOD(void, StartCalibration, (), (override));
        MOCK_METHOD(sensing::CalibrationState, Calibration, (), (const, override));
        MOCK_METHOD(platform::InertialAxes, GyroscopeBias, (), (const, override));
    };

    class AttitudeEstimationMock
        : public estimation::AttitudeEstimation
    {
    public:
        virtual ~AttitudeEstimationMock() = default;

        MOCK_METHOD(estimation::Estimate, Update, (const sensing::Measurement& measurement), (override));
        MOCK_METHOD(estimation::Estimate, Latest, (), (const, override));
        MOCK_METHOD(void, Select, (estimation::Filter filter), (override));
        MOCK_METHOD(estimation::Filter, Selected, (), (const, override));
    };

    class BalanceStageMock
        : public control::BalanceStage
    {
    public:
        virtual ~BalanceStageMock() = default;

        MOCK_METHOD(void, Balance, (const estimation::Estimate& estimate, infra::Duration interval), (override));
        MOCK_METHOD(void, Reset, (), (override));
    };

    class OuterStageMock
        : public control::OuterStage
    {
    public:
        virtual ~OuterStageMock() = default;

        MOCK_METHOD(void, Steer, (infra::Duration interval), (override));
    };

    MATCHER_P(PitchIs, pitch, "")
    {
        return arg.pitch == pitch;
    }

    class ControlLoopImplTest
        : public testing::Test
    {
    public:
        ControlLoopImplTest()
        {
            EXPECT_CALL(sensing, OnMeasurement(testing::_)).WillOnce(testing::SaveArg<0>(&onMeasurement));
            loop.emplace(sensing, estimation, balance, outer);
        }

        void Deliver(infra::Duration afterPrevious = 1ms, float pitch = 0.0f)
        {
            now += afterPrevious;

            sensing::Measurement measurement;
            measurement.sampledAt = now;
            measurement.valid = true;

            estimation::Estimate estimate;
            estimate.pitch = pitch;
            estimate.valid = true;

            EXPECT_CALL(estimation, Update(testing::Field(&sensing::Measurement::sampledAt, now))).WillOnce(testing::Return(estimate));
            onMeasurement(measurement);
        }

        void DeliverSilently(uint32_t samples)
        {
            EXPECT_CALL(balance, Balance(testing::_, testing::_)).Times(testing::AnyNumber());
            EXPECT_CALL(outer, Steer(testing::_)).Times(testing::AnyNumber());

            for (uint32_t sample = 0; sample < samples; ++sample)
                Deliver();

            testing::Mock::VerifyAndClearExpectations(&balance);
            testing::Mock::VerifyAndClearExpectations(&outer);
        }

        testing::StrictMock<InertialSensingMock> sensing;
        testing::StrictMock<AttitudeEstimationMock> estimation;
        testing::StrictMock<BalanceStageMock> balance;
        testing::StrictMock<OuterStageMock> outer;
        infra::Function<void(const sensing::Measurement&)> onMeasurement;
        std::optional<control::ControlLoopImpl> loop;
        infra::TimePoint now{ 1s };
    };
}

TEST_F(ControlLoopImplTest, every_sample_is_estimated_but_only_every_second_is_balanced)
{
    Deliver();

    EXPECT_CALL(balance, Balance(PitchIs(0.25f), infra::Duration{ 2ms }));
    Deliver(1ms, 0.25f);
}

TEST_F(ControlLoopImplTest, balance_receives_the_measured_interval_after_its_first_run)
{
    DeliverSilently(2);

    Deliver();
    EXPECT_CALL(balance, Balance(testing::_, infra::Duration{ 2500us }));
    Deliver(1500us);
}

TEST_F(ControlLoopImplTest, the_outer_stage_runs_on_every_tenth_balance_iteration)
{
    DeliverSilently(19);

    EXPECT_CALL(balance, Balance(testing::_, testing::_));
    EXPECT_CALL(outer, Steer(infra::Duration{ 20ms }));
    Deliver();
}

TEST_F(ControlLoopImplTest, the_outer_stage_receives_its_measured_interval)
{
    DeliverSilently(39);

    EXPECT_CALL(balance, Balance(testing::_, infra::Duration{ 7ms }));
    EXPECT_CALL(outer, Steer(infra::Duration{ 25ms }));
    Deliver(6ms);
}

TEST_F(ControlLoopImplTest, statistics_count_iterations_and_the_worst_jitter)
{
    DeliverSilently(6);

    EXPECT_EQ(3u, loop->Statistics().iterations);
    EXPECT_EQ(infra::Duration::zero(), loop->Statistics().worstJitter);
    EXPECT_EQ(0u, loop->Statistics().lateIterations);

    Deliver(900us);
    EXPECT_CALL(balance, Balance(testing::_, infra::Duration{ 1800us }));
    Deliver(900us);

    EXPECT_EQ(4u, loop->Statistics().iterations);
    EXPECT_EQ(infra::Duration{ 200us }, loop->Statistics().worstJitter);
    EXPECT_EQ(0u, loop->Statistics().lateIterations);
}

TEST_F(ControlLoopImplTest, an_interval_beyond_ten_percent_over_the_period_is_late)
{
    DeliverSilently(2);

    Deliver();
    EXPECT_CALL(balance, Balance(testing::_, infra::Duration{ 2200us }));
    Deliver(1200us);

    Deliver();
    EXPECT_CALL(balance, Balance(testing::_, infra::Duration{ 2201us }));
    Deliver(1201us);

    EXPECT_EQ(1u, loop->Statistics().lateIterations);
    EXPECT_EQ(infra::Duration{ 201us }, loop->Statistics().worstJitter);
}

TEST_F(ControlLoopImplTest, resetting_statistics_starts_counting_afresh)
{
    DeliverSilently(4);
    EXPECT_CALL(balance, Balance(testing::_, testing::_));
    Deliver();
    Deliver(5ms);

    loop->ResetStatistics();

    EXPECT_EQ(0u, loop->Statistics().iterations);
    EXPECT_EQ(infra::Duration::zero(), loop->Statistics().worstJitter);
    EXPECT_EQ(0u, loop->Statistics().lateIterations);
}
