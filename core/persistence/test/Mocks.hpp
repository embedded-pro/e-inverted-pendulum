#pragma once

#include "core/balance_control/interfaces/BalanceControl.hpp"
#include "core/safety_supervisor/interfaces/SafetySupervisor.hpp"
#include "gmock/gmock.h"

namespace persistence
{
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
}
