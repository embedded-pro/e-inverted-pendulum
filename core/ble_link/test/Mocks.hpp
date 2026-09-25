#pragma once

#include "core/balance_control/interfaces/BalanceControl.hpp"
#include "core/platform_abstraction/Bluetooth.hpp"
#include "core/safety_supervisor/interfaces/SafetySupervisor.hpp"
#include "core/telemetry/interfaces/Telemetry.hpp"
#include "gmock/gmock.h"

namespace ble
{
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
        MOCK_METHOD(bool, Move, (const balance::Setpoints& setpoints), (override));
        MOCK_METHOD(void, CancelMotion, (), (override));
        MOCK_METHOD(bool, Engaged, (), (const, override));
        MOCK_METHOD(balance::Effort, AppliedEffort, (), (const, override));
    };

    class TelemetrySourceMock
        : public telemetry::TelemetrySource
    {
    public:
        virtual ~TelemetrySourceMock() = default;

        MOCK_METHOD(telemetry::Sample, Latest, (), (const, override));
    };

    class BluetoothMock
        : public platform::Bluetooth
    {
    public:
        virtual ~BluetoothMock() = default;

        MOCK_METHOD(services::GapPeripheral&, Gap, (), (override));
        MOCK_METHOD(services::GattServer&, GattServer, (), (override));
        MOCK_METHOD(services::GattClient&, GattClient, (), (override));
    };
}
