#pragma once

#include "core/attitude_estimation/interfaces/AttitudeEstimation.hpp"
#include "core/balance_control/interfaces/BalanceControl.hpp"
#include "core/calibration/interfaces/Calibration.hpp"
#include "core/inertial_sensing/interfaces/InertialSensing.hpp"
#include "core/motion_actuation/interfaces/MotionActuation.hpp"
#include "core/platform_abstraction/Bluetooth.hpp"
#include "core/safety_supervisor/interfaces/SafetySupervisor.hpp"
#include "core/telemetry/interfaces/Telemetry.hpp"
#include "core/wheel_odometry/interfaces/WheelOdometry.hpp"
#include "gmock/gmock.h"

namespace ble
{
    class SafetySupervisorMock
        : public safety::SafetySupervisor
    {
    public:
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

    class TelemetrySourceMock
        : public telemetry::TelemetrySource
    {
    public:
        MOCK_METHOD(telemetry::Sample, Latest, (), (const, override));
    };

    class BluetoothMock
        : public platform::Bluetooth
    {
    public:
        MOCK_METHOD(services::GapPeripheral&, Gap, (), (override));
        MOCK_METHOD(services::GattServer&, GattServer, (), (override));
    };

    class CalibrationMock
        : public calibration::Calibration
    {
    public:
        MOCK_METHOD(calibration::Result, SetAxisMap, (const platform::AxisMap& map), (override));
        MOCK_METHOD(calibration::Result, SetStillnessThreshold, (float threshold), (override));
        MOCK_METHOD(calibration::Result, SetUprightOffset, (float offset), (override));
        MOCK_METHOD(calibration::Result, CaptureUpright, (), (override));
        MOCK_METHOD(calibration::Result, SetMotorInverted, (motion::WheelSide side, bool inverted), (override));
        MOCK_METHOD(calibration::Result, SetEncoderInverted, (odometry::WheelSide side, bool inverted), (override));
        MOCK_METHOD(calibration::Result, SetGeometry, (const odometry::Geometry& geometry), (override));
    };

    class InertialSensingMock
        : public sensing::InertialSensing
    {
    public:
        MOCK_METHOD(sensing::Measurement, Latest, (), (const, override));
        MOCK_METHOD(void, OnMeasurement, (const infra::Function<void(const sensing::Measurement&)>& onMeasurement), (override));
        MOCK_METHOD(sensing::InvalidCause, Cause, (), (const, override));
        MOCK_METHOD(void, StartCalibration, (), (override));
        MOCK_METHOD(sensing::CalibrationState, Calibration, (), (const, override));
        MOCK_METHOD(platform::InertialAxes, GyroscopeBias, (), (const, override));
        MOCK_METHOD(bool, SetAxisMap, (const platform::AxisMap& map), (override));
        MOCK_METHOD(platform::AxisMap, CurrentAxisMap, (), (const, override));
        MOCK_METHOD(bool, SetStillnessThreshold, (float threshold), (override));
        MOCK_METHOD(float, StillnessThreshold, (), (const, override));
    };

    class AttitudeEstimationMock
        : public estimation::AttitudeEstimation
    {
    public:
        MOCK_METHOD(estimation::Estimate, Update, (const sensing::Measurement& measurement), (override));
        MOCK_METHOD(estimation::Estimate, Latest, (), (const, override));
        MOCK_METHOD(void, Select, (estimation::Filter filter), (override));
        MOCK_METHOD(estimation::Filter, Selected, (), (const, override));
        MOCK_METHOD(bool, SetUprightOffset, (float offset), (override));
        MOCK_METHOD(float, UprightOffset, (), (const, override));
    };

    class MotionActuationMock
        : public motion::MotionActuation
    {
    public:
        MOCK_METHOD(void, Apply, (float effortLeft, float effortRight), (override));
        MOCK_METHOD(void, Disable, (motion::DisableState state), (override));
        MOCK_METHOD(motion::FaultCause, Fault, (), (const, override));
        MOCK_METHOD(void, ClearFault, (), (override));
        MOCK_METHOD(motion::DriverState, State, (), (const, override));
        MOCK_METHOD(void, SetMotorInverted, (motion::WheelSide side, bool inverted), (override));
        MOCK_METHOD(bool, MotorInverted, (motion::WheelSide side), (const, override));
    };

    class WheelOdometryMock
        : public odometry::WheelOdometry
    {
    public:
        MOCK_METHOD(odometry::WheelMotion, Left, (), (const, override));
        MOCK_METHOD(odometry::WheelMotion, Right, (), (const, override));
        MOCK_METHOD(odometry::ChassisMotion, Chassis, (), (const, override));
        MOCK_METHOD(void, SetEncoderInverted, (odometry::WheelSide side, bool inverted), (override));
        MOCK_METHOD(bool, EncoderInverted, (odometry::WheelSide side), (const, override));
        MOCK_METHOD(bool, SetGeometry, (const odometry::Geometry& geometry), (override));
        MOCK_METHOD(odometry::Geometry, CurrentGeometry, (), (const, override));
    };
}
