#pragma once

#include "core/attitude_estimation/interfaces/AttitudeEstimation.hpp"
#include "core/calibration/interfaces/Calibration.hpp"
#include "core/inertial_sensing/interfaces/InertialSensing.hpp"
#include "core/motion_actuation/interfaces/MotionActuation.hpp"
#include "core/wheel_odometry/interfaces/WheelOdometry.hpp"
#include "services/ble/GattServerCharacteristicImpl.hpp"
#include <array>

namespace ble
{
    class CalibrationAccess
        : private services::GattServerCharacteristicObserver
    {
    public:
        static constexpr std::size_t calibrationSize{ 30 };

        enum class Operation : uint8_t
        {
            readAll = 1,
            setAxisMap,
            setStillness,
            setUpright,
            captureUpright,
            setMotorSign,
            setEncoderSign,
            setGeometry
        };

        enum class Status : uint8_t
        {
            accepted,
            armed,
            restoring,
            invalidRequest,
            mtuTooSmall
        };

        struct Dependencies
        {
            calibration::Calibration& calibration;
            const sensing::InertialSensing& sensing;
            const estimation::AttitudeEstimation& estimation;
            const motion::MotionActuation& actuation;
            const odometry::WheelOdometry& odometry;
        };

        CalibrationAccess(services::GattServerService& service, const Dependencies& dependencies);
        ~CalibrationAccess();

        void Disconnected();
        void AttMtuChanged(uint16_t mtu);

    private:
        void DataReceived(infra::ConstByteRange data) override;

        std::size_t ResponseTo(infra::ConstByteRange data);
        std::size_t Change(Operation operation, uint8_t side, infra::ConstByteRange value);
        std::size_t ReadAll();
        std::size_t SetAxisMap(infra::ConstByteRange value);
        std::size_t SetFloat(Operation operation, infra::ConstByteRange value);
        std::size_t SetSign(Operation operation, uint8_t side, infra::ConstByteRange value);
        std::size_t SetGeometry(infra::ConstByteRange value);
        std::size_t Report(calibration::Result result);
        std::size_t Respond(Status status);
        void SendResponse(std::size_t length);

        Dependencies dependencies;
        services::GattServerCharacteristicImpl characteristic;
        uint16_t mtu{ services::attDefaultMaxMtuSize };
        bool inFlight{ false };
        std::array<uint8_t, calibrationSize> buffer{};
    };
}
