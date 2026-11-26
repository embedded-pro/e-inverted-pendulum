#pragma once

#include "core/attitude_estimation/interfaces/AttitudeEstimation.hpp"
#include "core/calibration/interfaces/Calibration.hpp"
#include "core/inertial_sensing/interfaces/InertialSensing.hpp"
#include "core/motion_actuation/interfaces/MotionActuation.hpp"
#include "core/persistence/interfaces/TuningStore.hpp"
#include "core/wheel_odometry/interfaces/WheelOdometry.hpp"
#include "infra/util/BoundedString.hpp"
#include "infra/util/Tokenizer.hpp"
#include "services/tracer/Tracer.hpp"

namespace application
{
    class CalibrationCommand
    {
    public:
        struct Dependencies
        {
            calibration::Calibration& calibration;
            const persistence::TuningStore& store;
            const sensing::InertialSensing& sensing;
            const estimation::AttitudeEstimation& estimation;
            const motion::MotionActuation& actuation;
            const odometry::WheelOdometry& odometry;
        };

        CalibrationCommand(services::Tracer& tracer, const Dependencies& dependencies);
        CalibrationCommand(const CalibrationCommand& other) = delete;
        CalibrationCommand& operator=(const CalibrationCommand& other) = delete;

        void Execute(const infra::BoundedConstString& params);

    private:
        void Print();
        void PrintAxes();
        void PrintSigns();
        bool Dispatch(const infra::Tokenizer& tokens);
        bool Axes(const infra::Tokenizer& tokens);
        bool Still(const infra::Tokenizer& tokens);
        bool Upright(const infra::Tokenizer& tokens);
        bool Sign(const infra::Tokenizer& tokens, bool motor);
        bool Wheel(const infra::Tokenizer& tokens);
        void Report(calibration::Result result, bool needsCalibrate = false);

        services::Tracer& tracer;
        Dependencies dependencies;
    };
}
