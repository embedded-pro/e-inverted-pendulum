#pragma once

#include "core/balance_control/interfaces/BalanceControl.hpp"
#include "core/control_loop/interfaces/Stages.hpp"
#include "core/telemetry/interfaces/Telemetry.hpp"
#include "core/wheel_odometry/interfaces/WheelOdometry.hpp"

namespace telemetry
{
    class TelemetryRecorder final
        : public TelemetrySource
        , public control::BalanceStage
    {
    public:
        TelemetryRecorder(control::BalanceStage& next, const balance::BalanceControl& balance, const odometry::WheelOdometry& odometry, const safety::SafetySupervisor& supervisor);

        Sample Latest() const override;
        void Balance(const estimation::Estimate& estimate, infra::Duration interval) override;

    private:
        control::BalanceStage& next;
        const balance::BalanceControl& balance;
        const odometry::WheelOdometry& odometry;
        const safety::SafetySupervisor& supervisor;
        Sample latest;
    };
}
