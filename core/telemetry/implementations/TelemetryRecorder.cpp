#include "core/telemetry/implementations/TelemetryRecorder.hpp"

namespace telemetry
{
    TelemetryRecorder::TelemetryRecorder(control::BalanceStage& next, const balance::BalanceControl& balance, const odometry::WheelOdometry& odometry, const safety::SafetySupervisor& supervisor)
        : next(next)
        , balance(balance)
        , odometry(odometry)
        , supervisor(supervisor)
    {}

    Sample TelemetryRecorder::Latest() const
    {
        return latest;
    }

    void TelemetryRecorder::Balance(const estimation::Estimate& estimate, infra::Duration interval)
    {
        next.Balance(estimate, interval);

        const auto chassis = odometry.Chassis();
        const auto effort = balance.AppliedEffort();

        latest = Sample{ estimate.pitch, estimate.pitchRate, chassis.forwardVelocity, chassis.yawRate, effort.left, effort.right, supervisor.Current(), supervisor.LatchedCause() };
    }
}
