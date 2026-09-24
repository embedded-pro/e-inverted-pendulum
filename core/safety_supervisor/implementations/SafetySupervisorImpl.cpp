#include "core/safety_supervisor/implementations/SafetySupervisorImpl.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <cmath>

namespace safety
{
    SafetySupervisorImpl::Config::Config() = default;

    SafetySupervisorImpl::SafetySupervisorImpl(motion::MotionActuation& actuation, sensing::InertialSensing& sensing, control::BalanceStage& strategy, const Config& config)
        : actuation(actuation)
        , sensing(sensing)
        , strategy(strategy)
        , config(config)
        , missedServices(config.missedServiceLimit)
        , started(infra::Now())
    {
        really_assert(config.servicePeriod.count() > 0);
        really_assert(config.missedServiceLimit > 0);

        supervision.Start(config.servicePeriod, [this]()
            {
                Supervise();
            });
    }

    bool SafetySupervisorImpl::Arm()
    {
        if (mode != Mode::idle || !latest.valid || !Upright() || !LoopServiced() || !DriverHealthy())
            return false;

        strategy.Reset();
        mode = Mode::armed;
        return true;
    }

    bool SafetySupervisorImpl::Disarm()
    {
        if (mode != Mode::armed)
            return false;

        actuation.Disable(motion::DisableState::tristate);
        mode = Mode::idle;
        return true;
    }

    bool SafetySupervisorImpl::ClearFault()
    {
        if (mode != Mode::fault)
            return false;

        actuation.ClearFault();
        cause = FaultCause::none;
        mode = Mode::idle;
        return true;
    }

    bool SafetySupervisorImpl::Calibrate()
    {
        if (mode != Mode::idle)
            return false;

        StartCalibration();
        return true;
    }

    Mode SafetySupervisorImpl::Current() const
    {
        return mode;
    }

    FaultCause SafetySupervisorImpl::LatchedCause() const
    {
        return cause;
    }

    bool SafetySupervisorImpl::DrivePermitted() const
    {
        return mode == Mode::armed;
    }

    void SafetySupervisorImpl::Balance(const estimation::Estimate& estimate, infra::Duration interval)
    {
        serviced = true;
        latest = estimate;

        if (mode != Mode::armed)
            return;

        if (!estimate.valid)
            EnterFault(FaultCause::estimateInvalid);
        else if (std::fabs(estimate.pitch) > config.fallThreshold)
            EnterFault(FaultCause::fall);
        else if (actuation.Fault() != motion::FaultCause::none)
            EnterFault(FaultCause::driverFault);
        else
            strategy.Balance(estimate, interval);
    }

    void SafetySupervisorImpl::Reset()
    {}

    void SafetySupervisorImpl::Supervise()
    {
        switch (mode)
        {
            case Mode::init:
                SuperviseSelfTest();
                break;
            case Mode::calibrating:
                SuperviseCalibration();
                break;
            case Mode::idle:
            case Mode::armed:
                SuperviseLiveness();
                break;
            default:
                break;
        }
    }

    void SafetySupervisorImpl::SuperviseSelfTest()
    {
        if (SelfTestPassed())
            StartCalibration();
        else if (infra::Now() - started >= config.selfTestDeadline)
            EnterFault(FaultCause::selfTestFailed);
    }

    void SafetySupervisorImpl::SuperviseCalibration()
    {
        const auto calibration = sensing.Calibration();

        if (calibration == sensing::CalibrationState::calibrated)
            mode = Mode::idle;
        else if (calibration == sensing::CalibrationState::failed)
            EnterFault(FaultCause::calibrationFailed);
    }

    void SafetySupervisorImpl::SuperviseLiveness()
    {
        if (actuation.Fault() != motion::FaultCause::none)
        {
            EnterFault(FaultCause::driverFault);
            return;
        }

        if (serviced)
            missedServices = 0;
        else if (missedServices < config.missedServiceLimit)
            ++missedServices;

        serviced = false;

        if (mode == Mode::armed && missedServices >= config.missedServiceLimit)
            EnterFault(FaultCause::loopStalled);
    }

    bool SafetySupervisorImpl::SelfTestPassed() const
    {
        const auto sensingCause = sensing.Cause();
        const bool sampling = sensingCause == sensing::InvalidCause::none || sensingCause == sensing::InvalidCause::uncalibrated;

        return actuation.State() == motion::DriverState::ready && sampling;
    }

    bool SafetySupervisorImpl::LoopServiced() const
    {
        return serviced || missedServices == 0;
    }

    bool SafetySupervisorImpl::DriverHealthy() const
    {
        return actuation.State() == motion::DriverState::ready && actuation.Fault() == motion::FaultCause::none;
    }

    bool SafetySupervisorImpl::Upright() const
    {
        return std::fabs(latest.pitch) <= config.armWindow;
    }

    void SafetySupervisorImpl::StartCalibration()
    {
        sensing.StartCalibration();
        mode = Mode::calibrating;
    }

    void SafetySupervisorImpl::EnterFault(FaultCause faultCause)
    {
        actuation.Disable(motion::DisableState::tristate);
        cause = faultCause;
        mode = Mode::fault;
    }
}
