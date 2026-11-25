#include "core/motion_actuation/implementations/MotionActuationImpl.hpp"

namespace motion
{
    MotionActuationImpl::Config::Config() = default;

    MotionActuationImpl::MotionActuationImpl(platform::MotorDriver& motors, DriverConfiguration& driverConfiguration, const Config& config)
        : motors(motors)
        , decay(config.decay)
    {
        motors.SetBaseFrequency(config.switchingFrequency);
        ReleaseBridges();

        motors.EnableFaultNotification([this]()
            {
                OnFault();
            });

        driverConfiguration.Configure([this](bool verified)
            {
                state = verified ? DriverState::ready : DriverState::failed;
            });
    }

    MotionActuationImpl::~MotionActuationImpl()
    {
        motors.DisableFaultNotification();
        ReleaseBridges();
    }

    void MotionActuationImpl::Apply(float effortLeft, float effortRight)
    {
        if (!DrivePermitted())
            return;

        const auto left = Oriented(WheelSide::left, effortLeft);
        const auto right = Oriented(WheelSide::right, effortRight);
        motors.Drive(InputsFor(left, decay), InputsFor(right, decay));
    }

    void MotionActuationImpl::Disable(DisableState disableState)
    {
        if (disableState == DisableState::brake && DrivePermitted())
            motors.Drive(BrakedInputs(), BrakedInputs());
        else
            ReleaseBridges();
    }

    FaultCause MotionActuationImpl::Fault() const
    {
        return fault;
    }

    void MotionActuationImpl::ClearFault()
    {
        fault = FaultCause::none;
    }

    DriverState MotionActuationImpl::State() const
    {
        return state;
    }

    void MotionActuationImpl::SetMotorInverted(WheelSide side, bool inverted)
    {
        motorInverted[static_cast<std::size_t>(side)] = inverted;
    }

    bool MotionActuationImpl::MotorInverted(WheelSide side) const
    {
        return motorInverted[static_cast<std::size_t>(side)];
    }

    float MotionActuationImpl::Oriented(WheelSide side, float effort) const
    {
        return MotorInverted(side) ? -effort : effort;
    }

    bool MotionActuationImpl::DrivePermitted() const
    {
        return state == DriverState::ready && fault == FaultCause::none;
    }

    void MotionActuationImpl::ReleaseBridges() const
    {
        motors.Drive(ReleasedInputs(), ReleasedInputs());
    }

    void MotionActuationImpl::OnFault()
    {
        fault = FaultCause::driverFault;
        ReleaseBridges();
    }
}
