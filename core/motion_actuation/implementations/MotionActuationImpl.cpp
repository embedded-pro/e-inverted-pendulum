#include "core/motion_actuation/implementations/MotionActuationImpl.hpp"
#include <algorithm>
#include <cmath>

namespace motion
{
    namespace
    {
        hal::DutyCycle Off()
        {
            return hal::DutyCycle::FromPercent(0);
        }

        hal::DutyCycle Full()
        {
            return hal::DutyCycle::FromPercent(100);
        }

        hal::DutyCycle DutyOf(float magnitude)
        {
            return hal::DutyCycle{ static_cast<uint32_t>(std::lround(std::clamp(magnitude, 0.0f, 1.0f) * static_cast<float>(hal::DutyCycle::fullScale))) };
        }
    }

    MotionActuationImpl::Config::Config() = default;

    MotionActuationImpl::MotionActuationImpl(platform::MotorDriver& motors, const Config& config)
        : motors(motors)
    {
        motors.Left().SetBaseFrequency(config.switchingFrequency);
        motors.Right().SetBaseFrequency(config.switchingFrequency);

        ReleaseBridges();

        motors.EnableFaultNotification([this]()
            {
                OnFault();
            });
    }

    MotionActuationImpl::~MotionActuationImpl()
    {
        motors.DisableFaultNotification();
        ReleaseBridges();
    }

    void MotionActuationImpl::Apply(float effortLeft, float effortRight)
    {
        if (fault != FaultCause::none)
            return;

        ApplyTo(motors.Left(), effortLeft);
        ApplyTo(motors.Right(), effortRight);
    }

    void MotionActuationImpl::ApplyTo(platform::MotorBridge& bridge, float effort) const
    {
        const auto clamped = std::clamp(effort, -1.0f, 1.0f);
        const auto duty = DutyOf(std::fabs(clamped));

        if (clamped >= 0.0f)
            bridge.Start(duty, Off());
        else
            bridge.Start(Off(), duty);
    }

    void MotionActuationImpl::Disable(DisableState state)
    {
        if (state == DisableState::brake)
        {
            motors.Left().Start(Full(), Full());
            motors.Right().Start(Full(), Full());
        }
        else
        {
            ReleaseBridges();
        }
    }

    void MotionActuationImpl::ReleaseBridges() const
    {
        motors.Left().Stop();
        motors.Right().Stop();
    }

    FaultCause MotionActuationImpl::Fault() const
    {
        return fault;
    }

    void MotionActuationImpl::ClearFault()
    {
        fault = FaultCause::none;
    }

    void MotionActuationImpl::OnFault()
    {
        fault = FaultCause::driverFault;
        ReleaseBridges();
    }
}
