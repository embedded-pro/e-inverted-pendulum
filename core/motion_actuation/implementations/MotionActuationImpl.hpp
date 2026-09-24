#pragma once

#include "core/motion_actuation/implementations/BridgeMapping.hpp"
#include "core/motion_actuation/interfaces/DriverConfiguration.hpp"
#include "core/motion_actuation/interfaces/MotionActuation.hpp"
#include "core/platform_abstraction/MotorDriver.hpp"

namespace motion
{
    class MotionActuationImpl final
        : public MotionActuation
    {
    public:
        struct Config
        {
            Config();

            hal::Hertz switchingFrequency{ 25000 };
            Decay decay{ Decay::slow };
        };

        MotionActuationImpl(platform::MotorDriver& motors, DriverConfiguration& driverConfiguration, const Config& config = Config());
        MotionActuationImpl(const MotionActuationImpl& other) = delete;
        MotionActuationImpl& operator=(const MotionActuationImpl& other) = delete;
        ~MotionActuationImpl();

        void Apply(float effortLeft, float effortRight) override;
        void Disable(DisableState state) override;
        FaultCause Fault() const override;
        void ClearFault() override;
        DriverState State() const override;

    private:
        bool DrivePermitted() const;
        void ReleaseBridges() const;
        void OnFault();

        platform::MotorDriver& motors;
        Decay decay;
        FaultCause fault{ FaultCause::none };
        DriverState state{ DriverState::configuring };
    };
}
