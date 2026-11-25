#include "integration_tests/support/MotorDriverFake.hpp"
#include "core/motion_actuation/implementations/BridgeMapping.hpp"

namespace integration
{
    void MotorDriverFake::SetBaseFrequency(hal::Hertz)
    {}

    void MotorDriverFake::Drive(const platform::BridgeInputs& left, const platform::BridgeInputs& right)
    {
        this->left = left;
        this->right = right;

        if (!Released())
            energised = true;
    }

    drivers::DirectPwmStepperMotorDrv8711Decorator& MotorDriverFake::Controller()
    {
        return controller;
    }

    void MotorDriverFake::EnableFaultNotification(const infra::Function<void()>& onFault)
    {
        this->onFault = onFault;
    }

    void MotorDriverFake::DisableFaultNotification()
    {
        onFault = nullptr;
    }

    void MotorDriverFake::RaiseFault()
    {
        if (onFault)
            onFault();
    }

    bool MotorDriverFake::Released() const
    {
        return left == motion::ReleasedInputs() && right == motion::ReleasedInputs();
    }

    bool MotorDriverFake::Braked() const
    {
        return left == motion::BrakedInputs() && right == motion::BrakedInputs();
    }

    bool MotorDriverFake::AnyBraked() const
    {
        return left == motion::BrakedInputs() || right == motion::BrakedInputs();
    }

    bool MotorDriverFake::EnergisedSinceMark() const
    {
        return energised;
    }

    void MotorDriverFake::Mark()
    {
        energised = false;
    }
}
