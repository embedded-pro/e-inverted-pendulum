#include PLATFORM_IMPL_HEADER
#include "core/attitude_estimation/implementations/AttitudeEstimationImpl.hpp"
#include "core/balance_control/implementations/BalanceControlImpl.hpp"
#include "core/balance_control/implementations/CascadedPidStrategy.hpp"
#include "core/balance_control/implementations/LqrStrategy.hpp"
#include "core/cli/Cli.hpp"
#include "core/control_loop/implementations/ControlLoopImpl.hpp"
#include "core/inertial_sensing/implementations/InertialSensingImpl.hpp"
#include "core/motion_actuation/implementations/Drv8711DriverConfiguration.hpp"
#include "core/motion_actuation/implementations/MotionActuationImpl.hpp"
#include "core/safety_supervisor/implementations/SafetySupervisorImpl.hpp"
#include "core/wheel_odometry/implementations/WheelOdometryImpl.hpp"
#include <array>

int main()
{
    static application::PlatformImpl platform;
    static motion::Drv8711DriverConfiguration driverConfiguration{ platform.Motors().Controller() };
    static motion::MotionActuationImpl motionActuation{ platform.Motors(), driverConfiguration };
    static odometry::WheelOdometryImpl wheelOdometry{ platform.Encoders() };
    static sensing::InertialSensingImpl inertialSensing{ platform.Inertial() };
    static estimation::AttitudeEstimationImpl attitudeEstimation;
    static balance::CascadedPidStrategy cascadedPid;
    static balance::LqrStrategy lqr;
    static std::array<balance::ControlStrategy*, 2> strategies{ &cascadedPid, &lqr };
    static balance::BalanceControlImpl balanceControl{ infra::MakeRange(strategies), motionActuation, wheelOdometry };
    static safety::SafetySupervisorImpl supervisor{ motionActuation, inertialSensing, balanceControl };
    static control::ControlLoopImpl controlLoop{ inertialSensing, attitudeEstimation, supervisor, supervisor };
    static application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor, balanceControl };

    platform.Run();

#if defined(__GNUC__) || defined(__clang__)
    __builtin_unreachable();
#endif
}
