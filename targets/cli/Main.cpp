#include PLATFORM_IMPL_HEADER
#include "core/attitude_estimation/implementations/AttitudeEstimationImpl.hpp"
#include "core/cli/Cli.hpp"
#include "core/control_loop/implementations/ControlLoopImpl.hpp"
#include "core/control_loop/implementations/IdleStages.hpp"
#include "core/inertial_sensing/implementations/InertialSensingImpl.hpp"
#include "core/motion_actuation/implementations/Drv8711DriverConfiguration.hpp"
#include "core/motion_actuation/implementations/MotionActuationImpl.hpp"
#include "core/safety_supervisor/implementations/SafetySupervisorImpl.hpp"
#include "core/wheel_odometry/implementations/WheelOdometryImpl.hpp"

int main()
{
    static application::PlatformImpl platform;
    static motion::Drv8711DriverConfiguration driverConfiguration{ platform.Motors().Controller() };
    static motion::MotionActuationImpl motionActuation{ platform.Motors(), driverConfiguration };
    static odometry::WheelOdometryImpl wheelOdometry{ platform.Encoders() };
    static sensing::InertialSensingImpl inertialSensing{ platform.Inertial() };
    static estimation::AttitudeEstimationImpl attitudeEstimation;
    static control::IdleStages idleStages;
    static safety::SafetySupervisorImpl supervisor{ motionActuation, inertialSensing, idleStages };
    static control::ControlLoopImpl controlLoop{ inertialSensing, attitudeEstimation, supervisor, idleStages };
    static application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing, attitudeEstimation, controlLoop, supervisor };

    platform.Run();

#if defined(__GNUC__) || defined(__clang__)
    __builtin_unreachable();
#endif
}
