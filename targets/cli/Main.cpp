#include PLATFORM_IMPL_HEADER
#include "core/cli/Cli.hpp"
#include "core/inertial_sensing/implementations/InertialSensingImpl.hpp"
#include "core/motion_actuation/implementations/Drv8711Configuration.hpp"
#include "core/motion_actuation/implementations/MotionActuationImpl.hpp"
#include "core/wheel_odometry/implementations/WheelOdometryImpl.hpp"

int main()
{
    static application::PlatformImpl platform;
    static motion::Drv8711Configuration driverConfiguration{ platform.Motors().ConfigurationChannel() };
    static motion::MotionActuationImpl motionActuation{ platform.Motors(), driverConfiguration };
    static odometry::WheelOdometryImpl wheelOdometry{ platform.Encoders() };
    static sensing::InertialSensingImpl inertialSensing{ platform.Inertial() };
    static application::Cli cli{ platform, motionActuation, wheelOdometry, inertialSensing };

    platform.Run();

#if defined(__GNUC__) || defined(__clang__)
    __builtin_unreachable();
#endif
}
