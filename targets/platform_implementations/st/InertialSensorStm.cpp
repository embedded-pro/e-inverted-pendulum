#include "targets/platform_implementations/st/InertialSensorStm.hpp"
#include "infra/util/ReallyAssert.hpp"

namespace application
{
    drivers::Mpu9250Core::Config InertialSensorStm::DeviceConfig()
    {
        drivers::Mpu9250Core::Config config;

        config.gyroscopeFullScale = drivers::Mpu9250Core::GyroscopeFullScale::dps500;
        config.accelerometerFullScale = drivers::Mpu9250Core::AccelerometerFullScale::g4;
        config.gyroscopeLowPassFilter = drivers::Mpu9250Core::GyroscopeLowPassFilter::bandwidth41Hz;
        config.accelerometerLowPassFilter = drivers::Mpu9250Core::AccelerometerLowPassFilter::bandwidth45Hz;
        config.sampleRateDivider = 0;
        config.interruptPolarity = drivers::Mpu9250Core::InterruptPolarity::activeHigh;
        config.interruptDrive = drivers::Mpu9250Core::InterruptDrive::pushPull;
        config.interruptLatch = drivers::Mpu9250Core::InterruptLatch::pulsed;

        return config;
    }

    hal::SpiMasterStm::Config InertialSensorStm::BusConfig()
    {
        hal::SpiMasterStm::Config config;

        config.baudRatePrescaler = SPI_BAUDRATEPRESCALER_64;

        return config;
    }

    InertialSensorStm::InertialSensorStm(const platform::AxisMap& axisMap)
        : axisMap(axisMap)
    {}

    void InertialSensorStm::OnAcceleration(drivers::Mpu9250Core::Accelerometer::Samples samples)
    {
        really_assert(samples.size() == 3);

        pending.acceleration = platform::ToBodyFrame(axisMap,
            platform::MilliMeterPerSecondSquaredToMeterPerSecondSquared(samples[0].Value()),
            platform::MilliMeterPerSecondSquaredToMeterPerSecondSquared(samples[1].Value()),
            platform::MilliMeterPerSecondSquaredToMeterPerSecondSquared(samples[2].Value()));

        pending.sampledAt = infra::Now();
        accelerationReceived = true;
    }

    void InertialSensorStm::OnAngularVelocity(drivers::Mpu9250Core::Gyroscope::Samples samples)
    {
        really_assert(samples.size() == 3);

        if (!accelerationReceived)
            return;

        accelerationReceived = false;

        pending.angularRate = platform::ToBodyFrame(axisMap,
            platform::MilliDegreePerSecondToRadianPerSecond(samples[0].Value()),
            platform::MilliDegreePerSecondToRadianPerSecond(samples[1].Value()),
            platform::MilliDegreePerSecondToRadianPerSecond(samples[2].Value()));

        pending.valid = true;

        if (onSample)
            onSample(pending);
    }

    void InertialSensorStm::Start(const infra::Function<void(const platform::InertialSample&)>& onSample)
    {
        this->onSample = onSample;
        accelerationReceived = false;

        if (stopping)
            return;

        if (identified)
            StartSampling();
        else if (!initializing)
        {
            initializing = true;

            device.Initialize(DeviceConfig(), [this](drivers::Mpu9250Core::InitializationResult result)
                {
                    initializing = false;
                    identified = result == drivers::Mpu9250Core::InitializationResult::success;

                    if (identified && this->onSample)
                        StartSampling();
                });
        }
    }

    void InertialSensorStm::StartSampling()
    {
        sampling = true;

        device.AsAccelerometer().Start([this](drivers::Mpu9250Core::Accelerometer::Samples samples)
            {
                OnAcceleration(samples);
            });

        device.AsGyroscope().Start([this](drivers::Mpu9250Core::Gyroscope::Samples samples)
            {
                OnAngularVelocity(samples);
            });
    }

    bool InertialSensorStm::Identified() const
    {
        return identified;
    }

    void InertialSensorStm::Stop()
    {
        onSample = nullptr;
        accelerationReceived = false;

        if (stopping || (!sampling && !initializing))
            return;

        sampling = false;
        stopping = true;

        device.Stop([this]()
            {
                stopping = false;
                initializing = false;
                identified = false;

                if (this->onSample)
                    Start(this->onSample);
            });
    }
}
