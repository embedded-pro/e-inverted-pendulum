#include "core/motion_actuation/implementations/Drv8711DriverConfiguration.hpp"
#include <array>

namespace motion
{
    namespace
    {
        using Drv8711 = drivers::StepperMotorControllerDrv8711;

        constexpr std::array<Drv8711::Isgain, 4> senseGainsHighestFirst{ Drv8711::Isgain::gain40, Drv8711::Isgain::gain20, Drv8711::Isgain::gain10, Drv8711::Isgain::gain5 };
        constexpr std::array<uint64_t, 4> senseGainValuesHighestFirst{ 40, 20, 10, 5 };
        constexpr uint64_t referenceMicroVolt{ 2'750'000 };
        constexpr uint64_t torqueSteps{ 256 };
        constexpr uint64_t maximumTorque{ 255 };

        uint64_t TorqueFor(uint64_t senseGain, uint32_t senseResistanceMilliOhm, uint32_t tripCurrentMilliAmpere)
        {
            return torqueSteps * senseGain * senseResistanceMilliOhm * tripCurrentMilliAmpere / referenceMicroVolt;
        }
    }

    CurrentScaling ScaleCurrent(uint32_t senseResistanceMilliOhm, uint32_t tripCurrentMilliAmpere)
    {
        for (std::size_t i = 0; i != senseGainsHighestFirst.size(); ++i)
        {
            const auto torque = TorqueFor(senseGainValuesHighestFirst[i], senseResistanceMilliOhm, tripCurrentMilliAmpere);

            if (torque <= maximumTorque)
                return { senseGainsHighestFirst[i], static_cast<uint8_t>(torque) };
        }

        return { Drv8711::Isgain::gain5, static_cast<uint8_t>(maximumTorque) };
    }

    Drv8711DriverConfiguration::Config::Config() = default;

    Drv8711DriverConfiguration::Drv8711DriverConfiguration(drivers::DirectPwmStepperMotorDrv8711Decorator& driver, const Config& config)
        : driver(driver)
        , configuration(DriverConfigurationFor(config))
        , wakeUpDelay(config.wakeUpDelay)
    {}

    Drv8711::Configuration Drv8711DriverConfiguration::DriverConfigurationFor(const Config& config)
    {
        const auto scaling = ScaleCurrent(config.senseResistanceMilliOhm, config.tripCurrentMilliAmpere);

        Drv8711::Configuration configuration;
        configuration.dtime = config.deadTime;
        configuration.isgain = scaling.senseGain;
        configuration.torque = scaling.torque;
        configuration.decayMode = config.currentDecay;
        configuration.pwmMode = true;
        configuration.enable = true;

        return configuration;
    }

    void Drv8711DriverConfiguration::Configure(const infra::Function<void(bool verified)>& onDone)
    {
        this->onDone = onDone;

        wakeUpTimer.Start(wakeUpDelay, [this]()
            {
                driver.Configure(configuration, [this](bool verified)
                    {
                        this->onDone(verified);
                    });
            });
    }
}
