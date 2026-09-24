#pragma once

#include "core/motion_actuation/implementations/Drv8711Registers.hpp"
#include "core/motion_actuation/interfaces/DriverConfiguration.hpp"
#include "hal/interfaces/Spi.hpp"
#include "infra/timer/Timer.hpp"
#include <array>
#include <cstddef>
#include <cstdint>

namespace motion
{
    class Drv8711Configuration final
        : public DriverConfiguration
    {
    public:
        enum class DeadTime : uint8_t
        {
            nanoseconds400,
            nanoseconds450,
            nanoseconds650,
            nanoseconds850
        };

        enum class CurrentDecay : uint8_t
        {
            slow,
            slowIncreasingMixedDecreasing,
            fast,
            mixed,
            slowIncreasingAutoMixedDecreasing,
            autoMixed
        };

        struct GateDrive
        {
            uint8_t sourceCurrentSelection{ 2 };
            uint8_t sinkCurrentSelection{ 2 };
            uint8_t sourceTimeSelection{ 1 };
            uint8_t sinkTimeSelection{ 1 };
            uint8_t overcurrentDeglitchSelection{ 2 };
            uint8_t overcurrentThresholdSelection{ 1 };
        };

        struct Config
        {
            Config();

            uint32_t senseResistanceMilliOhm{ 50 };
            uint32_t tripCurrentMilliAmpere{ 1000 };
            DeadTime deadTime{ DeadTime::nanoseconds850 };
            uint8_t fixedOffTimeSteps{ 0x30 };
            uint8_t blankingTimeSteps{ 0x80 };
            CurrentDecay currentDecay{ CurrentDecay::slowIncreasingMixedDecreasing };
            uint8_t mixedDecayTimeSteps{ 0x10 };
            GateDrive gateDrive;
            infra::Duration wakeUpDelay{ std::chrono::milliseconds{ 1 } };
        };

        explicit Drv8711Configuration(hal::SpiMaster& channel, const Config& config = Config());

        void Configure(const infra::Function<void(bool verified)>& onDone) override;

    private:
        enum class Access : uint8_t
        {
            write,
            readBack
        };

        struct Transfer
        {
            Access access;
            drv8711::Register reg;
            uint16_t data;
        };

        static constexpr std::size_t configuredRegisters{ 7 };
        static constexpr std::size_t firstEnablingTransfer{ 2 * configuredRegisters };

        static std::array<uint16_t, configuredRegisters> RegisterValues(const Config& config);
        static std::array<Transfer, firstEnablingTransfer + 2> Sequence(const Config& config);

        void StartTransfer();
        void OnTransferDone();
        void Finish(bool verified);

        hal::SpiMaster& channel;
        infra::Duration wakeUpDelay;
        std::array<Transfer, firstEnablingTransfer + 2> transfers;
        infra::TimerSingleShot wakeUpTimer;
        infra::Function<void(bool verified)> onDone;
        std::size_t current{ 0 };
        bool readBackMatches{ true };
        std::array<uint8_t, 2> sent{};
        std::array<uint8_t, 2> received{};
    };
}
