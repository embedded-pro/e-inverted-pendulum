#include "core/motion_actuation/implementations/Drv8711Configuration.hpp"

namespace motion
{
    namespace
    {
        constexpr std::array<drv8711::Register, 7> configurationOrder{
            drv8711::Register::control,
            drv8711::Register::torque,
            drv8711::Register::offTime,
            drv8711::Register::blankTime,
            drv8711::Register::decay,
            drv8711::Register::stall,
            drv8711::Register::drive,
        };

        constexpr uint16_t sampleThresholdDefault{ 1 << 8 };

        uint16_t Field(uint32_t value, uint8_t position)
        {
            return static_cast<uint16_t>(value << position);
        }
    }

    Drv8711Configuration::Config::Config() = default;

    Drv8711Configuration::Drv8711Configuration(hal::SpiMaster& channel, const Config& config)
        : channel(channel)
        , wakeUpDelay(config.wakeUpDelay)
        , transfers(Sequence(config))
    {}

    std::array<uint16_t, Drv8711Configuration::configuredRegisters> Drv8711Configuration::RegisterValues(const Config& config)
    {
        const auto scaling = drv8711::ScaleCurrent(config.senseResistanceMilliOhm, config.tripCurrentMilliAmpere);
        const auto& gate = config.gateDrive;

        return { {
            static_cast<uint16_t>(Field(static_cast<uint8_t>(config.deadTime), 10) | Field(scaling.senseGainSelection, 8)),
            static_cast<uint16_t>(sampleThresholdDefault | scaling.torque),
            static_cast<uint16_t>(drv8711::offTimeDirectPwm | config.fixedOffTimeSteps),
            config.blankingTimeSteps,
            static_cast<uint16_t>(Field(static_cast<uint8_t>(config.currentDecay), 8) | config.mixedDecayTimeSteps),
            drv8711::stallDetectionUnused,
            static_cast<uint16_t>(Field(gate.sourceCurrentSelection, 10) | Field(gate.sinkCurrentSelection, 8) | Field(gate.sourceTimeSelection, 6) | Field(gate.sinkTimeSelection, 4) | Field(gate.overcurrentDeglitchSelection, 2) | Field(gate.overcurrentThresholdSelection, 0)),
        } };
    }

    std::array<Drv8711Configuration::Transfer, Drv8711Configuration::firstEnablingTransfer + 2> Drv8711Configuration::Sequence(const Config& config)
    {
        const auto values = RegisterValues(config);
        std::array<Transfer, firstEnablingTransfer + 2> sequence{};

        for (std::size_t i = 0; i != configuredRegisters; ++i)
        {
            sequence[i] = { Access::write, configurationOrder[i], values[i] };
            sequence[configuredRegisters + i] = { Access::readBack, configurationOrder[i], values[i] };
        }

        sequence[firstEnablingTransfer] = { Access::write, drv8711::Register::status, 0 };
        sequence[firstEnablingTransfer + 1] = { Access::write, drv8711::Register::control, static_cast<uint16_t>(values[0] | drv8711::controlEnable) };

        return sequence;
    }

    void Drv8711Configuration::Configure(const infra::Function<void(bool verified)>& onDone)
    {
        this->onDone = onDone;
        current = 0;
        readBackMatches = true;

        wakeUpTimer.Start(wakeUpDelay, [this]()
            {
                StartTransfer();
            });
    }

    void Drv8711Configuration::StartTransfer()
    {
        const auto& transfer = transfers[current];
        const auto frame = transfer.access == Access::write ? drv8711::WriteFrame(transfer.reg, transfer.data) : drv8711::ReadFrame(transfer.reg);

        sent = { static_cast<uint8_t>(frame >> 8), static_cast<uint8_t>(frame & 0xff) };

        if (transfer.access == Access::write)
            channel.SendData(infra::MakeConstByteRange(sent), hal::SpiAction::stop, [this]()
                {
                    OnTransferDone();
                });
        else
            channel.SendAndReceive(infra::MakeConstByteRange(sent), infra::MakeByteRange(received), hal::SpiAction::stop, [this]()
                {
                    OnTransferDone();
                });
    }

    void Drv8711Configuration::OnTransferDone()
    {
        const auto& transfer = transfers[current];

        if (transfer.access == Access::readBack)
        {
            const auto readBack = static_cast<uint16_t>((received[0] << 8) | received[1]);
            readBackMatches = readBackMatches && drv8711::ComparableReadBack(transfer.reg, readBack) == drv8711::ComparableReadBack(transfer.reg, transfer.data);
        }

        ++current;

        if (current == firstEnablingTransfer && !readBackMatches)
            Finish(false);
        else if (current == transfers.size())
            Finish(true);
        else
            StartTransfer();
    }

    void Drv8711Configuration::Finish(bool verified)
    {
        onDone(verified);
    }
}
