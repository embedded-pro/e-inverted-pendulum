#pragma once

#include <array>
#include <cstdint>

namespace motion::drv8711
{
    enum class Register : uint8_t
    {
        control,
        torque,
        offTime,
        blankTime,
        decay,
        stall,
        drive,
        status
    };

    constexpr uint16_t dataMask{ 0x0fff };
    constexpr uint16_t readRequest{ 0x8000 };
    constexpr uint16_t writeOnlyTorqueBits{ 1 << 10 };
    constexpr uint16_t controlEnable{ 1 << 0 };
    constexpr uint16_t offTimeDirectPwm{ 1 << 8 };
    constexpr uint16_t stallDetectionUnused{ 0x040 };

    constexpr uint16_t WriteFrame(Register reg, uint16_t data)
    {
        return static_cast<uint16_t>((static_cast<uint16_t>(reg) << 12) | (data & dataMask));
    }

    constexpr uint16_t ReadFrame(Register reg)
    {
        return static_cast<uint16_t>(readRequest | (static_cast<uint16_t>(reg) << 12));
    }

    constexpr uint16_t ComparableReadBack(Register reg, uint16_t data)
    {
        if (reg == Register::torque)
            return data & dataMask & static_cast<uint16_t>(~writeOnlyTorqueBits);

        return data & dataMask;
    }

    struct CurrentScaling
    {
        uint8_t senseGainSelection;
        uint8_t torque;

        constexpr bool operator==(const CurrentScaling& other) const = default;
    };

    constexpr std::array<uint32_t, 4> senseGains{ 5, 10, 20, 40 };
    constexpr uint64_t referenceMilliVolt{ 2750 };
    constexpr uint64_t torqueSteps{ 256 };
    constexpr uint64_t maximumTorque{ 255 };

    constexpr uint64_t TorqueFor(uint32_t senseGain, uint32_t senseResistanceMilliOhm, uint32_t tripCurrentMilliAmpere)
    {
        return torqueSteps * senseGain * senseResistanceMilliOhm * tripCurrentMilliAmpere / (referenceMilliVolt * 1000);
    }

    constexpr CurrentScaling ScaleCurrent(uint32_t senseResistanceMilliOhm, uint32_t tripCurrentMilliAmpere)
    {
        for (uint8_t selection = senseGains.size(); selection-- > 0;)
        {
            const auto torque = TorqueFor(senseGains[selection], senseResistanceMilliOhm, tripCurrentMilliAmpere);

            if (torque <= maximumTorque)
                return { selection, static_cast<uint8_t>(torque) };
        }

        return { 0, static_cast<uint8_t>(maximumTorque) };
    }
}
