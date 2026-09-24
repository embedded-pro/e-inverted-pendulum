#pragma once

#include <cstdint>

namespace safety
{
    enum class Mode : uint8_t
    {
        init,
        calibrating,
        idle,
        armed,
        fault
    };

    enum class FaultCause : uint8_t
    {
        none,
        fall,
        driverFault,
        estimateInvalid,
        loopStalled,
        selfTestFailed,
        calibrationFailed
    };

    class SafetySupervisor
    {
    public:
        SafetySupervisor() = default;
        SafetySupervisor(const SafetySupervisor& other) = delete;
        SafetySupervisor& operator=(const SafetySupervisor& other) = delete;

        virtual bool Arm() = 0;
        virtual bool Disarm() = 0;
        virtual bool ClearFault() = 0;
        virtual bool Calibrate() = 0;

        virtual Mode Current() const = 0;
        virtual FaultCause LatchedCause() const = 0;
        virtual bool DrivePermitted() const = 0;

    protected:
        ~SafetySupervisor() = default;
    };
}
