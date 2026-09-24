#pragma once

namespace estimation
{
    class PitchFilter
    {
    public:
        PitchFilter() = default;
        PitchFilter(const PitchFilter& other) = delete;
        PitchFilter& operator=(const PitchFilter& other) = delete;

        virtual void Restart(float pitch) = 0;
        virtual float Update(float pitchRate, float accelerometerPitch, float accelerometerTrust, float intervalSeconds) = 0;

    protected:
        ~PitchFilter() = default;
    };
}
