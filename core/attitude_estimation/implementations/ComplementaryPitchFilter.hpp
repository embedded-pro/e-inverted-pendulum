#pragma once

#include "core/attitude_estimation/implementations/PitchFilter.hpp"
#include "numerical/filters/active/ComplementaryFilter.hpp"

namespace estimation
{
    class ComplementaryPitchFilter final
        : public PitchFilter
    {
    public:
        struct Config
        {
            Config();

            float crossoverSeconds{ 0.5f };
            float nominalIntervalSeconds{ 0.001f };
        };

        explicit ComplementaryPitchFilter(const Config& config = Config());

        void Restart(float pitch) override;
        float Update(float pitchRate, float accelerometerPitch, float accelerometerTrust, float intervalSeconds) override;

    private:
        float crossoverSeconds;
        filters::ComplementaryFilter<float> filter;
    };
}
