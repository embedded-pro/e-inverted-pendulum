#include "core/attitude_estimation/implementations/ComplementaryPitchFilter.hpp"
#include <algorithm>

namespace estimation
{
    ComplementaryPitchFilter::Config::Config() = default;

    ComplementaryPitchFilter::ComplementaryPitchFilter(const Config& config)
        : crossoverSeconds(config.crossoverSeconds)
        , filter(filters::ComplementaryFilter<float>::AlphaFromTau(config.crossoverSeconds, config.nominalIntervalSeconds), config.nominalIntervalSeconds)
    {}

    void ComplementaryPitchFilter::Restart(float pitch)
    {
        filter.Reset(pitch);
    }

    float ComplementaryPitchFilter::Update(float pitchRate, float accelerometerPitch, float accelerometerTrust, float intervalSeconds)
    {
        const auto gyroscopeWeight = filters::ComplementaryFilter<float>::AlphaFromTau(crossoverSeconds, intervalSeconds);
        const auto trust = std::clamp(accelerometerTrust, 0.0f, 1.0f);

        filter.SetAlpha(1.0f - (1.0f - gyroscopeWeight) * trust);
        return filter.Update(pitchRate, accelerometerPitch, intervalSeconds);
    }
}
