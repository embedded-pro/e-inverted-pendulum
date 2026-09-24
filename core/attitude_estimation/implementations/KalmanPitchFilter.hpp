#pragma once

#include "core/attitude_estimation/implementations/PitchFilter.hpp"
#include "numerical/filters/active/KalmanFilter.hpp"
#include <optional>

namespace estimation
{
    class KalmanPitchFilter final
        : public PitchFilter
    {
    public:
        struct Config
        {
            Config();

            float pitchNoiseDensity{ 1.0e-3f };
            float biasNoiseDensity{ 3.0e-6f };
            float accelerometerPitchVariance{ 3.0e-3f };
            float initialPitchVariance{ 1.0e-2f };
            float initialBiasVariance{ 1.0e-3f };
            float minimumTrust{ 0.05f };
        };

        explicit KalmanPitchFilter(const Config& config = Config());

        void Restart(float pitch) override;
        float Update(float pitchRate, float accelerometerPitch, float accelerometerTrust, float intervalSeconds) override;

        float EstimatedBias() const;

    private:
        using Filter = filters::KalmanFilter<float, 2, 1, 1>;

        void Predict(float pitchRate, float intervalSeconds);
        void Correct(float accelerometerPitch, float accelerometerTrust);

        Config config;
        std::optional<Filter> filter;
    };
}
