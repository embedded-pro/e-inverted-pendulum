#include "core/attitude_estimation/implementations/KalmanPitchFilter.hpp"

namespace estimation
{
    KalmanPitchFilter::Config::Config() = default;

    KalmanPitchFilter::KalmanPitchFilter(const Config& config)
        : config(config)
    {
        Restart(0.0f);
    }

    void KalmanPitchFilter::Restart(float pitch)
    {
        filter.emplace(Filter::StateVector{ { pitch }, { 0.0f } },
            Filter::StateMatrix{ { config.initialPitchVariance, 0.0f }, { 0.0f, config.initialBiasVariance } });

        filter->SetMeasurementMatrix(Filter::MeasurementMatrix{ { 1.0f, 0.0f } });
    }

    float KalmanPitchFilter::Update(float pitchRate, float accelerometerPitch, float accelerometerTrust, float intervalSeconds)
    {
        Predict(pitchRate, intervalSeconds);

        if (accelerometerTrust >= config.minimumTrust)
            Correct(accelerometerPitch, accelerometerTrust);

        return filter->GetState().at(0, 0);
    }

    float KalmanPitchFilter::EstimatedBias() const
    {
        return filter->GetState().at(1, 0);
    }

    void KalmanPitchFilter::Predict(float pitchRate, float intervalSeconds)
    {
        filter->SetStateTransition(Filter::StateMatrix{ { 1.0f, -intervalSeconds }, { 0.0f, 1.0f } });
        filter->SetControlInputMatrix(Filter::ControlMatrix{ { intervalSeconds }, { 0.0f } });
        filter->SetProcessNoise(Filter::StateMatrix{ { config.pitchNoiseDensity * intervalSeconds, 0.0f }, { 0.0f, config.biasNoiseDensity * intervalSeconds } });
        filter->Predict(Filter::ControlVector{ { pitchRate } });
    }

    void KalmanPitchFilter::Correct(float accelerometerPitch, float accelerometerTrust)
    {
        filter->SetMeasurementNoise(Filter::MeasurementCovariance{ { config.accelerometerPitchVariance / accelerometerTrust } });
        filter->Update(Filter::MeasurementVector{ { accelerometerPitch } });
    }
}
