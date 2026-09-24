#pragma once

#include "core/inertial_sensing/interfaces/InertialSensing.hpp"
#include <cstdint>

namespace estimation
{
    enum class Filter : uint8_t
    {
        complementary,
        kalman
    };

    enum class InvalidCause : uint8_t
    {
        none,
        sensing,
        converging
    };

    struct Estimate
    {
        float pitch{ 0.0f };
        float pitchRate{ 0.0f };
        bool valid{ false };
        InvalidCause cause{ InvalidCause::sensing };
    };

    class AttitudeEstimation
    {
    public:
        AttitudeEstimation() = default;
        AttitudeEstimation(const AttitudeEstimation& other) = delete;
        AttitudeEstimation& operator=(const AttitudeEstimation& other) = delete;

        virtual Estimate Update(const sensing::Measurement& measurement) = 0;
        virtual Estimate Latest() const = 0;

        virtual void Select(Filter filter) = 0;
        virtual Filter Selected() const = 0;

    protected:
        ~AttitudeEstimation() = default;
    };
}
