#pragma once

#include "numerical/math/Matrix.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace tools
{
    struct RobotParameters
    {
        float bodyMass{ 0.0f };
        float wheelsMass{ 0.0f };
        float height{ 0.0f };
        float bodyInertia{ 0.0f };
        float wheelInertia{ 0.0f };
        float wheelRadius{ 0.034f };
        float wheelTorque{ 0.0f };
        float period{ 0.002f };
        std::array<float, 4> stateLimits{ 0.1f, 1.0f, 0.5f, 1.0f };
    };

    enum class DesignFailure : uint8_t
    {
        none,
        discretisation,
        riccati,
        unstable,
        outOfRange
    };

    struct LqrDesign
    {
        math::SquareMatrix<float, 4> a{};
        math::Matrix<float, 4, 1> b{};
        std::array<float, 4> modelGains{};
        std::array<float, 4> firmwareGains{};
        float decay{ 0.0f };
        DesignFailure failure{ DesignFailure::none };
    };

    LqrDesign DesignLqr(const RobotParameters& parameters);
    bool FitsFirmwareRanges(const std::array<float, 4>& firmwareGains);
    std::array<float, 4> ToFirmwareGains(const std::array<float, 4>& modelGains, float wheelTorque);
    float ClosedLoopDecay(const math::SquareMatrix<float, 4>& a, const math::Matrix<float, 4, 1>& b, const std::array<float, 4>& gains, std::size_t steps);
}
