#include "tools/lqr_design/LqrDesign.hpp"
#include "core/balance_control/implementations/LqrStrategy.hpp"
#include "numerical/control_analysis/ContinuousToDiscrete.hpp"
#include "numerical/controllers/implementations/Lqr.hpp"
#include "robotics/mobile/WheeledInvertedPendulum.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <limits>
#include <map>

namespace tools
{
    namespace
    {
        constexpr float initialPitch{ 0.05f };
        constexpr float settleSeconds{ 5.0f };
        constexpr float requiredDecay{ 0.01f };
        constexpr float minimumPeriod{ 1e-4f };
        constexpr float maximumPeriod{ 0.1f };
        constexpr float unusedTrackWidth{ 0.1f };

        const std::map<std::string, float RobotParameters::*, std::less<>> scalarOptions{
            { "--body-mass", &RobotParameters::bodyMass },
            { "--wheels-mass", &RobotParameters::wheelsMass },
            { "--height", &RobotParameters::height },
            { "--body-inertia", &RobotParameters::bodyInertia },
            { "--wheel-inertia", &RobotParameters::wheelInertia },
            { "--wheel-radius", &RobotParameters::wheelRadius },
            { "--wheel-torque", &RobotParameters::wheelTorque },
            { "--period", &RobotParameters::period },
        };

        const std::map<std::string, std::size_t, std::less<>> limitOptions{
            { "--limit-pitch", 0 },
            { "--limit-pitch-rate", 1 },
            { "--limit-position", 2 },
            { "--limit-velocity", 3 },
        };

        std::optional<float> ParseFinite(const std::string& text)
        {
            char* end{ nullptr };
            const float value{ std::strtof(text.c_str(), &end) };
            if (end == text.c_str() || *end != '\0' || !std::isfinite(value))
                return std::nullopt;
            return value;
        }

        bool Store(RobotParameters& parameters, const std::string& name, float value)
        {
            if (const auto scalar = scalarOptions.find(name); scalar != scalarOptions.end())
                parameters.*(scalar->second) = value;
            else if (const auto limit = limitOptions.find(name); limit != limitOptions.end())
                parameters.stateLimits[limit->second] = value;
            else
                return false;
            return true;
        }

        bool Valid(const RobotParameters& p)
        {
            const bool positive = p.bodyMass > 0.0f && p.height > 0.0f && p.wheelTorque > 0.0f && p.wheelRadius > 0.0f;
            const bool nonNegative = p.wheelsMass >= 0.0f && p.bodyInertia >= 0.0f && p.wheelInertia >= 0.0f;
            const bool period = p.period >= minimumPeriod && p.period <= maximumPeriod;
            return positive && nonNegative && period && std::ranges::all_of(p.stateLimits, [](float limit)
                                                            {
                                                                return limit > 0.0f;
                                                            });
        }

        mobile::WheeledInvertedPendulumParameters<float> ModelParameters(const RobotParameters& parameters)
        {
            mobile::WheeledInvertedPendulumParameters<float> model;
            model.bodyMass = parameters.bodyMass;
            model.wheelsMass = parameters.wheelsMass;
            model.centreOfMassHeight = parameters.height;
            model.bodyPitchInertia = parameters.bodyInertia;
            model.wheelInertia = parameters.wheelInertia;
            model.wheelRadius = parameters.wheelRadius;
            model.trackWidth = unusedTrackWidth;
            return model;
        }

        math::SquareMatrix<float, 4> StateWeights(const RobotParameters& parameters)
        {
            math::SquareMatrix<float, 4> q{};
            for (std::size_t i = 0; i != 4; ++i)
                q.at(i, i) = 1.0f / (parameters.stateLimits[i] * parameters.stateLimits[i]);
            return q;
        }
    }

    bool FitsFirmwareRanges(const std::array<float, 4>& gains)
    {
        const balance::LqrStrategy strategy;
        const auto descriptors = strategy.Parameters();
        for (std::size_t i = 0; i != gains.size(); ++i)
            if (!(gains[i] >= descriptors[i].minimum && gains[i] <= descriptors[i].maximum))
                return false;
        return true;
    }

    std::optional<RobotParameters> ParseArguments(const std::vector<std::string>& arguments)
    {
        if (arguments.size() % 2 != 0)
            return std::nullopt;

        RobotParameters parameters;
        for (std::size_t i = 0; i != arguments.size(); i += 2)
        {
            const auto value = ParseFinite(arguments[i + 1]);
            if (!value.has_value() || !Store(parameters, arguments[i], *value))
                return std::nullopt;
        }

        if (!Valid(parameters))
            return std::nullopt;
        return parameters;
    }

    std::array<float, 4> ToFirmwareGains(const std::array<float, 4>& modelGains, float wheelTorque)
    {
        const float scale{ 1.0f / (2.0f * wheelTorque) };
        return { -modelGains[0] * scale, -modelGains[1] * scale, modelGains[2] * scale, modelGains[3] * scale };
    }

    float ClosedLoopDecay(const math::SquareMatrix<float, 4>& a, const math::Matrix<float, 4, 1>& b, const std::array<float, 4>& gains, std::size_t steps)
    {
        math::Vector<float, 4> state{ { initialPitch }, { 0.0f }, { 0.0f }, { 0.0f } };
        for (std::size_t step = 0; step != steps; ++step)
        {
            const float input{ -(gains[0] * state[0] + gains[1] * state[1] + gains[2] * state[2] + gains[3] * state[3]) };
            state = a * state + b * math::Vector<float, 1>{ { input } };
        }

        float largest{ 0.0f };
        for (std::size_t i = 0; i != 4; ++i)
        {
            if (!std::isfinite(state[i]))
                return std::numeric_limits<float>::infinity();
            largest = std::max(largest, std::fabs(state[i]));
        }
        return largest / initialPitch;
    }

    LqrDesign DesignLqr(const RobotParameters& parameters)
    {
        LqrDesign design;
        const mobile::WheeledInvertedPendulum model{ ModelParameters(parameters) };
        const auto discrete = control_analysis::ContinuousToDiscrete<float, 4, 1, 4>{}.Convert(model.Pitch(), parameters.period, control_analysis::DiscretizationMethod::ZeroOrderHold);
        if (!discrete.has_value())
        {
            design.failure = DesignFailure::discretisation;
            return design;
        }

        design.a = discrete->A;
        design.b = discrete->B;
        const float maximumTorque{ 2.0f * parameters.wheelTorque };
        const auto regulator = controllers::Lqr<float, 4, 1, 20000>::TryCreate(design.a, design.b, StateWeights(parameters), math::SquareMatrix<float, 1>{ { 1.0f / (maximumTorque * maximumTorque) } });
        if (!regulator.has_value())
        {
            design.failure = DesignFailure::riccati;
            return design;
        }

        for (std::size_t i = 0; i != 4; ++i)
            design.modelGains[i] = regulator->GetGain().at(0, i);
        design.firmwareGains = ToFirmwareGains(design.modelGains, parameters.wheelTorque);
        design.decay = ClosedLoopDecay(design.a, design.b, design.modelGains, static_cast<std::size_t>(settleSeconds / parameters.period));

        if (!(design.decay < requiredDecay))
            design.failure = DesignFailure::notSettled;
        else if (!FitsFirmwareRanges(design.firmwareGains))
            design.failure = DesignFailure::outOfRange;
        return design;
    }
}
