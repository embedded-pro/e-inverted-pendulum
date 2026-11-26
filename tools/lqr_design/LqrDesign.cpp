#include "tools/lqr_design/LqrDesign.hpp"
#include "core/balance_control/implementations/LqrStrategy.hpp"
#include "numerical/control_analysis/ContinuousToDiscrete.hpp"
#include "numerical/controllers/implementations/Lqr.hpp"
#include "robotics/mobile/WheeledInvertedPendulum.hpp"
#include <algorithm>
#include <cmath>

namespace tools
{
    namespace
    {
        constexpr float initialPitch{ 0.05f };
        constexpr float settleSeconds{ 5.0f };
        constexpr float requiredDecay{ 0.01f };

        mobile::WheeledInvertedPendulumParameters<float> ModelParameters(const RobotParameters& parameters)
        {
            mobile::WheeledInvertedPendulumParameters<float> model;
            model.bodyMass = parameters.bodyMass;
            model.wheelsMass = parameters.wheelsMass;
            model.centreOfMassHeight = parameters.height;
            model.bodyPitchInertia = parameters.bodyInertia;
            model.wheelInertia = parameters.wheelInertia;
            model.yawInertia = 0.0f;
            model.wheelRadius = parameters.wheelRadius;
            model.trackWidth = 0.1f;
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
            largest = std::max(largest, std::fabs(state[i]));
        return largest / initialPitch;
    }

    LqrDesign DesignLqr(const RobotParameters& parameters)
    {
        LqrDesign design;
        const mobile::WheeledInvertedPendulum<float> model{ ModelParameters(parameters) };
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
            design.failure = DesignFailure::unstable;
        else if (!FitsFirmwareRanges(design.firmwareGains))
            design.failure = DesignFailure::outOfRange;
        return design;
    }
}
