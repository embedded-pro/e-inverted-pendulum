#include "core/balance_control/implementations/LqrStrategy.hpp"
#include "tools/lqr_design/LqrDesign.hpp"
#include <cstdio>
#include <cstdlib>
#include <map>
#include <optional>
#include <string>

namespace
{
    const std::map<std::string, float tools::RobotParameters::*> scalarOptions{
        { "--body-mass", &tools::RobotParameters::bodyMass },
        { "--wheels-mass", &tools::RobotParameters::wheelsMass },
        { "--height", &tools::RobotParameters::height },
        { "--body-inertia", &tools::RobotParameters::bodyInertia },
        { "--wheel-inertia", &tools::RobotParameters::wheelInertia },
        { "--wheel-radius", &tools::RobotParameters::wheelRadius },
        { "--wheel-torque", &tools::RobotParameters::wheelTorque },
        { "--period", &tools::RobotParameters::period },
    };

    const std::map<std::string, std::size_t> limitOptions{
        { "--limit-pitch", 0 },
        { "--limit-pitch-rate", 1 },
        { "--limit-position", 2 },
        { "--limit-velocity", 3 },
    };

    void PrintUsage()
    {
        std::puts("usage: lqr_design --body-mass <kg> --wheels-mass <kg> --height <m> --body-inertia <kg m2>");
        std::puts("                  --wheel-inertia <kg m2> --wheel-torque <N m> [--wheel-radius <m>] [--period <s>]");
        std::puts("                  [--limit-pitch <rad>] [--limit-pitch-rate <rad/s>] [--limit-position <m>] [--limit-velocity <m/s>]");
        std::puts("see documentation/design/lqr-design-tool.md");
    }

    std::optional<tools::RobotParameters> Parse(int argc, char** argv)
    {
        tools::RobotParameters parameters;
        for (int i = 1; i + 1 < argc; i += 2)
        {
            char* end{ nullptr };
            const float value{ std::strtof(argv[i + 1], &end) };
            if (end == argv[i + 1] || *end != '\0')
                return std::nullopt;

            if (const auto scalar = scalarOptions.find(argv[i]); scalar != scalarOptions.end())
                parameters.*(scalar->second) = value;
            else if (const auto limit = limitOptions.find(argv[i]); limit != limitOptions.end())
                parameters.stateLimits[limit->second] = value;
            else
                return std::nullopt;
        }

        if (argc % 2 == 0 || parameters.bodyMass <= 0.0f || parameters.height <= 0.0f || parameters.wheelTorque <= 0.0f || parameters.wheelRadius <= 0.0f || parameters.period <= 0.0f || parameters.bodyInertia < 0.0f || parameters.wheelInertia < 0.0f || parameters.wheelsMass < 0.0f)
            return std::nullopt;
        return parameters;
    }

    const char* NameOf(tools::DesignFailure failure)
    {
        switch (failure)
        {
            case tools::DesignFailure::discretisation:
                return "the plant does not discretise";
            case tools::DesignFailure::riccati:
                return "the Riccati iteration does not converge";
            case tools::DesignFailure::unstable:
                return "the closed loop does not settle within 5 s; loosen the state limits";
            case tools::DesignFailure::outOfRange:
                return "a gain is outside its firmware range";
            default:
                return "none";
        }
    }

    void PrintDesign(const tools::LqrDesign& design)
    {
        std::puts("discrete plant, state (pitch, pitch rate, travel, speed), input common wheel torque:");
        for (std::size_t row = 0; row != 4; ++row)
            std::printf("  A %12.6f %12.6f %12.6f %12.6f   B %12.6f\n", design.a.at(row, 0), design.a.at(row, 1), design.a.at(row, 2), design.a.at(row, 3), design.b.at(row, 0));
        std::printf("model gains (N m per state unit): %.6f %.6f %.6f %.6f\n", design.modelGains[0], design.modelGains[1], design.modelGains[2], design.modelGains[3]);
        std::printf("closed loop after 5 s: %.4f of the start\n", design.decay);
    }
}

int main(int argc, char** argv)
{
    const auto parameters = Parse(argc, argv);
    if (!parameters.has_value())
    {
        PrintUsage();
        return EXIT_FAILURE;
    }

    const auto design = tools::DesignLqr(*parameters);
    PrintDesign(design);

    const balance::LqrStrategy strategy;
    const auto descriptors = strategy.Parameters();
    std::puts("firmware commands:");
    std::puts("  strategy lqr");
    for (std::size_t i = 0; i != design.firmwareGains.size(); ++i)
        std::printf("  param %zu %.6f    # %s, range %g to %g\n", i, design.firmwareGains[i], descriptors[i].name, descriptors[i].minimum, descriptors[i].maximum);

    if (design.failure != tools::DesignFailure::none)
    {
        std::printf("failed: %s\n", NameOf(design.failure));
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
