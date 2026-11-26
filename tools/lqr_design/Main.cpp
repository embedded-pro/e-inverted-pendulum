#include "core/balance_control/implementations/LqrStrategy.hpp"
#include "tools/lqr_design/LqrDesign.hpp"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace
{
    void PrintUsage()
    {
        std::puts("usage: lqr_design --body-mass <kg> --wheels-mass <kg> --height <m> --body-inertia <kg m2>");
        std::puts("                  --wheel-inertia <kg m2> --wheel-torque <N m> [--wheel-radius <m>] [--period <s>]");
        std::puts("                  [--limit-pitch <rad>] [--limit-pitch-rate <rad/s>] [--limit-position <m>] [--limit-velocity <m/s>]");
        std::puts("see documentation/design/lqr-design-tool.md");
    }

    const char* NameOf(tools::DesignFailure failure)
    {
        switch (failure)
        {
            case tools::DesignFailure::discretisation:
                return "the plant does not discretise";
            case tools::DesignFailure::riccati:
                return "the Riccati iteration does not converge";
            case tools::DesignFailure::notSettled:
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

    void PrintCommands(const tools::LqrDesign& design)
    {
        const balance::LqrStrategy strategy;
        const auto descriptors = strategy.Parameters();
        std::puts("firmware commands:");
        std::puts("  strategy lqr");
        for (std::size_t i = 0; i != design.firmwareGains.size(); ++i)
            std::printf("  param %zu %.6f\n", i, design.firmwareGains[i]);
        for (std::size_t i = 0; i != design.firmwareGains.size(); ++i)
            std::printf("  (%s, range %g to %g)\n", descriptors[i].name, descriptors[i].minimum, descriptors[i].maximum);
    }
}

int main(int argc, char** argv)
{
    const std::vector<std::string> arguments(argv + 1, argv + argc);
    if (arguments.size() == 1 && (arguments[0] == "--help" || arguments[0] == "-h"))
    {
        PrintUsage();
        return EXIT_SUCCESS;
    }

    const auto parameters = tools::ParseArguments(arguments);
    if (!parameters.has_value())
    {
        PrintUsage();
        return EXIT_FAILURE;
    }

    const auto design = tools::DesignLqr(*parameters);
    PrintDesign(design);

    if (design.failure != tools::DesignFailure::none)
    {
        std::printf("failed: %s\n", NameOf(design.failure));
        return EXIT_FAILURE;
    }

    PrintCommands(design);
    return EXIT_SUCCESS;
}
