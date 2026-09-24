#pragma once

#include "infra/util/Function.hpp"

namespace motion
{
    class DriverConfiguration
    {
    public:
        DriverConfiguration() = default;
        DriverConfiguration(const DriverConfiguration& other) = delete;
        DriverConfiguration& operator=(const DriverConfiguration& other) = delete;

        virtual void Configure(const infra::Function<void(bool verified)>& onDone) = 0;

    protected:
        ~DriverConfiguration() = default;
    };
}
