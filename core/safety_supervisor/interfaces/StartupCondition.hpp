#pragma once

namespace safety
{
    class StartupCondition
    {
    public:
        StartupCondition() = default;
        StartupCondition(const StartupCondition& other) = delete;
        StartupCondition& operator=(const StartupCondition& other) = delete;

        virtual bool Satisfied() const = 0;

    protected:
        ~StartupCondition() = default;
    };
}
