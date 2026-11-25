#pragma once

#include <cstdint>

namespace persistence
{
    enum class RestoreResult : uint8_t
    {
        pending,
        restored,
        defaults,
        corrupt
    };

    enum class SaveResult : uint8_t
    {
        none,
        pending,
        saved,
        failed
    };

    class TuningStore
    {
    public:
        TuningStore() = default;
        TuningStore(const TuningStore& other) = delete;
        TuningStore& operator=(const TuningStore& other) = delete;

        virtual RestoreResult Restore() const = 0;
        virtual SaveResult LastSave() const = 0;

    protected:
        ~TuningStore() = default;
    };
}
