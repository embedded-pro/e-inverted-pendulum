#pragma once

#include "core/persistence/interfaces/TuningStore.hpp"

namespace application
{
    const char* NameOf(persistence::RestoreResult result);
    const char* NameOf(persistence::SaveResult result);
}
