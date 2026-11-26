#pragma once

#include "core/persistence/interfaces/TuningStore.hpp"
#include "services/util/NvmRecordStore.hpp"

namespace persistence
{
    RestoreResult RestoreOf(services::NvmRecordStore::RestoreResult result, bool undecodable);
    SaveResult SaveOf(services::NvmRecordStore::SaveResult result);
}
