#include "core/persistence/implementations/StoreStatus.hpp"

namespace persistence
{
    RestoreResult RestoreOf(services::NvmRecordStore::RestoreResult result, bool undecodable)
    {
        if (undecodable)
            return RestoreResult::corrupt;

        switch (result)
        {
            case services::NvmRecordStore::RestoreResult::restored:
                return RestoreResult::restored;
            case services::NvmRecordStore::RestoreResult::defaults:
                return RestoreResult::defaults;
            case services::NvmRecordStore::RestoreResult::corrupt:
                return RestoreResult::corrupt;
            default:
                return RestoreResult::pending;
        }
    }

    SaveResult SaveOf(services::NvmRecordStore::SaveResult result)
    {
        switch (result)
        {
            case services::NvmRecordStore::SaveResult::pending:
                return SaveResult::pending;
            case services::NvmRecordStore::SaveResult::saved:
                return SaveResult::saved;
            case services::NvmRecordStore::SaveResult::failed:
                return SaveResult::failed;
            default:
                return SaveResult::none;
        }
    }
}
