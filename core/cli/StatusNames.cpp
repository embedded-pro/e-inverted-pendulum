#include "core/cli/StatusNames.hpp"

namespace application
{
    const char* NameOf(persistence::RestoreResult result)
    {
        switch (result)
        {
            case persistence::RestoreResult::restored:
                return "restored";
            case persistence::RestoreResult::defaults:
                return "defaults";
            case persistence::RestoreResult::corrupt:
                return "corrupt";
            default:
                return "pending";
        }
    }

    const char* NameOf(persistence::SaveResult result)
    {
        switch (result)
        {
            case persistence::SaveResult::pending:
                return "pending";
            case persistence::SaveResult::saved:
                return "saved";
            case persistence::SaveResult::failed:
                return "failed";
            default:
                return "none";
        }
    }
}
