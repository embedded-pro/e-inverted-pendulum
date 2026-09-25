#include "core/persistence/PersistentTuning.hpp"

namespace persistence
{
    PersistentTuning::PersistentTuning(hal::Flash& first, hal::Flash& second, services::Sha256& sha256, balance::BalanceControl& control, const safety::SafetySupervisor& supervisor, const PersistingBalanceControl::Config& config)
        : store{ first, second, sha256, [this](bool)
            {
                persisting.Restore();
            } }
        , persisting{ control, supervisor, store.Access(store.Configuration()), config }
    {}

    balance::BalanceControl& PersistentTuning::Control()
    {
        return persisting;
    }
}
