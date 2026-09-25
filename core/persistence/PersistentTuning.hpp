#pragma once

#include "core/persistence/PersistingBalanceControl.hpp"
#include "generated/echo/Tuning.pb.hpp"
#include "hal/interfaces/Flash.hpp"
#include "services/crypto/Sha256.hpp"
#include "services/util/ConfigurationStore.hpp"

namespace persistence
{
    class PersistentTuning
    {
    public:
        PersistentTuning(hal::Flash& first, hal::Flash& second, services::Sha256& sha256, balance::BalanceControl& control, const safety::SafetySupervisor& supervisor, const PersistingBalanceControl::Config& config = PersistingBalanceControl::Config());

        balance::BalanceControl& Control();

    private:
        services::ConfigurationStoreImpl<Tuning>::WithBlobs<> store;
        PersistingBalanceControl persisting;
    };
}
