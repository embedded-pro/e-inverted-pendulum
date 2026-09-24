#pragma once

#include "infra/util/BoundedVector.hpp"
#include "services/ble/BondStorageSynchronizer.hpp"

namespace application
{
    class VolatileBondStorage
        : public services::BondStorage
    {
    public:
        static constexpr uint32_t maxNumberOfBonds{ 10 };

        void BondStorageSynchronizerCreated(services::BondStorageSynchronizer& manager) override;
        void UpdateBondedDevice(hal::MacAddress address) override;
        void RemoveBond(hal::MacAddress address) override;
        void RemoveAllBonds() override;
        void RemoveBondIf(const infra::Function<bool(hal::MacAddress)>& onAddress) override;
        uint32_t GetMaxNumberOfBonds() const override;
        bool IsBondStored(hal::MacAddress address) const override;
        void IterateBondedDevices(const infra::Function<void(hal::MacAddress)>& onAddress) override;

    private:
        infra::BoundedVector<hal::MacAddress>::WithMaxSize<maxNumberOfBonds> addresses;
    };
}
