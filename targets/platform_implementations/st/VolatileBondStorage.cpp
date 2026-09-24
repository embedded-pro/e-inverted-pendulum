#include "targets/platform_implementations/st/VolatileBondStorage.hpp"
#include <algorithm>

namespace application
{
    void VolatileBondStorage::BondStorageSynchronizerCreated(services::BondStorageSynchronizer&)
    {}

    void VolatileBondStorage::UpdateBondedDevice(hal::MacAddress address)
    {
        if (IsBondStored(address))
            return;

        if (addresses.full())
            addresses.erase(addresses.begin());

        addresses.push_back(address);
    }

    void VolatileBondStorage::RemoveBond(hal::MacAddress address)
    {
        addresses.erase(std::remove(addresses.begin(), addresses.end(), address), addresses.end());
    }

    void VolatileBondStorage::RemoveAllBonds()
    {
        addresses.clear();
    }

    void VolatileBondStorage::RemoveBondIf(const infra::Function<bool(hal::MacAddress)>& onAddress)
    {
        addresses.erase(std::remove_if(addresses.begin(), addresses.end(), [&onAddress](const auto& address)
                            {
                                return onAddress(address);
                            }),
            addresses.end());
    }

    uint32_t VolatileBondStorage::GetMaxNumberOfBonds() const
    {
        return addresses.max_size();
    }

    bool VolatileBondStorage::IsBondStored(hal::MacAddress address) const
    {
        return std::find(addresses.begin(), addresses.end(), address) != addresses.end();
    }

    void VolatileBondStorage::IterateBondedDevices(const infra::Function<void(hal::MacAddress)>& onAddress)
    {
        for (const auto& address : addresses)
            onAddress(address);
    }
}
