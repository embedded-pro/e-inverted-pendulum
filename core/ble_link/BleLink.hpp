#pragma once

#include "core/ble_link/RobotControlService.hpp"
#include "core/ble_link/interfaces/LinkStatus.hpp"
#include "core/platform_abstraction/Bluetooth.hpp"
#include "infra/util/BoundedString.hpp"
#include "infra/util/BoundedVector.hpp"
#include "services/ble/GapPeripheral.hpp"
#include "services/ble/GattClient.hpp"

namespace ble
{
    class BleLink
        : private services::GapPeripheralObserver
        , private services::GattClientObserver
    {
    public:
        static constexpr std::size_t maximumAdvertisementSize{ 31 };

        BleLink(platform::Bluetooth& bluetooth, infra::BoundedConstString deviceName, safety::SafetySupervisor& supervisor, balance::BalanceControl& balanceControl, const telemetry::TelemetrySource& telemetry);
        ~BleLink();

        bool Connected() const;
        LinkReport Report() const;

    private:
        class MtuTracker
            : public services::GattClientConnectionObserver
        {
        public:
            explicit MtuTracker(RobotControlService& service);

            void ServiceDiscovered(const services::GattService& service) override;
            void IncludedServiceDiscovered(const services::GattIncludedService& includedService) override;
            void CharacteristicDiscovered(const services::GattCharacteristic& characteristic) override;
            void DescriptorDiscovered(const services::GattDescriptor& descriptor) override;
            void MtuChanged(uint16_t mtu) override;

        private:
            RobotControlService& service;
        };

        void StateChanged(services::GapPeripheralState newState) override;
        void ConnectionEstablished(infra::SharedPtr<services::GattClientConnection> connection) override;
        void ConnectionReleased(services::GattClientConnection& connection) override;
        void Advertise();

        RobotControlService service;
        MtuTracker mtuTracker{ service };
        infra::SharedPtr<services::GattClientConnection> connection;
        infra::BoundedVector<uint8_t>::WithMaxSize<maximumAdvertisementSize> advertisement;
        infra::BoundedVector<uint8_t>::WithMaxSize<maximumAdvertisementSize> scanResponse;
        services::GapPeripheralState state{ services::GapPeripheralState::standby };
    };
}
