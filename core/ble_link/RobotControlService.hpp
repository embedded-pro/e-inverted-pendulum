#pragma once

#include "core/balance_control/interfaces/BalanceControl.hpp"
#include "core/platform_abstraction/Bluetooth.hpp"
#include "core/safety_supervisor/interfaces/SafetySupervisor.hpp"
#include "core/telemetry/interfaces/Telemetry.hpp"
#include "infra/timer/Timer.hpp"
#include "services/ble/GattServerCharacteristicImpl.hpp"
#include <array>
#include <chrono>

namespace ble
{
    class RobotControlService
        : public platform::BluetoothLinkObserver
    {
    public:
        static constexpr std::size_t motionSize{ 8 };
        static constexpr std::size_t modeSize{ 4 };
        static constexpr std::size_t telemetrySize{ 26 };
        static constexpr std::size_t tuningSize{ 33 };
        static constexpr std::size_t maximumNameLength{ 16 };
        static constexpr services::AttAttribute::Handle clientConfigurationOffset{ 2 };

        enum class Command : uint8_t
        {
            arm = 1,
            disarm,
            clearFault,
            calibrate
        };

        enum class Outcome : uint8_t
        {
            none,
            accepted,
            refused
        };

        enum class Operation : uint8_t
        {
            describeStrategy = 1,
            selectStrategy,
            describeParameter,
            writeParameter
        };

        enum class Status : uint8_t
        {
            accepted,
            refused,
            invalidRequest,
            mtuTooSmall
        };

        struct Config
        {
            Config();

            std::chrono::milliseconds telemetryPeriod{ 40 };
        };

        RobotControlService(services::GattServer& gattServer, safety::SafetySupervisor& supervisor, balance::BalanceControl& balanceControl, const telemetry::TelemetrySource& telemetry, const Config& config = Config());

        void Connected();
        void Disconnected();

        void AttMtuChanged(uint16_t mtu) override;
        void ClientConfigurationWritten(services::AttAttribute::Handle handle, uint16_t value) override;

        services::GattServerService& Service();

    private:
        class WriteHandler
            : public services::GattServerCharacteristicObserver
        {
        public:
            WriteHandler(services::GattServerCharacteristic& characteristic, const infra::Function<void(infra::ConstByteRange data)>& onWrite);
            ~WriteHandler();

            void DataReceived(infra::ConstByteRange data) override;

        private:
            infra::Function<void(infra::ConstByteRange data)> onWrite;
        };

        void MotionWritten(infra::ConstByteRange data);
        void ModeWritten(infra::ConstByteRange data);
        void TuningWritten(infra::ConstByteRange data);

        bool Execute(uint8_t command);
        std::size_t ResponseTo(infra::ConstByteRange data);
        std::size_t DescribeStrategy(uint8_t index);
        std::size_t SelectStrategy(uint8_t index);
        std::size_t DescribeParameter(uint8_t index);
        std::size_t WriteParameter(uint8_t index, infra::ConstByteRange data);
        std::size_t Respond(Status status);
        std::size_t AppendName(std::size_t offset, const char* name);
        void SendResponse(std::size_t length);

        void Tick();
        void PublishMode();
        void PublishTelemetry();
        bool Fits(std::size_t length) const;

        safety::SafetySupervisor& supervisor;
        balance::BalanceControl& balanceControl;
        const telemetry::TelemetrySource& telemetry;
        Config config;

        services::GattServerService service;
        services::GattServerCharacteristicImpl motion;
        services::GattServerCharacteristicImpl mode;
        services::GattServerCharacteristicImpl telemetryValue;
        services::GattServerCharacteristicImpl tuning;
        WriteHandler motionWrites;
        WriteHandler modeWrites;
        WriteHandler tuningWrites;
        infra::TimerRepeating ticker;

        uint16_t mtu{ services::attDefaultMaxMtuSize };
        bool telemetrySubscribed{ false };
        bool telemetryInFlight{ false };
        bool modeInFlight{ false };
        bool tuningInFlight{ false };
        uint8_t lastCommand{ 0 };
        Outcome lastOutcome{ Outcome::none };

        static constexpr std::array<uint8_t, modeSize> unpublished{ 0xff, 0xff, 0xff, 0xff };
        std::array<uint8_t, modeSize> publishedMode{ unpublished };
        std::array<uint8_t, modeSize> modeBuffer{};
        std::array<uint8_t, telemetrySize> telemetryBuffer{};
        std::array<uint8_t, tuningSize> tuningBuffer{};
    };
}
