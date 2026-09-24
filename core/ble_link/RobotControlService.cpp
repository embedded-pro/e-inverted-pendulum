#include "core/ble_link/RobotControlService.hpp"
#include "core/ble_link/WireFormat.hpp"
#include "infra/util/EnumCast.hpp"
#include <algorithm>
#include <cstring>

namespace ble
{
    namespace
    {
        using Properties = services::GattCharacteristic::PropertyFlags;
        using Permissions = services::GattServerCharacteristic::PermissionFlags;

        constexpr uint16_t enableNotification{ 0x0001 };
        constexpr std::size_t responseHeaderSize{ 3 };
        constexpr std::size_t attributeValueHeaderSize{ 3 };
    }

    RobotControlService::Config::Config() = default;

    RobotControlService::WriteHandler::WriteHandler(services::GattServerCharacteristic& characteristic, const infra::Function<void(infra::ConstByteRange data)>& onWrite)
        : onWrite(onWrite)
    {
        Attach(characteristic);
    }

    RobotControlService::WriteHandler::~WriteHandler()
    {
        Detach();
    }

    void RobotControlService::WriteHandler::DataReceived(infra::ConstByteRange data)
    {
        onWrite(data);
    }

    RobotControlService::RobotControlService(services::GattServer& gattServer, safety::SafetySupervisor& supervisor, balance::BalanceControl& balanceControl, const telemetry::TelemetrySource& telemetry, const Config& config)
        : supervisor(supervisor)
        , balanceControl(balanceControl)
        , telemetry(telemetry)
        , config(config)
        , service(uuid::robotControlService)
        , motion(service, uuid::motion, motionSize, Properties::writeWithoutResponse, Permissions::encryptedWrite)
        , mode(service, uuid::mode, modeSize, Properties::read | Properties::write | Properties::notify, Permissions::encryptedWrite)
        , telemetryValue(service, uuid::telemetry, telemetrySize, Properties::read | Properties::notify, Permissions::none)
        , tuning(service, uuid::tuning, tuningSize, Properties::write | Properties::notify, Permissions::encryptedWrite)
        , motionWrites(motion, [this](infra::ConstByteRange data)
              {
                  MotionWritten(data);
              })
        , modeWrites(mode, [this](infra::ConstByteRange data)
              {
                  ModeWritten(data);
              })
        , tuningWrites(tuning, [this](infra::ConstByteRange data)
              {
                  TuningWritten(data);
              })
    {
        gattServer.AddService(service);
    }

    void RobotControlService::Connected()
    {
        ticker.Start(config.telemetryPeriod, [this]()
            {
                Tick();
            });

        PublishMode();
    }

    void RobotControlService::Disconnected()
    {
        ticker.Cancel();
        balanceControl.CancelMotion();

        mtu = services::attDefaultMaxMtuSize;
        telemetrySubscribed = false;
        telemetryInFlight = false;
        modeInFlight = false;
        tuningInFlight = false;
        publishedMode = unpublished;
    }

    void RobotControlService::AttMtuChanged(uint16_t newMtu)
    {
        mtu = std::max(newMtu, services::attDefaultMaxMtuSize);
    }

    void RobotControlService::ClientConfigurationWritten(services::AttAttribute::Handle handle, uint16_t value)
    {
        if (handle == telemetryValue.Handle() + clientConfigurationOffset)
            telemetrySubscribed = (value & enableNotification) != 0;
    }

    services::GattServerService& RobotControlService::Service()
    {
        return service;
    }

    void RobotControlService::MotionWritten(infra::ConstByteRange data)
    {
        if (data.size() == motionSize)
            balanceControl.Move(balance::Setpoints{ GetFloat(data, 0), GetFloat(data, sizeof(float)) });
    }

    void RobotControlService::ModeWritten(infra::ConstByteRange data)
    {
        if (data.size() != 1)
            return;

        lastCommand = data[0];
        lastOutcome = Execute(data[0]) ? Outcome::accepted : Outcome::refused;
        PublishMode();
    }

    bool RobotControlService::Execute(uint8_t command)
    {
        switch (static_cast<Command>(command))
        {
            case Command::arm:
                return supervisor.Arm();
            case Command::disarm:
                return supervisor.Disarm();
            case Command::clearFault:
                return supervisor.ClearFault();
            case Command::calibrate:
                return supervisor.Calibrate();
            default:
                return false;
        }
    }

    void RobotControlService::TuningWritten(infra::ConstByteRange data)
    {
        if (tuningInFlight)
            return;

        tuningBuffer[0] = data.empty() ? 0 : data[0];
        tuningBuffer[1] = data.size() < 2 ? 0 : data[1];

        SendResponse(ResponseTo(data));
    }

    std::size_t RobotControlService::ResponseTo(infra::ConstByteRange data)
    {
        if (data.size() < 2)
            return Respond(Status::invalidRequest);

        const auto index = data[1];
        const bool indexOnly = data.size() == 2;

        switch (static_cast<Operation>(data[0]))
        {
            case Operation::describeStrategy:
                return indexOnly ? DescribeStrategy(index) : Respond(Status::invalidRequest);
            case Operation::selectStrategy:
                return indexOnly ? SelectStrategy(index) : Respond(Status::invalidRequest);
            case Operation::describeParameter:
                return indexOnly ? DescribeParameter(index) : Respond(Status::invalidRequest);
            case Operation::writeParameter:
                return WriteParameter(index, data);
            default:
                return Respond(Status::invalidRequest);
        }
    }

    std::size_t RobotControlService::DescribeStrategy(uint8_t index)
    {
        if (index >= balanceControl.StrategyCount())
            return Respond(Status::invalidRequest);

        Respond(Status::accepted);
        tuningBuffer[3] = static_cast<uint8_t>(balanceControl.StrategyCount());
        tuningBuffer[4] = static_cast<uint8_t>(balanceControl.ActiveStrategy());
        return AppendName(5, balanceControl.StrategyName(index));
    }

    std::size_t RobotControlService::SelectStrategy(uint8_t index)
    {
        if (index >= balanceControl.StrategyCount())
            return Respond(Status::invalidRequest);

        return Respond(balanceControl.Select(index) ? Status::accepted : Status::refused);
    }

    std::size_t RobotControlService::DescribeParameter(uint8_t index)
    {
        const auto descriptors = balanceControl.Parameters();

        if (index >= descriptors.size())
            return Respond(Status::invalidRequest);

        Respond(Status::accepted);
        tuningBuffer[3] = static_cast<uint8_t>(descriptors.size());
        PutFloat(infra::MakeRange(tuningBuffer), 4, balanceControl.Parameter(index));
        PutFloat(infra::MakeRange(tuningBuffer), 8, descriptors[index].minimum);
        PutFloat(infra::MakeRange(tuningBuffer), 12, descriptors[index].maximum);
        return AppendName(16, descriptors[index].name);
    }

    std::size_t RobotControlService::WriteParameter(uint8_t index, infra::ConstByteRange data)
    {
        if (data.size() != 2 + sizeof(float) || index >= balanceControl.Parameters().size())
            return Respond(Status::invalidRequest);

        Respond(balanceControl.SetParameter(index, GetFloat(data, 2)) ? Status::accepted : Status::refused);
        PutFloat(infra::MakeRange(tuningBuffer), responseHeaderSize, balanceControl.Parameter(index));
        return responseHeaderSize + sizeof(float);
    }

    std::size_t RobotControlService::Respond(Status status)
    {
        tuningBuffer[2] = infra::enum_cast(status);
        return responseHeaderSize;
    }

    std::size_t RobotControlService::AppendName(std::size_t offset, const char* name)
    {
        const auto length = std::min(std::strlen(name), maximumNameLength);

        tuningBuffer[offset] = static_cast<uint8_t>(length);
        std::copy_n(name, length, tuningBuffer.begin() + offset + 1);
        return offset + 1 + length;
    }

    void RobotControlService::SendResponse(std::size_t length)
    {
        if (!Fits(length))
            length = Respond(Status::mtuTooSmall);

        tuningInFlight = true;
        tuning.Update(infra::ConstByteRange(tuningBuffer.data(), tuningBuffer.data() + length), [this]()
            {
                tuningInFlight = false;
            });
    }

    void RobotControlService::Tick()
    {
        PublishMode();
        PublishTelemetry();
    }

    void RobotControlService::PublishMode()
    {
        const std::array<uint8_t, modeSize> current{ infra::enum_cast(supervisor.Current()), infra::enum_cast(supervisor.LatchedCause()), lastCommand, infra::enum_cast(lastOutcome) };

        if (modeInFlight || current == publishedMode)
            return;

        publishedMode = current;
        modeBuffer = current;
        modeInFlight = true;
        mode.Update(infra::MakeRange(modeBuffer), [this]()
            {
                modeInFlight = false;
            });
    }

    void RobotControlService::PublishTelemetry()
    {
        if (!telemetrySubscribed || telemetryInFlight || !Fits(telemetrySize))
            return;

        const auto sample = telemetry.Latest();
        const auto buffer = infra::MakeRange(telemetryBuffer);

        PutFloat(buffer, 0, sample.pitch);
        PutFloat(buffer, 4, sample.pitchRate);
        PutFloat(buffer, 8, sample.forwardVelocity);
        PutFloat(buffer, 12, sample.yawRate);
        PutFloat(buffer, 16, sample.effortLeft);
        PutFloat(buffer, 20, sample.effortRight);
        telemetryBuffer[24] = infra::enum_cast(sample.mode);
        telemetryBuffer[25] = infra::enum_cast(sample.cause);

        telemetryInFlight = true;
        telemetryValue.Update(infra::MakeRange(telemetryBuffer), [this]()
            {
                telemetryInFlight = false;
            });
    }

    bool RobotControlService::Fits(std::size_t length) const
    {
        return length + attributeValueHeaderSize <= mtu;
    }
}
