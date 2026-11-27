#include "core/ble_link/CalibrationAccess.hpp"
#include "core/ble_link/WireFormat.hpp"
#include "infra/util/EnumCast.hpp"
#include <algorithm>
#include <optional>
#include <utility>

namespace ble
{
    namespace
    {
        using Properties = services::GattCharacteristic::PropertyFlags;
        using Permissions = services::GattServerCharacteristic::PermissionFlags;

        constexpr std::size_t headerSize{ 3 };
        constexpr std::size_t requestHeaderSize{ 2 };
        constexpr std::size_t attributeValueHeaderSize{ 3 };
        constexpr uint8_t right{ 1 };

        std::optional<std::pair<uint8_t, float>> AxisOf(uint8_t value)
        {
            const auto axis = static_cast<int8_t>(value);
            const auto from = axis < 0 ? -axis : axis;

            if (from < 1 || from > 3)
                return std::nullopt;

            return std::pair{ static_cast<uint8_t>(from - 1), axis < 0 ? -1.0f : 1.0f };
        }

        uint8_t EncodeAxis(uint8_t from, float sign)
        {
            const auto axis = static_cast<int8_t>(from + 1);
            return static_cast<uint8_t>(sign < 0.0f ? -axis : axis);
        }

        uint8_t EncodeSign(bool inverted)
        {
            return static_cast<uint8_t>(inverted ? -1 : 1);
        }

        std::optional<bool> InvertedOf(uint8_t value)
        {
            const auto sign = static_cast<int8_t>(value);

            if (sign == 1)
                return false;
            if (sign == -1)
                return true;

            return std::nullopt;
        }
    }

    CalibrationAccess::CalibrationAccess(services::GattServerService& service, const Dependencies& dependencies)
        : dependencies(dependencies)
        , characteristic(service, uuid::calibration, calibrationSize, Properties::write | Properties::notify, Permissions::encryptedWrite)
    {
        Attach(characteristic);
    }

    CalibrationAccess::~CalibrationAccess()
    {
        Detach();
    }

    void CalibrationAccess::Disconnected()
    {
        mtu = services::attDefaultMaxMtuSize;
        inFlight = false;
    }

    void CalibrationAccess::AttMtuChanged(uint16_t newMtu)
    {
        mtu = std::max(newMtu, services::attDefaultMaxMtuSize);
    }

    void CalibrationAccess::DataReceived(infra::ConstByteRange data)
    {
        if (inFlight)
            return;

        buffer[0] = data.empty() ? 0 : data[0];
        buffer[1] = data.size() < requestHeaderSize ? 0 : data[1];

        SendResponse(ResponseTo(data));
    }

    std::size_t CalibrationAccess::ResponseTo(infra::ConstByteRange data)
    {
        if (data.size() < requestHeaderSize)
            return Respond(Status::invalidRequest);

        const auto operation = static_cast<Operation>(data[0]);
        const auto side = data[1];
        const auto value = infra::DiscardHead(data, requestHeaderSize);
        const bool usesSide = operation == Operation::setMotorSign || operation == Operation::setEncoderSign;

        if (side > (usesSide ? right : 0))
            return Respond(Status::invalidRequest);

        return Change(operation, side, value);
    }

    std::size_t CalibrationAccess::Change(Operation operation, uint8_t side, infra::ConstByteRange value)
    {
        switch (operation)
        {
            case Operation::readAll:
                return value.empty() ? ReadAll() : Respond(Status::invalidRequest);
            case Operation::setAxisMap:
                return SetAxisMap(value);
            case Operation::setStillness:
            case Operation::setUpright:
                return SetFloat(operation, value);
            case Operation::captureUpright:
                return value.empty() ? Report(dependencies.calibration.CaptureUpright()) : Respond(Status::invalidRequest);
            case Operation::setMotorSign:
            case Operation::setEncoderSign:
                return SetSign(operation, side, value);
            case Operation::setGeometry:
                return SetGeometry(value);
            default:
                return Respond(Status::invalidRequest);
        }
    }

    std::size_t CalibrationAccess::ReadAll()
    {
        if (calibrationSize + attributeValueHeaderSize > mtu)
            return Respond(Status::mtuTooSmall);

        Respond(Status::accepted);
        const auto map = dependencies.sensing.CurrentAxisMap();
        const auto geometry = dependencies.odometry.CurrentGeometry();
        const auto range = infra::MakeRange(buffer);

        buffer[3] = EncodeAxis(map.xFrom, map.xSign);
        buffer[4] = EncodeAxis(map.yFrom, map.ySign);
        buffer[5] = EncodeAxis(map.zFrom, map.zSign);
        PutFloat(range, 6, dependencies.sensing.StillnessThreshold());
        PutFloat(range, 10, dependencies.estimation.UprightOffset());
        buffer[14] = EncodeSign(dependencies.actuation.MotorInverted(motion::WheelSide::left));
        buffer[15] = EncodeSign(dependencies.actuation.MotorInverted(motion::WheelSide::right));
        buffer[16] = EncodeSign(dependencies.odometry.EncoderInverted(odometry::WheelSide::left));
        buffer[17] = EncodeSign(dependencies.odometry.EncoderInverted(odometry::WheelSide::right));
        PutFloat(range, 18, geometry.wheelRadius);
        PutFloat(range, 22, geometry.trackWidth);
        PutFloat(range, 26, geometry.gearRatio);
        return calibrationSize;
    }

    std::size_t CalibrationAccess::SetAxisMap(infra::ConstByteRange value)
    {
        if (value.size() != 3)
            return Respond(Status::invalidRequest);

        const auto x = AxisOf(value[0]);
        const auto y = AxisOf(value[1]);
        const auto z = AxisOf(value[2]);

        if (!x || !y || !z)
            return Respond(Status::invalidRequest);

        return Report(dependencies.calibration.SetAxisMap(platform::AxisMap{ x->first, y->first, z->first, x->second, y->second, z->second }));
    }

    std::size_t CalibrationAccess::SetFloat(Operation operation, infra::ConstByteRange value)
    {
        if (value.size() != sizeof(float))
            return Respond(Status::invalidRequest);

        const auto number = GetFloat(value, 0);

        if (operation == Operation::setStillness)
            return Report(dependencies.calibration.SetStillnessThreshold(number));

        return Report(dependencies.calibration.SetUprightOffset(number));
    }

    std::size_t CalibrationAccess::SetSign(Operation operation, uint8_t side, infra::ConstByteRange value)
    {
        const auto inverted = value.size() == 1 ? InvertedOf(value[0]) : std::nullopt;

        if (!inverted)
            return Respond(Status::invalidRequest);

        if (operation == Operation::setMotorSign)
            return Report(dependencies.calibration.SetMotorInverted(static_cast<motion::WheelSide>(side), *inverted));

        return Report(dependencies.calibration.SetEncoderInverted(static_cast<odometry::WheelSide>(side), *inverted));
    }

    std::size_t CalibrationAccess::SetGeometry(infra::ConstByteRange value)
    {
        if (value.size() != 3 * sizeof(float))
            return Respond(Status::invalidRequest);

        return Report(dependencies.calibration.SetGeometry(odometry::Geometry{ GetFloat(value, 0), GetFloat(value, 4), GetFloat(value, 8) }));
    }

    std::size_t CalibrationAccess::Report(calibration::Result result)
    {
        switch (result)
        {
            case calibration::Result::accepted:
                return Respond(Status::accepted);
            case calibration::Result::armed:
                return Respond(Status::armed);
            case calibration::Result::restoring:
                return Respond(Status::restoring);
            default:
                return Respond(Status::invalidRequest);
        }
    }

    std::size_t CalibrationAccess::Respond(Status status)
    {
        buffer[2] = infra::enum_cast(status);
        return headerSize;
    }

    void CalibrationAccess::SendResponse(std::size_t length)
    {
        inFlight = true;
        characteristic.Update(infra::ConstByteRange(buffer.data(), buffer.data() + length), [this]()
            {
                inFlight = false;
            });
    }
}
