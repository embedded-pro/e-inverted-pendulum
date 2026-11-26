#include "core/calibration/implementations/CalibrationImpl.hpp"
#include "core/persistence/implementations/StoreStatus.hpp"
#include "infra/util/ReallyAssert.hpp"
#include <algorithm>
#include <array>
#include <cmath>

namespace calibration
{
    namespace
    {
        constexpr const char* scope{ "cal" };
        constexpr std::array<const char*, 3> axisKeys{ "axis.x", "axis.y", "axis.z" };
        constexpr std::array<const char*, 2> motorKeys{ "motor.left", "motor.right" };
        constexpr std::array<const char*, 2> encoderKeys{ "encoder.left", "encoder.right" };
        constexpr const char* stillKey{ "still" };
        constexpr const char* uprightKey{ "upright" };
        constexpr std::array<const char*, 3> wheelKeys{ "wheel.radius", "wheel.track", "wheel.gear" };

        services::NvmRecordStore::Config StoreConfig(const CalibrationImpl::Config& config)
        {
            services::NvmRecordStore::Config storeConfig;
            storeConfig.saveDelay = config.saveDelay;
            return storeConfig;
        }

        std::optional<float> Find(const persistence::TuningRecord& record, const char* name)
        {
            const auto key = persistence::KeyOf(scope, name);
            const auto entry = std::ranges::find_if(record.entries, [key](const persistence::TuningEntry& candidate)
                {
                    return candidate.key == key;
                });

            if (entry == record.entries.end())
                return std::nullopt;
            return entry->value;
        }

        void Add(persistence::TuningRecord& record, const char* name, float value)
        {
            really_assert(!record.entries.full());
            record.entries.push_back({ persistence::KeyOf(scope, name), value });
        }

        float SignOf(bool inverted)
        {
            return inverted ? -1.0f : 1.0f;
        }

        float AxisValue(uint8_t from, float sign)
        {
            return sign * static_cast<float>(from + 1);
        }
    }

    CalibrationImpl::Config::Config() = default;

    CalibrationImpl::CalibrationImpl(const Components& components, hal::Eeprom& eeprom, infra::ClaimableResource& access, const Config& config)
        : components(components)
        , first(eeprom, baseAddress, regionSize)
        , second(eeprom, baseAddress + regionSize, regionSize)
        , store(first, second, access, static_cast<services::NvmRecordStore::Client&>(*this), StoreConfig(config))
    {}

    Result CalibrationImpl::SetAxisMap(const platform::AxisMap& map)
    {
        if (Busy())
            return Result::armed;
        return Accept(components.sensing.SetAxisMap(map));
    }

    Result CalibrationImpl::SetStillnessThreshold(float threshold)
    {
        if (Busy())
            return Result::armed;
        return Accept(components.sensing.SetStillnessThreshold(threshold));
    }

    Result CalibrationImpl::SetUprightOffset(float offset)
    {
        if (Busy())
            return Result::armed;
        return Accept(components.estimation.SetUprightOffset(offset));
    }

    Result CalibrationImpl::CaptureUpright()
    {
        if (Busy())
            return Result::armed;

        const auto estimate = components.estimation.Latest();
        if (!estimate.valid)
            return Result::invalid;

        return Accept(components.estimation.SetUprightOffset(components.estimation.UprightOffset() + estimate.pitch));
    }

    Result CalibrationImpl::SetMotorInverted(motion::WheelSide side, bool inverted)
    {
        if (Busy())
            return Result::armed;

        components.actuation.SetMotorInverted(side, inverted);
        return Accept(true);
    }

    Result CalibrationImpl::SetEncoderInverted(odometry::WheelSide side, bool inverted)
    {
        if (Busy())
            return Result::armed;

        components.odometry.SetEncoderInverted(side, inverted);
        return Accept(true);
    }

    Result CalibrationImpl::SetGeometry(const odometry::Geometry& geometry)
    {
        if (Busy())
            return Result::armed;
        return Accept(components.odometry.SetGeometry(geometry));
    }

    persistence::RestoreResult CalibrationImpl::Restore() const
    {
        return persistence::RestoreOf(store.Restore(), restoreFailed);
    }

    persistence::SaveResult CalibrationImpl::LastSave() const
    {
        return persistence::SaveOf(store.LastSave());
    }

    bool CalibrationImpl::Satisfied() const
    {
        return store.Restore() != services::NvmRecordStore::RestoreResult::pending;
    }

    void CalibrationImpl::Restored(infra::ConstByteRange payload)
    {
        const auto record = persistence::Decode(payload);
        restoreFailed = !record.has_value() || !Apply(*record);
    }

    std::size_t CalibrationImpl::Capture(infra::ByteRange payload)
    {
        return persistence::Encode(Record(), payload);
    }

    bool CalibrationImpl::Busy() const
    {
        return components.control.Engaged();
    }

    Result CalibrationImpl::Accept(bool accepted)
    {
        if (!accepted)
            return Result::invalid;

        store.ScheduleSave();
        return Result::accepted;
    }

    persistence::TuningRecord CalibrationImpl::Record() const
    {
        persistence::TuningRecord record;
        const auto map = components.sensing.CurrentAxisMap();
        Add(record, axisKeys[0], AxisValue(map.xFrom, map.xSign));
        Add(record, axisKeys[1], AxisValue(map.yFrom, map.ySign));
        Add(record, axisKeys[2], AxisValue(map.zFrom, map.zSign));
        Add(record, stillKey, components.sensing.StillnessThreshold());
        Add(record, uprightKey, components.estimation.UprightOffset());
        Add(record, motorKeys[0], SignOf(components.actuation.MotorInverted(motion::WheelSide::left)));
        Add(record, motorKeys[1], SignOf(components.actuation.MotorInverted(motion::WheelSide::right)));
        Add(record, encoderKeys[0], SignOf(components.odometry.EncoderInverted(odometry::WheelSide::left)));
        Add(record, encoderKeys[1], SignOf(components.odometry.EncoderInverted(odometry::WheelSide::right)));
        const auto geometry = components.odometry.CurrentGeometry();
        Add(record, wheelKeys[0], geometry.wheelRadius);
        Add(record, wheelKeys[1], geometry.trackWidth);
        Add(record, wheelKeys[2], geometry.gearRatio);
        return record;
    }

    bool CalibrationImpl::Apply(const persistence::TuningRecord& record)
    {
        bool applied = ApplyAxisMap(record);

        if (const auto still = Find(record, stillKey); still.has_value())
            applied = components.sensing.SetStillnessThreshold(*still) && applied;
        if (const auto upright = Find(record, uprightKey); upright.has_value())
            applied = components.estimation.SetUprightOffset(*upright) && applied;

        ApplySigns(record);
        return ApplyGeometry(record) && applied;
    }

    bool CalibrationImpl::ApplyAxisMap(const persistence::TuningRecord& record)
    {
        std::array<std::optional<float>, 3> axes{ Find(record, axisKeys[0]), Find(record, axisKeys[1]), Find(record, axisKeys[2]) };
        if (!std::ranges::all_of(axes, [](const std::optional<float>& axis)
                {
                    return axis.has_value() && (std::fabs(*axis) == 1.0f || std::fabs(*axis) == 2.0f || std::fabs(*axis) == 3.0f);
                }))
            return std::ranges::none_of(axes, [](const std::optional<float>& axis)
                {
                    return axis.has_value();
                });

        const auto from = [](float value)
        {
            return static_cast<uint8_t>(std::lround(std::fabs(value)) - 1);
        };
        const auto sign = [](float value)
        {
            return value < 0.0f ? -1.0f : 1.0f;
        };

        return components.sensing.SetAxisMap(platform::AxisMap{ from(*axes[0]), from(*axes[1]), from(*axes[2]), sign(*axes[0]), sign(*axes[1]), sign(*axes[2]) });
    }

    void CalibrationImpl::ApplySigns(const persistence::TuningRecord& record)
    {
        if (const auto left = Find(record, motorKeys[0]); left.has_value())
            components.actuation.SetMotorInverted(motion::WheelSide::left, *left < 0.0f);
        if (const auto right = Find(record, motorKeys[1]); right.has_value())
            components.actuation.SetMotorInverted(motion::WheelSide::right, *right < 0.0f);
        if (const auto left = Find(record, encoderKeys[0]); left.has_value())
            components.odometry.SetEncoderInverted(odometry::WheelSide::left, *left < 0.0f);
        if (const auto right = Find(record, encoderKeys[1]); right.has_value())
            components.odometry.SetEncoderInverted(odometry::WheelSide::right, *right < 0.0f);
    }

    bool CalibrationImpl::ApplyGeometry(const persistence::TuningRecord& record)
    {
        auto geometry = components.odometry.CurrentGeometry();
        geometry.wheelRadius = Find(record, wheelKeys[0]).value_or(geometry.wheelRadius);
        geometry.trackWidth = Find(record, wheelKeys[1]).value_or(geometry.trackWidth);
        geometry.gearRatio = Find(record, wheelKeys[2]).value_or(geometry.gearRatio);
        return components.odometry.SetGeometry(geometry);
    }
}
