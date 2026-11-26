#pragma once

#include "core/attitude_estimation/interfaces/AttitudeEstimation.hpp"
#include "core/balance_control/interfaces/BalanceControl.hpp"
#include "core/calibration/interfaces/Calibration.hpp"
#include "core/inertial_sensing/interfaces/InertialSensing.hpp"
#include "core/persistence/implementations/TuningRecord.hpp"
#include "core/persistence/interfaces/TuningStore.hpp"
#include "core/safety_supervisor/interfaces/StartupCondition.hpp"
#include "hal/interfaces/Eeprom.hpp"
#include "infra/event/ClaimableResource.hpp"
#include "services/util/NvmEepromRegion.hpp"
#include "services/util/NvmRecordStore.hpp"
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace calibration
{
    class CalibrationImpl final
        : public Calibration
        , public persistence::TuningStore
        , public safety::StartupCondition
        , private services::NvmRecordStore::Client
    {
    public:
        static constexpr uint32_t baseAddress{ 256 };
        static constexpr uint32_t regionSize{ 128 };

        struct Components
        {
            sensing::InertialSensing& sensing;
            estimation::AttitudeEstimation& estimation;
            motion::MotionActuation& actuation;
            odometry::WheelOdometry& odometry;
            const balance::BalanceControl& control;
        };

        struct Config
        {
            Config();

            infra::Duration saveDelay{ std::chrono::seconds(2) };
        };

        CalibrationImpl(const Components& components, hal::Eeprom& eeprom, infra::ClaimableResource& access, const Config& config = Config());

        Result SetAxisMap(const platform::AxisMap& map) override;
        Result SetStillnessThreshold(float threshold) override;
        Result SetUprightOffset(float offset) override;
        Result CaptureUpright() override;
        Result SetMotorInverted(motion::WheelSide side, bool inverted) override;
        Result SetEncoderInverted(odometry::WheelSide side, bool inverted) override;
        Result SetGeometry(const odometry::Geometry& geometry) override;

        persistence::RestoreResult Restore() const override;
        persistence::SaveResult LastSave() const override;
        bool Satisfied() const override;

    private:
        static_assert(services::NvmRecordStore::headerSize + persistence::TuningRecord::maxSize <= regionSize);

        void Restored(infra::ConstByteRange payload) override;
        std::size_t Capture(infra::ByteRange payload) override;
        bool Busy() const override;

        std::optional<Result> Refusal() const;
        Result Accept(bool accepted);
        persistence::TuningRecord Record() const;
        bool Apply(const persistence::TuningRecord& record);
        bool ApplyAxisMap(const persistence::TuningRecord& record);
        void ApplySigns(const persistence::TuningRecord& record);
        bool ApplyGeometry(const persistence::TuningRecord& record);

        Components components;
        services::NvmEepromRegion first;
        services::NvmEepromRegion second;
        bool restoreFailed{ false };
        services::NvmRecordStore::WithRegionSize<regionSize> store;
    };
}
