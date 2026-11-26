#include "core/cli/CalibrationCommand.hpp"
#include "core/cli/NumberParser.hpp"
#include "core/cli/StatusNames.hpp"
#include <array>
#include <cstdint>
#include <numbers>
#include <optional>

namespace application
{
    namespace
    {
        constexpr float degree{ std::numbers::pi_v<float> / 180.0f };
        constexpr std::array<char, 3> axisNames{ 'x', 'y', 'z' };

        struct SignedAxis
        {
            uint8_t from;
            float sign;
        };

        std::optional<SignedAxis> ParseAxis(infra::BoundedConstString text)
        {
            if (text.size() != 2 || (text[0] != '+' && text[0] != '-'))
                return std::nullopt;

            for (uint8_t axis = 0; axis != axisNames.size(); ++axis)
                if (text[1] == axisNames[axis])
                    return SignedAxis{ axis, text[0] == '-' ? -1.0f : 1.0f };

            return std::nullopt;
        }

        std::optional<bool> ParseInverted(infra::BoundedConstString text)
        {
            if (text == "-")
                return true;
            if (text == "+")
                return false;
            return std::nullopt;
        }

        std::optional<bool> ParseLeft(infra::BoundedConstString text)
        {
            if (text == "left")
                return true;
            if (text == "right")
                return false;
            return std::nullopt;
        }

        const char* SignOf(bool inverted)
        {
            return inverted ? "-" : "+";
        }

        const char* SignOf(float sign)
        {
            return sign < 0.0f ? "-" : "+";
        }
    }

    CalibrationCommand::CalibrationCommand(services::Tracer& tracer, const Dependencies& dependencies)
        : tracer(tracer)
        , dependencies(dependencies)
    {}

    void CalibrationCommand::Execute(const infra::BoundedConstString& params)
    {
        if (params.empty())
        {
            Print();
            return;
        }

        const infra::Tokenizer tokens{ params, ' ' };
        if (!Dispatch(tokens))
            tracer.Trace() << "usage: cal [axes <+x|-x|+y|-y|+z|-z> x3 | still <rad/s> | upright [<deg>] | motor <left|right> <+|-> | encoder <left|right> <+|-> | wheel <radius> <track> <gear>]";
    }

    void CalibrationCommand::Print()
    {
        PrintAxes();
        tracer.Trace() << "cal still " << dependencies.sensing.StillnessThreshold() << " rad/s";
        tracer.Trace() << "cal upright " << dependencies.estimation.UprightOffset() / degree << " deg";
        PrintSigns();
        const auto geometry = dependencies.odometry.CurrentGeometry();
        tracer.Trace() << "cal wheel " << geometry.wheelRadius << " " << geometry.trackWidth << " " << geometry.gearRatio;
        tracer.Trace() << "cal store restore " << NameOf(dependencies.store.Restore()) << " save " << NameOf(dependencies.store.LastSave());
    }

    void CalibrationCommand::PrintAxes()
    {
        const auto map = dependencies.sensing.CurrentAxisMap();
        tracer.Trace() << "cal axes " << SignOf(map.xSign) << axisNames[map.xFrom] << " " << SignOf(map.ySign) << axisNames[map.yFrom] << " " << SignOf(map.zSign) << axisNames[map.zFrom];
    }

    void CalibrationCommand::PrintSigns()
    {
        tracer.Trace() << "cal motor left " << SignOf(dependencies.actuation.MotorInverted(motion::WheelSide::left)) << " right " << SignOf(dependencies.actuation.MotorInverted(motion::WheelSide::right));
        tracer.Trace() << "cal encoder left " << SignOf(dependencies.odometry.EncoderInverted(odometry::WheelSide::left)) << " right " << SignOf(dependencies.odometry.EncoderInverted(odometry::WheelSide::right));
    }

    bool CalibrationCommand::Dispatch(const infra::Tokenizer& tokens)
    {
        const auto name = tokens.Token(0);

        if (name == "axes")
            return Axes(tokens);
        if (name == "still")
            return Still(tokens);
        if (name == "upright")
            return Upright(tokens);
        if (name == "motor" || name == "encoder")
            return Sign(tokens, name == "motor");
        if (name == "wheel")
            return Wheel(tokens);
        return false;
    }

    bool CalibrationCommand::Axes(const infra::Tokenizer& tokens)
    {
        if (tokens.Size() != 4)
            return false;

        const auto x = ParseAxis(tokens.Token(1));
        const auto y = ParseAxis(tokens.Token(2));
        const auto z = ParseAxis(tokens.Token(3));
        if (!x.has_value() || !y.has_value() || !z.has_value())
            return false;

        Report(dependencies.calibration.SetAxisMap(platform::AxisMap{ x->from, y->from, z->from, x->sign, y->sign, z->sign }), true);
        return true;
    }

    bool CalibrationCommand::Still(const infra::Tokenizer& tokens)
    {
        const auto threshold = tokens.Size() == 2 ? ParseDecimal(tokens.Token(1)) : std::nullopt;
        if (!threshold.has_value())
            return false;

        Report(dependencies.calibration.SetStillnessThreshold(*threshold), true);
        return true;
    }

    bool CalibrationCommand::Upright(const infra::Tokenizer& tokens)
    {
        if (tokens.Size() == 1)
        {
            Report(dependencies.calibration.CaptureUpright());
            return true;
        }

        const auto offset = tokens.Size() == 2 ? ParseDecimal(tokens.Token(1)) : std::nullopt;
        if (!offset.has_value())
            return false;

        Report(dependencies.calibration.SetUprightOffset(*offset * degree));
        return true;
    }

    bool CalibrationCommand::Sign(const infra::Tokenizer& tokens, bool motor)
    {
        const auto left = tokens.Size() == 3 ? ParseLeft(tokens.Token(1)) : std::nullopt;
        const auto inverted = tokens.Size() == 3 ? ParseInverted(tokens.Token(2)) : std::nullopt;
        if (!left.has_value() || !inverted.has_value())
            return false;

        if (motor)
            Report(dependencies.calibration.SetMotorInverted(*left ? motion::WheelSide::left : motion::WheelSide::right, *inverted));
        else
            Report(dependencies.calibration.SetEncoderInverted(*left ? odometry::WheelSide::left : odometry::WheelSide::right, *inverted));
        return true;
    }

    bool CalibrationCommand::Wheel(const infra::Tokenizer& tokens)
    {
        if (tokens.Size() != 4)
            return false;

        const auto radius = ParseDecimal(tokens.Token(1));
        const auto track = ParseDecimal(tokens.Token(2));
        const auto gear = ParseDecimal(tokens.Token(3));
        if (!radius.has_value() || !track.has_value() || !gear.has_value())
            return false;

        Report(dependencies.calibration.SetGeometry(odometry::Geometry{ *radius, *track, *gear }));
        return true;
    }

    void CalibrationCommand::Report(calibration::Result result, bool needsCalibrate)
    {
        switch (result)
        {
            case calibration::Result::accepted:
                if (needsCalibrate)
                    tracer.Trace() << "ok, run 'calibrate' before arming";
                else
                    tracer.Trace() << "ok";
                break;
            case calibration::Result::armed:
                tracer.Trace() << "refused: calibration changes need disarmed";
                break;
            case calibration::Result::restoring:
                tracer.Trace() << "refused: the saved calibration is still loading";
                break;
            default:
                tracer.Trace() << "refused: value out of range or not a permutation";
                break;
        }
    }
}
