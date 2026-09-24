#include "targets/platform_implementations/st/WheelEncodersStm.hpp"

namespace application
{
    hal::SynchronousQuadratureEncoderLpTimStm::Config WheelEncodersStm::LeftEncoderConfig()
    {
        hal::SynchronousQuadratureEncoderLpTimStm::Config config;

        config.decodeMode = hal::SynchronousQuadratureEncoderLpTimStm::Config::DecodeMode::x4OnBothEdges;

        return config;
    }

    hal::SynchronousQuadratureEncoderStm::Config WheelEncodersStm::RightEncoderConfig()
    {
        hal::SynchronousQuadratureEncoderStm::Config config;

        config.decodeMode = hal::SynchronousQuadratureEncoderStm::Config::DecodeMode::x4OnBothPhases;
        config.invertPhaseA = true;

        return config;
    }

    WheelEncodersStm::WheelEncodersStm()
        : left(1, leftPhaseA, leftPhaseB, leftIndex, LeftEncoderConfig())
        , right(2, rightPhaseA, rightPhaseB, rightIndex, RightEncoderConfig())
    {}

    hal::SynchronousQuadratureEncoder& WheelEncodersStm::Left()
    {
        return left;
    }

    hal::SynchronousQuadratureEncoder& WheelEncodersStm::Right()
    {
        return right;
    }
}
