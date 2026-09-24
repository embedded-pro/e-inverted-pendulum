#include "core/cli/NumberParser.hpp"
#include "infra/stream/StringInputStream.hpp"

namespace application
{
    namespace
    {
        constexpr std::size_t maximumLength{ 11 };
        constexpr std::size_t maximumIndexLength{ 3 };
    }

    std::optional<float> ParseDecimal(infra::BoundedConstString text)
    {
        if (text.size() > maximumLength || text.find('.') == infra::BoundedConstString::npos)
            return std::nullopt;

        infra::StringInputStream stream{ text, infra::softFail };
        float value{ 0.0f };
        stream >> value;

        if (stream.Failed() || !stream.Empty())
            return std::nullopt;

        return value;
    }

    std::optional<uint32_t> ParseIndex(infra::BoundedConstString text)
    {
        if (text.empty() || text.size() > maximumIndexLength)
            return std::nullopt;

        infra::StringInputStream stream{ text, infra::softFail };
        uint32_t value{ 0 };
        stream >> value;

        if (stream.Failed() || !stream.Empty())
            return std::nullopt;

        return value;
    }
}
