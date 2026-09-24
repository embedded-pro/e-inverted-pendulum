#include "core/cli/DecimalParser.hpp"
#include <array>
#include <cstdint>

namespace application
{
    namespace
    {
        constexpr std::size_t maximumLength{ 15 };
        constexpr std::size_t maximumIntegerDigits{ 9 };
        constexpr std::size_t maximumFractionDigits{ 7 };
        constexpr std::array<float, maximumFractionDigits + 1> powersOfTen{ 1.0f, 1.0e1f, 1.0e2f, 1.0e3f, 1.0e4f, 1.0e5f, 1.0e6f, 1.0e7f };

        bool IsDigit(char character)
        {
            return character >= '0' && character <= '9';
        }

        uint32_t DigitValue(char character)
        {
            return static_cast<uint32_t>(character - '0');
        }
    }

    std::optional<float> ParseDecimal(infra::BoundedConstString text)
    {
        if (text.size() > maximumLength)
            return std::nullopt;

        std::size_t position{ 0 };
        bool negative{ false };

        if (position < text.size() && (text[position] == '-' || text[position] == '+'))
        {
            negative = text[position] == '-';
            ++position;
        }

        uint32_t integer{ 0 };
        std::size_t integerDigits{ 0 };

        for (; position < text.size() && IsDigit(text[position]); ++position, ++integerDigits)
        {
            if (integerDigits == maximumIntegerDigits)
                return std::nullopt;

            integer = integer * 10 + DigitValue(text[position]);
        }

        uint32_t fraction{ 0 };
        std::size_t fractionDigits{ 0 };
        std::size_t significantFractionDigits{ 0 };

        if (position < text.size() && text[position] == '.')
            for (++position; position < text.size() && IsDigit(text[position]); ++position, ++fractionDigits)
                if (significantFractionDigits < maximumFractionDigits)
                {
                    fraction = fraction * 10 + DigitValue(text[position]);
                    ++significantFractionDigits;
                }

        if (position != text.size() || integerDigits + fractionDigits == 0)
            return std::nullopt;

        const auto magnitude = static_cast<float>(integer) + static_cast<float>(fraction) / powersOfTen[significantFractionDigits];

        return negative ? -magnitude : magnitude;
    }
}
