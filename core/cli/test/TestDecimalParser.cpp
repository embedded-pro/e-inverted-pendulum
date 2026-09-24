#include "core/cli/DecimalParser.hpp"
#include "gtest/gtest.h"
#include <optional>

namespace
{
    float Parsed(const char* text)
    {
        const auto value = application::ParseDecimal(text);
        EXPECT_TRUE(value.has_value()) << text;
        return value.value_or(0.0f);
    }

    bool Rejected(const char* text)
    {
        return !application::ParseDecimal(text).has_value();
    }
}

TEST(DecimalParserTest, parses_decimals)
{
    EXPECT_FLOAT_EQ(0.3f, Parsed("0.3"));
    EXPECT_FLOAT_EQ(1.0f, Parsed("1.0"));
    EXPECT_FLOAT_EQ(1.125f, Parsed("1.125"));
    EXPECT_FLOAT_EQ(250.5f, Parsed("250.5"));
}

TEST(DecimalParserTest, parses_negative_decimals)
{
    EXPECT_FLOAT_EQ(-0.7f, Parsed("-0.7"));
    EXPECT_FLOAT_EQ(-1.0f, Parsed("-1.0"));
}

TEST(DecimalParserTest, parses_up_to_nine_fraction_digits)
{
    EXPECT_NEAR(0.123456789f, Parsed("0.123456789"), 1e-7f);
}

TEST(DecimalParserTest, a_decimal_point_is_required)
{
    EXPECT_TRUE(Rejected("1"));
    EXPECT_TRUE(Rejected("-12"));
    EXPECT_TRUE(Rejected("--1"));
}

TEST(DecimalParserTest, rejects_more_than_eleven_characters)
{
    EXPECT_FLOAT_EQ(-0.12345678f, Parsed("-0.12345678"));
    EXPECT_TRUE(Rejected("0.1234567890"));
}

TEST(DecimalParserTest, rejects_empty_and_incomplete_numbers)
{
    EXPECT_TRUE(Rejected(""));
    EXPECT_TRUE(Rejected("-"));
    EXPECT_TRUE(Rejected("."));
    EXPECT_TRUE(Rejected(".5"));
    EXPECT_TRUE(Rejected("3."));
}

TEST(DecimalParserTest, rejects_anything_but_one_plain_decimal)
{
    EXPECT_TRUE(Rejected("abc"));
    EXPECT_TRUE(Rejected("0.3x"));
    EXPECT_TRUE(Rejected("1.2.3"));
    EXPECT_TRUE(Rejected("1.0e3"));
    EXPECT_TRUE(Rejected("+0.3"));
    EXPECT_TRUE(Rejected("nan"));
}
