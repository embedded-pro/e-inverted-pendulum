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

TEST(DecimalParserTest, parses_integers)
{
    EXPECT_FLOAT_EQ(0.0f, Parsed("0"));
    EXPECT_FLOAT_EQ(1.0f, Parsed("1"));
    EXPECT_FLOAT_EQ(250.0f, Parsed("250"));
}

TEST(DecimalParserTest, parses_signed_values)
{
    EXPECT_FLOAT_EQ(-0.7f, Parsed("-0.7"));
    EXPECT_FLOAT_EQ(0.3f, Parsed("+0.3"));
    EXPECT_FLOAT_EQ(-12.0f, Parsed("-12"));
}

TEST(DecimalParserTest, parses_fractions_with_or_without_leading_or_trailing_digits)
{
    EXPECT_FLOAT_EQ(0.5f, Parsed(".5"));
    EXPECT_FLOAT_EQ(-0.25f, Parsed("-.25"));
    EXPECT_FLOAT_EQ(3.0f, Parsed("3."));
    EXPECT_FLOAT_EQ(1.125f, Parsed("1.125"));
}

TEST(DecimalParserTest, precision_beyond_seven_fraction_digits_is_ignored)
{
    EXPECT_NEAR(0.1234567f, Parsed("0.1234567890123"), 1e-7f);
}

TEST(DecimalParserTest, accepts_up_to_nine_integer_digits)
{
    EXPECT_FLOAT_EQ(999999999.0f, Parsed("999999999"));
    EXPECT_TRUE(Rejected("1000000000"));
}

TEST(DecimalParserTest, rejects_empty_and_sign_or_point_alone)
{
    EXPECT_TRUE(Rejected(""));
    EXPECT_TRUE(Rejected("-"));
    EXPECT_TRUE(Rejected("+"));
    EXPECT_TRUE(Rejected("."));
    EXPECT_TRUE(Rejected("-."));
}

TEST(DecimalParserTest, rejects_anything_but_one_plain_decimal)
{
    EXPECT_TRUE(Rejected("abc"));
    EXPECT_TRUE(Rejected("0.3x"));
    EXPECT_TRUE(Rejected("1.2.3"));
    EXPECT_TRUE(Rejected("1e3"));
    EXPECT_TRUE(Rejected(" 1"));
    EXPECT_TRUE(Rejected("1 "));
    EXPECT_TRUE(Rejected("--1"));
    EXPECT_TRUE(Rejected("nan"));
    EXPECT_TRUE(Rejected("inf"));
}

TEST(DecimalParserTest, rejects_more_than_fifteen_characters)
{
    EXPECT_FLOAT_EQ(0.1234567f, Parsed("0.1234567000000"));
    EXPECT_TRUE(Rejected("0.12345670000000"));
}
