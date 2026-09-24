#include "core/balance_control/implementations/ContinuousPid.hpp"
#include "gtest/gtest.h"

TEST(ContinuousPidTest, the_proportional_term_acts_on_the_error)
{
    balance::ContinuousPid pid{ 1.0f };

    EXPECT_NEAR(0.25f, pid.Process(0.5f, 0.0f, balance::PidGains{ 0.5f, 0.0f, 0.0f }, 0.02f), 1e-6f);
}

TEST(ContinuousPidTest, the_integral_term_is_scaled_by_the_measured_interval)
{
    balance::ContinuousPid pid{ 1.0f };
    const balance::PidGains integralOnly{ 0.0f, 2.0f, 0.0f };

    EXPECT_NEAR(0.02f, pid.Process(0.5f, 0.0f, integralOnly, 0.02f), 1e-6f);
    EXPECT_NEAR(0.06f, pid.Process(0.5f, 0.0f, integralOnly, 0.04f), 1e-6f);
}

TEST(ContinuousPidTest, the_derivative_term_is_divided_by_the_measured_interval)
{
    balance::ContinuousPid pid{ 1.0f };

    EXPECT_NEAR(0.25f, pid.Process(0.1f, 0.0f, balance::PidGains{ 0.0f, 0.0f, 0.05f }, 0.02f), 1e-6f);
}

TEST(ContinuousPidTest, the_output_is_held_within_its_limit)
{
    balance::ContinuousPid pid{ 0.3f };

    EXPECT_NEAR(0.3f, pid.Process(10.0f, 0.0f, balance::PidGains{ 1.0f, 0.0f, 0.0f }, 0.02f), 1e-6f);
    EXPECT_NEAR(-0.3f, pid.Process(-10.0f, 0.0f, balance::PidGains{ 1.0f, 0.0f, 0.0f }, 0.02f), 1e-6f);
}

TEST(ContinuousPidTest, reset_forgets_accumulated_output)
{
    balance::ContinuousPid pid{ 1.0f };
    const balance::PidGains integralOnly{ 0.0f, 2.0f, 0.0f };
    pid.Process(0.5f, 0.0f, integralOnly, 0.02f);

    pid.Reset();

    EXPECT_NEAR(0.0f, pid.Process(0.0f, 0.0f, integralOnly, 0.02f), 1e-6f);
}
