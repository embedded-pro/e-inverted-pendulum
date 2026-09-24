#pragma once

#include "infra/util/BoundedString.hpp"
#include <optional>

namespace application
{
    std::optional<float> ParseDecimal(infra::BoundedConstString text);
}
