#pragma once

#include <cstdint>
#include "clock_shared.hpp"
#include "rcc.hpp"
#include "tim.hpp"

namespace vermils
{
namespace stm32
{
namespace clock
{
    using rcc::enable_clock;
    using rcc::disable_clock;
}
}
}
