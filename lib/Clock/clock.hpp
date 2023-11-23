#pragma once

#include <cstdint>
namespace vermils
{
namespace stm32
{
namespace clock
{
    extern uint32_t & SystemCoreClock;
}
using clock::SystemCoreClock;
}
}