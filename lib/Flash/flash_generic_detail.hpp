#pragma once

#include <cstdint>
#include "userconfig.hpp"

#if _VERMIL_STM32_USE_GENERIC

namespace vms
{
namespace stm32
{
namespace flash
{

namespace detail
{
    struct FlashRegister
    {
        volatile uint32_t ACR;
    };

    inline FlashRegister flash_reg;
}

}
}
}

#endif