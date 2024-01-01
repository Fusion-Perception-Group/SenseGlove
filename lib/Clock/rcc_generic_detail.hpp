#pragma once

#include <cstdint>
#include "userconfig.hpp"

#if _VERMIL_STM32_USE_GENERIC

namespace vermils
{
namespace stm32
{
namespace clock
{
namespace rcc
{

namespace detail
{
    struct RCCRegister
    {
    };

    inline RCCRegister RCCReg;
}

}
}
}
}

#endif
