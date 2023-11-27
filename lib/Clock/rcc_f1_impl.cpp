#include "_config.hpp"
#include "rcc.hpp"

#if defined(__VERMIL_STM32F1) && !__VERMIL_STM32_USE_GENERIC

namespace vermils
{
namespace stm32
{
namespace clock
{
namespace rcc
{

void reset_backup_domain() noexcept
{
    detail::RCCReg.BDCR |= 1U << 16;
    detail::RCCReg.BDCR &= ~(1U << 16);
}

}
}
}
}

#endif
