#pragma once

#include <cstdint>
#include "userconfig.hpp"

#if defined(_VERMIL_STM32F4) && !_VERMIL_STM32_USE_GENERIC

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
        volatile uint32_t ACR;      /*!< FLASH access control register,   Address offset: 0x00 */
        volatile uint32_t KEYR;     /*!< FLASH key register,              Address offset: 0x04 */
        volatile uint32_t OPTKEYR;  /*!< FLASH option key register,       Address offset: 0x08 */
        volatile uint32_t SR;       /*!< FLASH status register,           Address offset: 0x0C */
        volatile uint32_t CR;       /*!< FLASH control register,          Address offset: 0x10 */
        volatile uint32_t OPTCR;    /*!< FLASH option control register ,  Address offset: 0x14 */
        volatile uint32_t OPTCR1;   /*!< FLASH option control register 1, Address offset: 0x18 */
    };

    extern FlashRegister &flash_reg;
}

}
}
}

#endif
