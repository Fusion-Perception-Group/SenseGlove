#pragma once

#include "_hpp_config.hpp"
#include <cstdint>

#if defined(_ELFE_STM32F1) && !_ELFE_STM32_USE_GENERIC

namespace elfe {
namespace stm32 {
    namespace flash {

        namespace detail {
            struct FlashRegister {
                volatile uint32_t ACR;
                volatile uint32_t KEYR;
                volatile uint32_t OPTKEYR;
                volatile uint32_t SR;
                volatile uint32_t CR;
                volatile uint32_t AR;
                volatile uint32_t RESERVED;
                volatile uint32_t OBR;
                volatile uint32_t WRPR;
            };

            extern FlashRegister& flash_reg;
        }

    }
}
}

#endif
