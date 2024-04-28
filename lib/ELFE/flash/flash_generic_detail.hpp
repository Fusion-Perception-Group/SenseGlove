#pragma once

#include "_hpp_config.hpp"
#include <cstdint>

#if _ELFE_STM32_USE_GENERIC

namespace elfe {
namespace stm32 {
    namespace flash {

        namespace detail {
            struct FlashRegister {
                volatile uint32_t ACR;
            };

            inline FlashRegister flash_reg;
        }

    }
}
}

#endif