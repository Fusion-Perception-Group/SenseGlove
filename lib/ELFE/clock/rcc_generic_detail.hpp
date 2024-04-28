#pragma once

#include "_hpp_config.hpp"
#include <cstdint>

#if _ELFE_STM32_USE_GENERIC

namespace elfe {
namespace stm32 {
    namespace clock {
        namespace rcc {

            namespace detail {
                struct RCCRegister {
                };

                inline RCCRegister RCCReg;
            }

        }
    }
}
}

#endif
