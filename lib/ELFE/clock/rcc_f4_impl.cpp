#include "./_cpp_config.hpp"
#include "clock/rcc.hpp"

#if defined(_ELFE_STM32F4) && !_ELFE_STM32_USE_GENERIC

namespace elfe {
namespace stm32 {
    namespace clock {
        namespace rcc {
            namespace detail {
#ifdef RCC_BASE
                RCCRegister& RCCReg = *reinterpret_cast<RCCRegister*>(RCC_BASE);
#endif
            }

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
