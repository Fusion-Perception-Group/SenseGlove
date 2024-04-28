#include "./_cpp_config.hpp"
#include "clock/rcc.hpp"

#if defined(_ELFE_STM32F1) && !_ELFE_STM32_USE_GENERIC

namespace elfe {
namespace stm32 {
    namespace clock {
        namespace rcc {

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
