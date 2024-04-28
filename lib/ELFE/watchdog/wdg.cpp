#include "watchdog/wdg.hpp"
#include "./_cpp_config.hpp"

namespace elfe {
namespace stm32 {
    namespace wdg {

        namespace detail {
            IWDGRegister& iwdg_reg = *reinterpret_cast<IWDGRegister*>(IWDG_BASE);
            WWDGRegister& wwdg_reg = *reinterpret_cast<WWDGRegister*>(WWDG_BASE);
        }

        extern "C" {
        void WWDG_IRQHandler() noexcept
        {
            detail::wwdg_reg.SR &= ~WWDG_SR_EWIF;
            if (detail::on_warning)
                detail::on_warning();
        }
        }

    }
}
}