#include "power/power.hpp"
#include "./_cpp_config.hpp"

namespace elfe {
namespace stm32 {
    namespace power {
        namespace detail {
            Register& reg = *reinterpret_cast<Register*>(PWR_BASE);
        }

        void set_sleepmode(SleepMode mode) noexcept
        {
            config_standby_mode(false, false);
            SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
            switch (mode) {
            case SleepMode::Standby:
                config_standby_mode(true, true);
            case SleepMode::Stop:
                SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
            case SleepMode::Sleep:
                break;
            default:
                break;
            }
        }

        SleepMode get_sleepmode() noexcept
        {
            const uint32_t PDDS_MASK = 1 << 1;
            const uint32_t SLEEPDEEP_MASK = SCB_SCR_SLEEPDEEP_Msk;
            uint32_t cr = detail::reg.CR;

            if (SCB->SCR & SLEEPDEEP_MASK) {
                if (cr & PDDS_MASK) {
                    return SleepMode::Standby;
                }
                return SleepMode::Stop;
            }
            return SleepMode::Sleep;
        }

    }
}
}
