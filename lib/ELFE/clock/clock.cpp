#include "clock/clock.hpp"
#include "./_cpp_config.hpp"
#include "nvic/nvic.hpp"
#include <utility>

#define weak __attribute__((weak))
namespace elfe {
namespace stm32 {
    namespace clock {
        uint32_t& SystemCoreClock = ::SystemCoreClock;
        static volatile uint64_t ticks_high = 0;

        static inline void increase_tick()
        {
            ticks_high = ticks_high + 1U + SysTick->LOAD;
        }

        VoidResult<> init_systick(const uint32_t reload_value, const uint8_t interrupt_prior)
        {
            SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
            SysTick->LOAD = std::min(reload_value, SysTick_LOAD_RELOAD_Msk); /* set reload register */
            SysTick->VAL = 0UL; /* Load the SysTick Counter Value */
            SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
            auto r = nvic::set_priority(nvic::SysTick_IRQn, interrupt_prior);
            ELFE_PROP(r, r);
            nvic::enable_irq(nvic::SysTick_IRQn);
            ticks_high = 0;
            return EC::None;
        }

        uint64_t get_systick() noexcept
        {
            return ticks_high + (SysTick_LOAD_RELOAD_Msk - SysTick->VAL);
        }

        extern "C" {
        void SysTick_Handler()
        {
            increase_tick();
        }

        uint32_t HAL_GetTick()
        {
            return get_systick_ms();
        }
        }
    }
}
}