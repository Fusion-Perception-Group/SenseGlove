#include "clock.hpp"
#include "nvic.hpp"
#include "_config.hpp"

#define weak __attribute__((weak))
namespace vermils
{
namespace stm32
{
namespace clock
{
uint32_t & SystemCoreClock = ::SystemCoreClock;
static uint_fast64_t ticks_high=0;

static void increase_tick()
{
    ticks_high += 1U + SysTick_LOAD_RELOAD_Msk;
}

void init_systick() noexcept
{
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    SysTick->LOAD  = SysTick_LOAD_RELOAD_Msk;/* set reload register */
    SysTick->VAL   = 0UL;                     /* Load the SysTick Counter Value */
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk   |
                    SysTick_CTRL_ENABLE_Msk;
    nvic::enable_irq(nvic::SysTick_IRQn);
    nvic::set_priority(nvic::SysTick_IRQn, 15);
    on_systick = increase_tick;
    ticks_high = 0;
}

uint64_t get_systick() noexcept
{
    return ticks_high + (SysTick_LOAD_RELOAD_Msk - SysTick->VAL);
}

extern "C"
{
    void SysTick_Handler()
    {
        //HAL_IncTick();
        if (on_systick)
           on_systick();
    }

    uint32_t HAL_GetTick()
    {
        return get_systick_ms();
    }
}
}
}
}