#pragma once

#include <cstdint>
#include "Trace.hpp"
#include "Clock.hpp"
namespace vermils
{
namespace stm32
{
namespace time
{
    class HighResTimer
    {
        DataWatchpointTrigger dwt = {};
    public:
        HighResTimer() 
        {
            enable_trace();
            dwt.CYCCNTENA = true;
        }

        bool is_alive() const
        {
            uint32_t start = dwt.CYCCNT;
            asm("NOP");
            asm("NOP");
            asm("NOP");
            return bool(dwt.CYCCNT - start);
        }

        void delay_ms(uint32_t ms) const
        {
            uint32_t start = dwt.CYCCNT;
            uint32_t clocks = ms * (SystemCoreClock / 1000);
            while (dwt.CYCCNT - start < clocks);
        }

        void delay_us(uint32_t us) const
        {
            uint32_t start = dwt.CYCCNT;
            uint32_t clocks = us * (SystemCoreClock / 1000000);
            while (dwt.CYCCNT - start < clocks);
        }

        void delay_ns(uint32_t ns) const
        {
            uint32_t start = dwt.CYCCNT;
            uint32_t clocks = ns * (SystemCoreClock / 1000000000);
            while (dwt.CYCCNT - start < clocks);
        }

        void delay_clks(uint32_t clks) const
        {
            uint32_t start = dwt.CYCCNT;
            while (dwt.CYCCNT - start < clks);
        }

        uint32_t get_cycles() const
        {
            return dwt.CYCCNT;
        }

        uint32_t get_ns() const
        {
            return dwt.CYCCNT / (SystemCoreClock / 1000000000);
        }

        uint32_t get_us() const
        {
            return dwt.CYCCNT / (SystemCoreClock / 1000000);
        }

        uint32_t get_ms() const
        {
            return dwt.CYCCNT / (SystemCoreClock / 1000);
        }
    };
}
}
}
