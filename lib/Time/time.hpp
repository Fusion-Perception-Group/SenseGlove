#pragma once

#include <cstdint>
#include <chrono>
#include "trace.hpp"
#include "userconfig.hpp"
#include "clock.hpp"
namespace vermils
{
namespace stm32
{
namespace time
{
    class DWTTimer
    {
        DataWatchpointTrigger dwt = {};
    public:
        DWTTimer() 
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

        inline void delay(const std::chrono::duration<uint32_t, std::nano> &ns) const
        {
            delay_ns(ns.count());
        }

        inline void delay_ms(const uint32_t ms) const
        {
            uint_fast32_t start = dwt.CYCCNT;
            uint_fast32_t clocks = ms * (SystemCoreClock / 1000U);
            while (dwt.CYCCNT - start < clocks);
        }

        inline void delay_us(const uint32_t us) const
        {
            uint_fast32_t start = dwt.CYCCNT;
            uint_fast32_t clocks = us * (SystemCoreClock / 1000000U);
            while (dwt.CYCCNT - start < clocks);
        }

        inline void delay_ns(const uint32_t ns) const
        {
            uint_fast32_t start = dwt.CYCCNT;
            uint_fast32_t clocks = ns * (SystemCoreClock / 1000000000U);
            while (dwt.CYCCNT - start < clocks);
        }

        inline void delay_clks(const uint32_t clks) const
        {
            uint_fast32_t start = dwt.CYCCNT;
            while (dwt.CYCCNT - start < clks);
        }

        inline uint32_t get_cycles() const
        {
            return dwt.CYCCNT;
        }

        inline uint32_t get_ns() const
        {
            return dwt.CYCCNT / (SystemCoreClock / 1000000000U);
        }

        inline uint32_t get_us() const
        {
            return dwt.CYCCNT / (SystemCoreClock / 1000000U);
        }

        inline uint32_t get_ms() const
        {
            return dwt.CYCCNT / (SystemCoreClock / 1000U);
        }
    };

    #if defined(__VERMIL_STM32HX)
    // not implemented
    #else
    using HighResTimer = DWTTimer;
    #endif
}
}
}
