#pragma once

#include <cstdint>
#include <stdexcept>
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
    class DurationExceeded : public std::invalid_argument
    {
    public:
        DurationExceeded(const char *msg) : std::invalid_argument(msg) {}
    };
    
    class BaseTimer
    {
    public:
        virtual ~BaseTimer() = default;
        /**
         * @brief delay a duration
         * 
         * @param duration
         */
        virtual void delay(const uint64_t) const noexcept = 0;
        virtual uint64_t now() const noexcept = 0;
    };

    class DWTTimer
    {
        dwt::DataWatchpointTrigger dwt = {};
    public:
        DWTTimer() 
        {
            dwt::enable_trace();
            dwt.enable();
        }

        bool is_alive() const
        {
            uint32_t start = dwt.CYCCNT;
            asm("NOP");
            asm("NOP");
            asm("NOP");
            return bool(dwt.CYCCNT - start);
        }

        inline void delay(const std::chrono::nanoseconds &ns) const noexcept
        {
            delay_ns(ns.count());
        }

        inline uint64_t now() const noexcept
        {
            return get_ns();
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

        inline void delay_ns(const uint64_t ns) const
        {
            uint_fast32_t start = dwt.CYCCNT;
            uint_fast32_t clocks = ns * SystemCoreClock / 1000000000U;
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

    #if defined(_VERMIL_STM32HX)
    // not implemented
    #else
    using HighResTimer = DWTTimer;
    #endif
}
}
}
