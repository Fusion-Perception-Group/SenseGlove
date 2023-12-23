#pragma once

#include <cstdint>
#include <exception>
#include <functional>

namespace vermils
{
namespace stm32
{
namespace clock
{
    extern uint32_t & SystemCoreClock;
    inline consteval uint64_t operator ""_Hz(uint64_t hz) noexcept
    {
        return hz;
    }

    inline consteval uint64_t operator ""_KHz(uint64_t khz) noexcept
    {
        return khz * 1000;
    }

    inline consteval uint64_t operator ""_MHz(uint64_t mhz) noexcept
    {
        return mhz * 1000000;
    }

    class TimeoutError : public std::exception
    {
    public:
        const char *what() const noexcept override
        {
            return "Timeout";
        }
    };
}
using clock::SystemCoreClock;
}
}