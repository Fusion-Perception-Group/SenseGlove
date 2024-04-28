#pragma once

#include "errors.hpp"
#include <cstdint>
#include <exception>
#include <functional>

namespace elfe {
namespace stm32 {
    namespace clock {
        using err::TimeoutError;
        extern uint32_t& SystemCoreClock;
        inline consteval uint64_t operator""_Hz(uint64_t hz) noexcept
        {
            return hz;
        }

        inline consteval uint64_t operator""_KHz(uint64_t khz) noexcept
        {
            return khz * 1000;
        }

        inline consteval uint64_t operator""_MHz(uint64_t mhz) noexcept
        {
            return mhz * 1000000;
        }
    }
    using clock::SystemCoreClock;
}
}