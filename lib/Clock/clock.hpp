#pragma once

#include <cstdint>
#include <ratio>
#include <concepts>
#include <functional>
#include <chrono>
#include <type_traits>
#include "clock_shared.hpp"
#include "rcc.hpp"
#include "tim.hpp"
#include "property.hpp"

namespace vermils
{
namespace stm32
{
namespace clock
{
    using rcc::enable_clock;
    using rcc::disable_clock;

    using nano = std::ratio<1, 1000000000ULL>;
    using micro = std::ratio<1, 1000000ULL>;
    using milli = std::ratio<1, 1000ULL>;
    using centi = std::ratio<1, 100ULL>;

    template <typename T>
    concept Timestamp = std::totally_ordered<T>;

    template <Timestamp T>
    class TimeoutChecker
    {
    protected:
        using StampGetterType = std::function<T()>;
        StampGetterType _stamp_getter;

    public:
        const T target;

        TimeoutChecker(StampGetterType stamp_getter, T stamp) noexcept
            : _stamp_getter(stamp_getter), target(stamp) {}

        template <typename C>
        requires Timestamp<decltype(std::declval<C>().get_timestamp())>
        TimeoutChecker(C &clock, T stamp) noexcept
            : _stamp_getter([&clock]() -> T { return clock.get_timestamp(); }), target(stamp) {}

        virtual ~TimeoutChecker() = default;

        bool has_timedout() const noexcept {
            return _stamp_getter() >= target;
        }

        template <typename EXC_T = TimeoutError, typename... ARG_T>
        requires std::is_constructible_v<EXC_T, ARG_T...>
        void raise_if_timedout(ARG_T && ...args) const {
            if (has_timedout())
                throw EXC_T(std::forward<ARG_T>(args)...);
        }

        operator bool() const noexcept {
            return has_timedout();
        }
    };


    void init_systick() noexcept;
    uint64_t get_systick() noexcept;
    inline uint64_t get_systick_ns() noexcept
    {
        uint64_t ticks = get_systick();
        return ticks * 1000000000ULL / SystemCoreClock;
    }
    inline uint32_t get_systick_ms() noexcept
    {
        uint64_t ticks = get_systick();
        return ticks * 1000ULL / SystemCoreClock;
    }


    inline void delay(uint64_t delay_us) noexcept
    {
        uint64_t target = (delay_us * SystemCoreClock / 1000000ULL) + get_systick();
        while (get_systick() < target);
    }
    inline void delay(std::chrono::microseconds delay_time) noexcept
    {
        delay(delay_time.count());
    }


    inline TimeoutChecker<uint64_t> make_timeout(uint64_t timeout_us) noexcept
    {
        uint64_t target = (timeout_us * SystemCoreClock / 1000000ULL) + get_systick();
        return TimeoutChecker<uint64_t>(get_systick, target);
    }
    inline TimeoutChecker<uint64_t> make_timeout(std::chrono::microseconds timeout) noexcept
    {
        return make_timeout(timeout.count());
    }
}
}
}
