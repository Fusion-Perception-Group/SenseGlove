#pragma once

#include "result.hpp"
#include "clock/clock_shared.hpp"
#include "clock/rcc.hpp"
#include "clock/rtc.hpp"
#include "clock/tim.hpp"
#include "utils/property.hpp"
#include <chrono>
#include <concepts>
#include <cstdint>
#include <functional>
#include <ratio>
#include <type_traits>

namespace elfe {
namespace stm32 {
    namespace clock {
        using EC = err::ErrorCode;
        using rcc::disable_clock;
        using rcc::enable_clock;

        using nano = std::ratio<1, 1000000000ULL>;
        using micro = std::ratio<1, 1000000ULL>;
        using milli = std::ratio<1, 1000ULL>;
        using centi = std::ratio<1, 100ULL>;

        template <typename T>
        concept Timestamp = std::totally_ordered<T>;

        template <Timestamp T>
        class TimeoutChecker {
        protected:
            using StampGetterType = std::function<T()>;
            StampGetterType _stamp_getter;

        public:
            T target;
            TimeoutChecker(StampGetterType stamp_getter, T stamp) noexcept
                : _stamp_getter(stamp_getter)
                , target(stamp)
            {
            }

            template <typename C>
                requires Timestamp<decltype(std::declval<C>().get_timestamp())>
            TimeoutChecker(C& clock, T stamp) noexcept
                : _stamp_getter([&clock]() -> T { return clock.get_timestamp(); })
                , target(stamp)
            {
            }

            TimeoutChecker<T>& operator=(const TimeoutChecker<T>& other) noexcept
            {
                _stamp_getter = other._stamp_getter;
                target = other.target;
                return *this;
            }
            TimeoutChecker<T>& operator=(TimeoutChecker<T>&& other) noexcept
            {
                _stamp_getter = std::move(other._stamp_getter);
                target = std::move(other.target);
                return *this;
            }

            virtual ~TimeoutChecker() = default;

            bool has_timedout() const noexcept
            {
                return _stamp_getter() >= target;
            }

            template <typename EXC_T = TimeoutError, typename... ARG_T>
                requires std::is_constructible_v<EXC_T, ARG_T...>
            VoidResult<> raise_if_timedout(ARG_T&&... args) const
            {
                ELFE_ERROR_IF(has_timedout(), EC::TimeoutError, EXC_T(std::forward<ARG_T>(args)...));
                return EC::None;
            }

            operator bool() const noexcept
            {
                return has_timedout();
            }
        };

        VoidResult<> init_systick(const uint32_t reload_value = 0xFFFFFFFF, const uint8_t interrupt_prior = 0);
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

        /**
         * @brief delay
         *
         * @param delay when passed as `uint64_t`, it is interpreted as microseconds
         */
        inline void delay(uint64_t delay_us) noexcept
        {
            const uint64_t target = (delay_us * SystemCoreClock / 1000000ULL) + get_systick();
            while (get_systick() < target)
                ;
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
