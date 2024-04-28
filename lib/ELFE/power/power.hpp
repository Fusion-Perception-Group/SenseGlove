#pragma once

#include "_hpp_config.hpp"
#include "clock/rcc.hpp"
#include <concepts>
#include <cstdint>
#include <functional>
#include <iterator>
#include <ranges>
#include <stdexcept>
#include <utility>

namespace elfe {
namespace stm32 {
    namespace power {
        namespace detail {
            struct Register {
                volatile uint32_t CR; /*!< PWR power control register,        Address offset: 0x00 */
                volatile uint32_t CSR; /*!< PWR power control/status register, Address offset: 0x04 */
#ifdef _ELFE_STM32HX
                volatile uint32_t CR2; /*!< PWR power control register 2,            Address offset: 0x08 */
                volatile uint32_t CR3; /*!< PWR power control register 3,            Address offset: 0x0C */
                volatile uint32_t CPUCR; /*!< PWR CPU control register,                Address offset: 0x10 */
                uint32_t RESERVED0; /*!< Reserved,                                Address offset: 0x14 */
                volatile uint32_t D3CR; /*!< PWR D3 domain control register,          Address offset: 0x18 */
                uint32_t RESERVED1; /*!< Reserved,                                Address offset: 0x1C */
                volatile uint32_t WKUPCR; /*!< PWR wakeup clear register,               Address offset: 0x20 */
                volatile uint32_t WKUPFR; /*!< PWR wakeup flag register,                Address offset: 0x24 */
                volatile uint32_t WKUPEPR; /*!< PWR wakeup enable and polarity register, Address offset: 0x28 */
#endif
            };

            extern Register& reg;
        }

        enum class SleepMode {
            Sleep, // stop clock, but keep SRAM and registers
            Stop, // stop clock, keep SRAM, but reset registers
            Standby // stop clock, reset SRAM and registers
        };

        inline void init() noexcept
        {
            clock::rcc::enable_pwr_clock();
        }

        inline void deinit() noexcept
        {
            clock::rcc::disable_pwr_clock();
        }

        inline void wait_on_interrupt() noexcept
        {
            asm volatile("wfi");
        }

        inline void wait_on_event() noexcept
        {
            asm volatile("wfe");
        }

        /**
         * @brief alias of wait_on_interrupt
         *
         */
        inline void sleep() noexcept
        {
            asm volatile("wfi");
        }

        inline void set_backup_protection(bool enable) noexcept
        {
            const uint32_t mask = 1 << 8;
            detail::reg.CR = (detail::reg.CR & ~mask) | (not enable ? mask : 0);
        }

        inline bool is_backup_protection_enabled() noexcept
        {
            const uint32_t mask = 1 << 8;
            return (detail::reg.CR & mask) != 0;
        }

        inline void set_backup_regulator(bool enable) noexcept
        {
            const uint32_t mask = 1 << 9;
            detail::reg.CSR = (detail::reg.CSR & ~mask) | (enable ? mask : 0);
        }

        inline bool is_backup_regulator_enabled() noexcept
        {
            const uint32_t mask = 1 << 9;
            return (detail::reg.CSR & mask) != 0;
        }

        inline bool is_backup_regulator_ready() noexcept
        {
            const uint32_t mask = 1 << 3;
            return (detail::reg.CSR & mask) != 0;
        }

        inline bool was_in_standby() noexcept
        {
            const uint32_t mask = 1 << 1;
            return (detail::reg.CSR & mask) != 0;
        }

        inline void clear_in_standby() noexcept
        {
            const uint32_t mask = 1 << 3;
            detail::reg.CR |= mask;
        }

        inline bool was_waked_up() noexcept
        {
            const uint32_t mask = 1;
            return (detail::reg.CSR & mask) != 0;
        }

        inline void clear_waked_up() noexcept
        {
            const uint32_t mask = 1 << 2;
            detail::reg.CR |= mask;
        }

        inline void config_standby_mode(bool enable, bool use_low_voltage_regulator)
        {
            const uint32_t PDDS_MASK = 1 << 1;
            const uint32_t LPDS_MASK = 1 << 0;
            detail::reg.CR = (detail::reg.CR & ~(PDDS_MASK | LPDS_MASK)) | (enable ? PDDS_MASK : 0) | (use_low_voltage_regulator ? LPDS_MASK : 0);
        }

        void set_sleepmode(SleepMode mode) noexcept;
        SleepMode get_sleepmode() noexcept;

    }
}
}