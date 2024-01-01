#pragma once

#include <cstdint>
#include <chrono>
#include "userconfig.hpp"
#include "clock_shared.hpp"
#include "power.hpp"

namespace vermils
{
namespace stm32
{
namespace clock
{
namespace rtc
{
using power::set_backup_protection;
    namespace detail
    {
        struct Register
        {
            #if defined(_VERMIL_STM32F4)
            volatile uint32_t TR;      /*!< RTC time register,                                        Address offset: 0x00 */
            volatile uint32_t DR;      /*!< RTC date register,                                        Address offset: 0x04 */
            volatile uint32_t CR;      /*!< RTC control register,                                     Address offset: 0x08 */
            volatile uint32_t ISR;     /*!< RTC initialization and status register,                   Address offset: 0x0C */
            volatile uint32_t PRER;    /*!< RTC prescaler register,                                   Address offset: 0x10 */
            volatile uint32_t WUTR;    /*!< RTC wakeup timer register,                                Address offset: 0x14 */
            volatile uint32_t CALIBR;  /*!< RTC calibration register,                                 Address offset: 0x18 */
            volatile uint32_t ALRMAR;  /*!< RTC alarm A register,                                     Address offset: 0x1C */
            volatile uint32_t ALRMBR;  /*!< RTC alarm B register,                                     Address offset: 0x20 */
            volatile uint32_t WPR;     /*!< RTC write protection register,                            Address offset: 0x24 */
            volatile uint32_t SSR;     /*!< RTC sub second register,                                  Address offset: 0x28 */
            volatile uint32_t SHIFTR;  /*!< RTC shift control register,                               Address offset: 0x2C */
            volatile uint32_t TSTR;    /*!< RTC time stamp time register,                             Address offset: 0x30 */
            volatile uint32_t TSDR;    /*!< RTC time stamp date register,                             Address offset: 0x34 */
            volatile uint32_t TSSSR;   /*!< RTC time-stamp sub second register,                       Address offset: 0x38 */
            volatile uint32_t CALR;    /*!< RTC calibration register,                                 Address offset: 0x3C */
            volatile uint32_t TAFCR;   /*!< RTC tamper and alternate function configuration register, Address offset: 0x40 */
            volatile uint32_t ALRMASSR;/*!< RTC alarm A sub second register,                          Address offset: 0x44 */
            volatile uint32_t ALRMBSSR;/*!< RTC alarm B sub second register,                          Address offset: 0x48 */
            uint32_t RESERVED7;    /*!< Reserved, 0x4C                                                                 */
            volatile uint32_t BKP0R;   /*!< RTC backup register 1,                                    Address offset: 0x50 */
            volatile uint32_t BKP1R;   /*!< RTC backup register 1,                                    Address offset: 0x54 */
            volatile uint32_t BKP2R;   /*!< RTC backup register 2,                                    Address offset: 0x58 */
            volatile uint32_t BKP3R;   /*!< RTC backup register 3,                                    Address offset: 0x5C */
            volatile uint32_t BKP4R;   /*!< RTC backup register 4,                                    Address offset: 0x60 */
            volatile uint32_t BKP5R;   /*!< RTC backup register 5,                                    Address offset: 0x64 */
            volatile uint32_t BKP6R;   /*!< RTC backup register 6,                                    Address offset: 0x68 */
            volatile uint32_t BKP7R;   /*!< RTC backup register 7,                                    Address offset: 0x6C */
            volatile uint32_t BKP8R;   /*!< RTC backup register 8,                                    Address offset: 0x70 */
            volatile uint32_t BKP9R;   /*!< RTC backup register 9,                                    Address offset: 0x74 */
            volatile uint32_t BKP10R;  /*!< RTC backup register 10,                                   Address offset: 0x78 */
            volatile uint32_t BKP11R;  /*!< RTC backup register 11,                                   Address offset: 0x7C */
            volatile uint32_t BKP12R;  /*!< RTC backup register 12,                                   Address offset: 0x80 */
            volatile uint32_t BKP13R;  /*!< RTC backup register 13,                                   Address offset: 0x84 */
            volatile uint32_t BKP14R;  /*!< RTC backup register 14,                                   Address offset: 0x88 */
            volatile uint32_t BKP15R;  /*!< RTC backup register 15,                                   Address offset: 0x8C */
            volatile uint32_t BKP16R;  /*!< RTC backup register 16,                                   Address offset: 0x90 */
            volatile uint32_t BKP17R;  /*!< RTC backup register 17,                                   Address offset: 0x94 */
            volatile uint32_t BKP18R;  /*!< RTC backup register 18,                                   Address offset: 0x98 */
            volatile uint32_t BKP19R;  /*!< RTC backup register 19,                                   Address offset: 0x9C */
            #elif defined(_VERMIL_STM32F1)
            volatile uint32_t CRH;
            volatile uint32_t CRL;
            volatile uint32_t PRLH;
            volatile uint32_t PRLL;
            volatile uint32_t DIVH;
            volatile uint32_t DIVL;
            volatile uint32_t CNTH;
            volatile uint32_t CNTL;
            volatile uint32_t ALRH;
            volatile uint32_t ALRL;
            #elif defined(_VERMIL_STM32H7)
            volatile uint32_t TR;         /*!< RTC time register,                                         Address offset: 0x00 */
            volatile uint32_t DR;         /*!< RTC date register,                                         Address offset: 0x04 */
            volatile uint32_t CR;         /*!< RTC control register,                                      Address offset: 0x08 */
            volatile uint32_t ISR;        /*!< RTC initialization and status register,                    Address offset: 0x0C */
            volatile uint32_t PRER;       /*!< RTC prescaler register,                                    Address offset: 0x10 */
            volatile uint32_t WUTR;       /*!< RTC wakeup timer register,                                 Address offset: 0x14 */
                uint32_t RESERVED;   /*!< Reserved,                                                  Address offset: 0x18 */
            volatile uint32_t ALRMAR;     /*!< RTC alarm A register,                                      Address offset: 0x1C */
            volatile uint32_t ALRMBR;     /*!< RTC alarm B register,                                      Address offset: 0x20 */
            volatile uint32_t WPR;        /*!< RTC write protection register,                             Address offset: 0x24 */
            volatile uint32_t SSR;        /*!< RTC sub second register,                                   Address offset: 0x28 */
            volatile uint32_t SHIFTR;     /*!< RTC shift control register,                                Address offset: 0x2C */
            volatile uint32_t TSTR;       /*!< RTC time stamp time register,                              Address offset: 0x30 */
            volatile uint32_t TSDR;       /*!< RTC time stamp date register,                              Address offset: 0x34 */
            volatile uint32_t TSSSR;      /*!< RTC time-stamp sub second register,                        Address offset: 0x38 */
            volatile uint32_t CALR;       /*!< RTC calibration register,                                  Address offset: 0x3C */
            volatile uint32_t TAMPCR;     /*!< RTC tamper configuration register,                         Address offset: 0x40 */
            volatile uint32_t ALRMASSR;   /*!< RTC alarm A sub second register,                           Address offset: 0x44 */
            volatile uint32_t ALRMBSSR;   /*!< RTC alarm B sub second register,                           Address offset: 0x48 */
            volatile uint32_t OR;         /*!< RTC option register,                                       Address offset: 0x4C */
            volatile uint32_t BKP0R;      /*!< RTC backup register 0,                                     Address offset: 0x50 */
            volatile uint32_t BKP1R;      /*!< RTC backup register 1,                                     Address offset: 0x54 */
            volatile uint32_t BKP2R;      /*!< RTC backup register 2,                                     Address offset: 0x58 */
            volatile uint32_t BKP3R;      /*!< RTC backup register 3,                                     Address offset: 0x5C */
            volatile uint32_t BKP4R;      /*!< RTC backup register 4,                                     Address offset: 0x60 */
            volatile uint32_t BKP5R;      /*!< RTC backup register 5,                                     Address offset: 0x64 */
            volatile uint32_t BKP6R;      /*!< RTC backup register 6,                                     Address offset: 0x68 */
            volatile uint32_t BKP7R;      /*!< RTC backup register 7,                                     Address offset: 0x6C */
            volatile uint32_t BKP8R;      /*!< RTC backup register 8,                                     Address offset: 0x70 */
            volatile uint32_t BKP9R;      /*!< RTC backup register 9,                                     Address offset: 0x74 */
            volatile uint32_t BKP10R;     /*!< RTC backup register 10,                                    Address offset: 0x78 */
            volatile uint32_t BKP11R;     /*!< RTC backup register 11,                                    Address offset: 0x7C */
            volatile uint32_t BKP12R;     /*!< RTC backup register 12,                                    Address offset: 0x80 */
            volatile uint32_t BKP13R;     /*!< RTC backup register 13,                                    Address offset: 0x84 */
            volatile uint32_t BKP14R;     /*!< RTC backup register 14,                                    Address offset: 0x88 */
            volatile uint32_t BKP15R;     /*!< RTC backup register 15,                                    Address offset: 0x8C */
            volatile uint32_t BKP16R;     /*!< RTC backup register 16,                                    Address offset: 0x90 */
            volatile uint32_t BKP17R;     /*!< RTC backup register 17,                                    Address offset: 0x94 */
            volatile uint32_t BKP18R;     /*!< RTC backup register 18,                                    Address offset: 0x98 */
            volatile uint32_t BKP19R;     /*!< RTC backup register 19,                                    Address offset: 0x9C */
            volatile uint32_t BKP20R;     /*!< RTC backup register 20,                                    Address offset: 0xA0 */
            volatile uint32_t BKP21R;     /*!< RTC backup register 21,                                    Address offset: 0xA4 */
            volatile uint32_t BKP22R;     /*!< RTC backup register 22,                                    Address offset: 0xA8 */
            volatile uint32_t BKP23R;     /*!< RTC backup register 23,                                    Address offset: 0xAC */
            volatile uint32_t BKP24R;     /*!< RTC backup register 24,                                    Address offset: 0xB0 */
            volatile uint32_t BKP25R;     /*!< RTC backup register 25,                                    Address offset: 0xB4 */
            volatile uint32_t BKP26R;     /*!< RTC backup register 26,                                    Address offset: 0xB8 */
            volatile uint32_t BKP27R;     /*!< RTC backup register 27,                                    Address offset: 0xBC */
            volatile uint32_t BKP28R;     /*!< RTC backup register 28,                                    Address offset: 0xC0 */
            volatile uint32_t BKP29R;     /*!< RTC backup register 29,                                    Address offset: 0xC4 */
            volatile uint32_t BKP30R;     /*!< RTC backup register 30,                                    Address offset: 0xC8 */
            volatile uint32_t BKP31R;     /*!< RTC backup register 31,                                    Address offset: 0xCC */
            #endif
        };
        extern Register & reg;
    }

enum class ClockSource
{
    None = 0,
    LSE,
    LSI,
    HSE
};

void init() noexcept;

inline void deinit() noexcept
{
    set_backup_protection(false);
    clock::rcc::disable_rtc_clock();
    set_backup_protection(true);
    power::deinit();
}

inline void unlock() noexcept
{
    #if defined(_VERMIL_STM32F4) || defined(_VERMIL_STM32HX)
    detail::reg.WPR = 0xCA;
    detail::reg.WPR = 0x53;
    #endif
}

inline void lock() noexcept
{
    #if defined(_VERMIL_STM32F4) || defined(_VERMIL_STM32HX)
    detail::reg.WPR = 0xFF;
    #endif
}

void set_clock_source(ClockSource source) noexcept;
ClockSource get_clock_source() noexcept;

}
}
}
}