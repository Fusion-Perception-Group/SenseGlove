#include "rtc.hpp"
#include "_config.hpp"
#include <ctime>
#include <sys/time.h>

namespace vermils
{
namespace stm32
{
namespace clock
{
namespace rtc
{
    namespace detail
    {
        Register &reg = *reinterpret_cast<Register *>(RTC_BASE);
    }

static RTC_HandleTypeDef rtc_handle{
    .Instance = RTC,
    .Init = {
        .HourFormat = RTC_HOURFORMAT_24,
        .AsynchPrediv = 0x7F,
        .SynchPrediv = 0xFF,
        .OutPut = RTC_OUTPUT_DISABLE,
        .OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH,
        .OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN
    }
};

static inline time_t get_unix_timestamp_ms() noexcept
{
    RTC_DateTypeDef dt;
    HAL_RTC_GetDate(&rtc_handle, &dt, RTC_FORMAT_BIN);
    RTC_TimeTypeDef tt;
    HAL_RTC_GetTime(&rtc_handle, &tt, RTC_FORMAT_BIN);

    tm t{
        .tm_sec = tt.Seconds,
        .tm_min = tt.Minutes,
        .tm_hour = tt.Hours,
        .tm_mday = dt.Date,
        .tm_mon = dt.Month,
        .tm_year = dt.Year,
        .tm_wday = dt.WeekDay,
        .tm_isdst = 0
    };
    return mktime(&t)*1000 + (1000*(rtc_handle.Init.SynchPrediv - tt.SubSeconds)/(rtc_handle.Init.SynchPrediv+1));
}

void init() noexcept
{
    power::init();
    set_backup_protection(false);
    clock::rcc::enable_rtc_clock();
    set_backup_protection(true);
    HAL_RTC_Init(&rtc_handle);
}

void set_clock_source(ClockSource source) noexcept
{
    RCC->BDCR &= ~RCC_BDCR_RTCSEL;
    RCC->BDCR |= static_cast<uint32_t>(source) << RCC_BDCR_RTCSEL_Pos;
}

ClockSource get_clock_source() noexcept
{
    return static_cast<ClockSource>((RCC->BDCR & RCC_BDCR_RTCSEL) >> RCC_BDCR_RTCSEL_Pos);
}

extern "C"
{
    #define weak __attribute__((weak))
    weak int gettimeofday(struct timeval *tv, void *tzvp)
    {
        auto ms = get_unix_timestamp_ms();
        tv->tv_sec = ms/1000;
        tv->tv_usec = ms%1000;
        return 0;
    }
}

}
}
}
}