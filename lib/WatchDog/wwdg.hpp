#pragma once

#include <cstdint>
#include <stdexcept>
#include <iterator>
#include <ranges>
#include <concepts>
#include <functional>
#include <utility>
#include <chrono>
#include "rcc.hpp"
#include "nvic.hpp"
#include "userconfig.hpp"

namespace vermils
{
namespace stm32
{
namespace wdg
{

namespace detail
{
    struct WWDGRegister
    {
        volatile uint32_t CR;   /*!< WWDG Control register,       Address offset: 0x00 */
        volatile uint32_t CFR;  /*!< WWDG Configuration register, Address offset: 0x04 */
        volatile uint32_t SR;   /*!< WWDG Status register,        Address offset: 0x08 */
    };

    extern WWDGRegister &wwdg_reg;
    using CallbackType = std::function<void()>;
    inline CallbackType on_warning;
}

enum class WWDGPrescaler : uint8_t
{
    Div1 = 0,
    Div2 = 1,
    Div4 = 2,
    Div8 = 3
};

class WindowWatchDog
{
    uint8_t _default_reload;
public:
    detail::WWDGRegister &reg = detail::wwdg_reg;
    detail::CallbackType &on_warning = detail::on_warning;

    WindowWatchDog(uint8_t min, uint8_t max)
    {
        clock::rcc::enable_clock(*this);
        set_default_reload(max);
        set_window(min);
    }

    WindowWatchDog(std::chrono::microseconds min_us, std::chrono::microseconds max_us)
    {
        clock::rcc::enable_clock(*this);
        set_hungry_time_window(min_us, max_us);
    }

    /**
     * @brief Set the default reload value
     * 
     * @param reload 
     * @throw std::invalid_argument if reload is not in range [0x0, 0x3F]
     */
    constexpr void set_default_reload(uint8_t reload)
    {
        if (reload > 0x3F)
            throw std::invalid_argument("reload must be in range [0x0, 0x3F]");
        _default_reload = reload;
    }

    /**
     * @brief Get the default reload value
     * 
     * @return uint8_t 
     */
    constexpr uint8_t get_default_reload() const noexcept
    {
        return _default_reload;
    }

    void set_prescaler(WWDGPrescaler prescaler) const noexcept
    {
        const uint32_t mask = 3U << 7;
        reg.CFR = (reg.CFR & ~mask) | (static_cast<uint32_t>(prescaler) << 7);
    }

    WWDGPrescaler get_prescaler() const noexcept
    {
        return static_cast<WWDGPrescaler>((reg.CFR >> 7) & 3U);
    }

    void set_window(uint8_t window) const
    {
        if (window > 0x3F)
            throw std::invalid_argument("window must be in range [0x0, 0x3F]");
        reg.CFR = (reg.CFR & ~0x7F) | window | 0x40;
    }

    uint8_t get_window() const noexcept
    {
        return reg.CFR & 0x3F;
    }

    /**
     * @brief Set the hungry time window
     * 
     * @param min_us 
     * @param max_us 
     * @throw std::invalid_argument
     */
    void set_hungry_time_window(std::chrono::microseconds min_us, std::chrono::microseconds max_us)
    {
        if (min_us > max_us)
            throw std::invalid_argument("min_us must be less than max_us");
        const uint32_t BASE_CLK = clock::rcc::get_pclk1()/4096;
        uint64_t window = static_cast<uint64_t>(min_us.count()) * BASE_CLK / 1000000;
        uint64_t reload = static_cast<uint64_t>(max_us.count()) * BASE_CLK / 1000000;
        uint8_t prescaler = 0;

        while (reload > 0x3F)
        {
            reload /= 2;
            window /= 2;
            ++prescaler;
        }

        if (prescaler > 3)
            throw std::invalid_argument("hungry time window is too long");
        
        reg.CFR = (reg.CFR & ~0x7F) | window | 0x40;
        reg.CFR = (reg.CFR & ~(3U << 7)) | (prescaler << 7);
        reg.CR = (reg.CR & ~0x7F) | reload;
        _default_reload = reload;
    }

    std::chrono::microseconds get_min_hungry_time() const noexcept
    {
        const uint32_t BASE_CLK = clock::rcc::get_pclk1()/4096;
        uint64_t window = reg.CFR & 0x3F;
        uint8_t prescaler = (reg.CFR >> 7) & 3U;
        return std::chrono::microseconds(window * 1000000 / BASE_CLK * (1 << prescaler));
    }

    std::chrono::microseconds get_max_hungry_time() const noexcept
    {
        const uint32_t BASE_CLK = clock::rcc::get_pclk1()/4096;
        uint64_t reload = reg.CR & 0x3F;
        uint8_t prescaler = (reg.CFR >> 7) & 3U;
        return std::chrono::microseconds(reload * 1000000 / BASE_CLK * (1 << prescaler));
    }

    void feed() const noexcept
    {
        reg.CR = (reg.CR & ~0x7F) | 0x40 | _default_reload;
    }

    /**
     * @brief feed with custom value
     * 
     * @param reload 
     * @throw std::invalid_argument if reload is not in range [0x0, 0x3F]
     */
    void feed(uint8_t reload) const
    {
        if (reload > 0x3F)
            throw std::invalid_argument("reload must be in range [0x0, 0x3F]");
        reg.CR = (reg.CR & ~0x7F) | 0x40 | reload;
    }

    void start() const noexcept
    {
        reg.CFR |= 1U << 7;
    }

    bool was_reset_by_me() const noexcept
    {
        return clock::rcc::detail::RCCReg.CSR & (1U << 30);
    }

    void enable_interrupt() const noexcept
    {
        reg.CFR |= 1U << 9;
        nvic::enable_irq(nvic::WWDG_IRQn);
    }

    void disable_interrupt() const noexcept
    {
        reg.CFR &= ~(1U << 9);
        nvic::disable_irq(nvic::WWDG_IRQn);
    }

    void set_irq_priority(const uint8_t priority=8) const
    {
        nvic::set_priority(nvic::WWDG_IRQn, priority);
    }
};

}
}
}