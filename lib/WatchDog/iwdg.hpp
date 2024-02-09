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
#include "property.hpp"
#include "userconfig.hpp"

namespace vms
{
namespace stm32
{
namespace wdg
{

namespace detail
{
    struct IWDGRegister
    {
        volatile uint32_t KR;   /*!< IWDG Key register,       Address offset: 0x00 */
        volatile uint32_t PR;   /*!< IWDG Prescaler register, Address offset: 0x04 */
        volatile uint32_t RLR;  /*!< IWDG Reload register,    Address offset: 0x08 */
        volatile uint32_t SR;   /*!< IWDG Status register,    Address offset: 0x0C */
        volatile uint32_t WINR; /*!< IWDG Window register,    Address offset: 0x10 */
    };

    extern IWDGRegister &iwdg_reg;
}

enum class IWDGPrescaler : uint8_t
{
    Div4 = 0,
    Div8 = 1,
    Div16 = 2,
    Div32 = 3,
    Div64 = 4,
    Div128 = 5,
    Div256 = 6
};

class IndependentWatchDog
{
public:
    detail::IWDGRegister &reg = detail::iwdg_reg;

    void set_prescaler(IWDGPrescaler prescaler) const noexcept
    {
        reg.KR = 0x5555;
        reg.PR = static_cast<uint32_t>(prescaler);
        lock();
    }

    IWDGPrescaler get_prescaler() const noexcept
    {
        return static_cast<IWDGPrescaler>(reg.PR);
    }

    /**
     * @brief Set the reload value
     * 
     * @param reload 
     * @throw std::invalid_argument if reload is greater than 0xFFF
     */
    void set_reload(uint16_t reload) const
    {
        reg.KR = 0x5555;
        if (reload > 0xFFF)
            throw std::invalid_argument("reload value must be less than 0xFFF");
        reg.RLR = reload;
        lock();
    }

    uint16_t get_reload() const noexcept
    {
        return reg.RLR;
    }

    /**
     * @brief Set the max hungry time
     * 
     * @param us : max hungry time in microseconds
     * @throw std::invalid_argument if max hungry time is too long
     */
    void set_max_hungry_time(std::chrono::microseconds us) const
    {
        const uint32_t LSI_CLK = clock::rcc::get_lsi_clock();
        if (us.count() / 256 / LSI_CLK > 0xFFF)
            throw std::invalid_argument("max hungry time is too long");
        uint64_t reload = static_cast<uint64_t>(us.count()) * LSI_CLK / 1000000;
        unsigned prescaler = 0;
        reload /= 2;
        while ((reload /= 2) > 0xFFFU)
            ++prescaler;

        reg.KR = 0x5555;
        reg.RLR = reload;
        reg.PR = prescaler;
        lock();
    }

    /**
     * @brief Get the max hungry time
     * 
     * @return std::chrono::microseconds 
     */
    std::chrono::microseconds get_max_hungry_time() const noexcept
    {
        const uint32_t LSI_CLK = clock::rcc::get_lsi_clock();
        uint64_t reload = reg.RLR;
        unsigned prescaler = reg.PR;
        return std::chrono::microseconds(reload * (1 << (2+prescaler)) * 1000000 / LSI_CLK);
    }

    void lock() const noexcept
    {
        reg.KR = 0xFFFF;
    }

    void feed() const noexcept
    {
        reg.KR = 0xAAAA;
    }

    void start() const noexcept
    {
        reg.KR = 0xCCCC;
    }

    bool is_reload_updating() const noexcept
    {
        return reg.SR & 2U;
    }

    bool is_prescaler_updating() const noexcept
    {
        return reg.SR & 1U;
    }

    bool was_reset_by_me() const noexcept
    {
        return clock::rcc::detail::RCCReg.CSR & (1U << 29);
    }
};

}
}
}