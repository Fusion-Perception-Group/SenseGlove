#pragma once

#include <stdexcept>
#include <cstdint>
#include "property.hpp"
#include "nvic.hpp"
#include "userconfig.hpp"
#include "block_future.hpp"

namespace vermils
{
namespace stm32
{
namespace adc
{
using CallbackType = std::function<void()>;
const nvic::IRQn_Type ADC_IRQn = nvic::IRQn_Type::ADC_IRQn;
namespace detail
{
    struct _ADCReg
    {
        volatile uint32_t SR;     /*!< ADC status register,                         Address offset: 0x00 */
        volatile uint32_t CR1;    /*!< ADC control register 1,                      Address offset: 0x04 */
        volatile uint32_t CR2;    /*!< ADC control register 2,                      Address offset: 0x08 */
        volatile uint32_t SMPR1;  /*!< ADC sample time register 1,                  Address offset: 0x0C */
        volatile uint32_t SMPR2;  /*!< ADC sample time register 2,                  Address offset: 0x10 */
        volatile uint32_t JOFR1;  /*!< ADC injected channel data offset register 1, Address offset: 0x14 */
        volatile uint32_t JOFR2;  /*!< ADC injected channel data offset register 2, Address offset: 0x18 */
        volatile uint32_t JOFR3;  /*!< ADC injected channel data offset register 3, Address offset: 0x1C */
        volatile uint32_t JOFR4;  /*!< ADC injected channel data offset register 4, Address offset: 0x20 */
        volatile uint32_t HTR;    /*!< ADC watchdog higher threshold register,      Address offset: 0x24 */
        volatile uint32_t LTR;    /*!< ADC watchdog lower threshold register,       Address offset: 0x28 */
        volatile uint32_t SQR1;   /*!< ADC regular sequence register 1,             Address offset: 0x2C */
        volatile uint32_t SQR2;   /*!< ADC regular sequence register 2,             Address offset: 0x30 */
        volatile uint32_t SQR3;   /*!< ADC regular sequence register 3,             Address offset: 0x34 */
        volatile uint32_t JSQR;   /*!< ADC injected sequence register,              Address offset: 0x38*/
        volatile uint32_t JDR1;   /*!< ADC injected data register 1,                Address offset: 0x3C */
        volatile uint32_t JDR2;   /*!< ADC injected data register 2,                Address offset: 0x40 */
        volatile uint32_t JDR3;   /*!< ADC injected data register 3,                Address offset: 0x44 */
        volatile uint32_t JDR4;   /*!< ADC injected data register 4,                Address offset: 0x48 */
        volatile uint32_t DR;     /*!< ADC regular data register,                   Address offset: 0x4C */
    };

    struct _ADCCommonReg
    {
        volatile uint32_t CSR;    /*!< ADC Common status register,                  Address offset: ADC1 base address + 0x300 */
        volatile uint32_t CCR;    /*!< ADC common control register,                 Address offset: ADC1 base address + 0x304 */
        volatile uint32_t CDR;    /*!< ADC common regular data register for dual
                                     AND triple modes,                             Address offset: ADC1 base address + 0x308 */
    };

    extern _ADCReg & ADC1Reg;
    extern _ADCReg & ADC2Reg;
    extern _ADCReg & ADC3Reg;

    extern _ADCCommonReg & ADC1CommonReg;
    extern _ADCCommonReg & ADC2CommonReg;
    extern _ADCCommonReg & ADC3CommonReg;
}

enum class ExternTriggerMode
{
    None = 0,
    RisingEdge = 1,
    FallingEdge = 2,
    BothEdge = 3,
};

enum class SampleCycle
{
    Cycles_3 = 0,
    Cycles_15 = 1,
    Cycles_28 = 2,
    Cycles_56 = 3,
    Cycles_84 = 4,
    Cycles_112 = 5,
    Cycles_144 = 6,
    Cycles_480 = 7,
};

enum class Resolution
{
    Bits_12 = 0,
    Bits_10 = 1,
    Bits_8 = 2,
    Bits_6 = 3,
};

class RegularADC
{
public:
    const uint8_t order;
    detail::_ADCCommonReg &common_reg;
    detail::_ADCReg &reg;
    mutable CallbackType on_regular_done;
    mutable CallbackType on_watchdog;
    mutable CallbackType on_overrun;

    constexpr RegularADC(uint8_t order, detail::_ADCCommonReg &common_reg, detail::_ADCReg &reg) noexcept
        : order(order), common_reg(common_reg), reg(reg) {}

    void init() const noexcept;
    void deinit() const noexcept;

    void start_regular() const noexcept
    {
        reg.CR2 |= 1U << 30;
    }

    bool is_regular_running() const noexcept
    {
        return reg.SR & 0x10U;
    }

    void set_regular_trigger_mode(ExternTriggerMode mode) const noexcept
    {
        reg.CR2 &= ~(0x3U << 28);
        reg.CR2 |= static_cast<uint32_t>(mode) << 28;
    }

    ExternTriggerMode get_regular_trigger_mode() const noexcept
    {
        return static_cast<ExternTriggerMode>((reg.CR2 >> 28) & 0x3U);
    }

    void select_regular_trigger_channel(uint8_t channel) const
    {
        if (channel > 15)
            throw std::invalid_argument("channel out of range");
        reg.CR2 &= ~(0xFU << 24);
    }

    uint8_t get_regular_trigger_channel() const noexcept
    {
        return (reg.CR2 >> 24) & 0xFU;
    }

    void set_left_aligned(bool left_aligned) const noexcept
    {
        if (left_aligned)
            reg.CR2 |= 1U << 11;
        else
            reg.CR2 &= ~(1U << 11);
    }

    bool is_left_aligned() const noexcept
    {
        return (reg.CR2 >> 11) & 1U;
    }

    void set_set_eoc_after_each_conversion(bool set) const noexcept
    {
        if (set)
            reg.CR2 |= 1U << 10;
        else
            reg.CR2 &= ~(1U << 10);
    }

    bool is_set_eoc_after_each_conversion() const noexcept
    {
        return (reg.CR2 >> 10) & 1U;
    }

    void set_dma_disabled(bool disabled) const noexcept  // only work for single ADC
    {
        if (disabled)
            reg.CR2 |= 1U << 9;
        else
            reg.CR2 &= ~(1U << 9);
    }

    bool is_dma_disabled() const noexcept
    {
        return (reg.CR2 >> 9) & 1U;
    }

    void set_dma_mode(bool on) const noexcept
    {
        if (on)
            reg.CR2 |= 1U << 8;
        else
            reg.CR2 &= ~(1U << 8);
    }

    bool is_dma_mode() const noexcept
    {
        return (reg.CR2 >> 8) & 1U;
    }

    void set_continuous(bool on) const noexcept
    {
        if (on)
            reg.CR2 |= 1U << 1;
        else
            reg.CR2 &= ~(1U << 1);
    }

    bool is_continuous() const noexcept
    {
        return (reg.CR2 >> 1) & 1U;
    }

    void set_sample_cycle(SampleCycle cycle, uint8_t channel) const noexcept
    {
        const uint8_t shift = (channel%9) * 3;
        if (channel < 9)
        {
            reg.SMPR2 &= ~(0x7U << shift);
            reg.SMPR2 |= static_cast<uint32_t>(cycle) << shift;
        }
        else
        {
            reg.SMPR1 &= ~(0x7U << shift);
            reg.SMPR1 |= static_cast<uint32_t>(cycle) << shift;
        }
    }

    SampleCycle get_sample_cycle(uint8_t channel) const noexcept
    {
        const uint8_t shift = (channel%9) * 3;
        if (channel < 9)
            return static_cast<SampleCycle>((reg.SMPR2 >> shift) & 0x7U);
        else
            return static_cast<SampleCycle>((reg.SMPR1 >> shift) & 0x7U);
    }

    void set_wdg_hi_thre(uint16_t thre) const
    {
        if (thre > 0xFFFU)
            throw std::invalid_argument("threshold out of range");
        reg.HTR = thre;
    }

    void set_wdg_lo_thre(uint16_t thre) const
    {
        if (thre > 0xFFFU)
            throw std::invalid_argument("threshold out of range");
        reg.LTR = thre;
    }

    void set_regular_sequence_len(uint8_t len) const
    {
        if (len > 16)
            throw std::invalid_argument("sequence length out of range");
        reg.SQR1 &= ~(0xFU << 20);
        reg.SQR1 |= (len - 1) << 20;
    }

    uint8_t get_regular_sequence_len() const noexcept
    {
        return ((reg.SQR1 >> 20) & 0xFU) + 1;
    }

    void set_regular_sequence(uint8_t order, uint8_t channel) const
    {
        if (order > 15)
            throw std::invalid_argument("sequence order out of range");
        if (channel > 17)
            throw std::invalid_argument("channel out of range");
        const uint8_t shift = (order % 6) * 5;
        if (order < 6)
        {
            reg.SQR3 &= ~(0x1FU << shift);
            reg.SQR3 |= channel << shift;
        }
        else if (order < 12)
        {
            reg.SQR2 &= ~(0x1FU << shift);
            reg.SQR2 |= channel << shift;
        }
        else
        {
            reg.SQR1 &= ~(0x1FU << shift);
            reg.SQR1 |= channel << shift;
        }
        set_regular_sequence_len(std::max<uint8_t>(order + 1, get_regular_sequence_len()));
    }

    uint8_t get_regular_sequence(uint8_t order) const
    {
        if (order > 15)
            throw std::invalid_argument("sequence order out of range");
        const uint8_t shift = (order % 6) * 5;
        if (order < 6)
            return (reg.SQR3 >> shift) & 0x1FU;
        else if (order < 12)
            return (reg.SQR2 >> shift) & 0x1FU;
        else
            return (reg.SQR1 >> shift) & 0x1FU;
    }

    uint16_t get_regular_data() const noexcept
    {
        return reg.DR;
    }

    void set_resolution(Resolution res) const noexcept
    {
        reg.CR1 &= ~(0x3U << 24);
        reg.CR1 |= static_cast<uint32_t>(res) << 24;
    }

    Resolution get_resolution() const noexcept
    {
        return static_cast<Resolution>((reg.CR1 >> 24) & 0x3U);
    }

    void set_regular_analog_watchdog(bool on) const noexcept
    {
        if (on)
            reg.CR1 |= 1U << 23;
        else
            reg.CR1 &= ~(1U << 23);
    }

    void set_discontinuous_mode(uint8_t len) const
    {
        if (len > 8)
            throw std::invalid_argument("discontinuous mode length out of range");
        reg.CR1 &= ~(0x7U << 13);
        reg.CR1 |= (len - 1) << 13;
    }

    uint8_t get_discontinuous_mode() const noexcept
    {
        return ((reg.CR1 >> 13) & 0x7U) + 1;
    }

    void set_regular_discontinuous(bool on) const noexcept
    {
        if (on)
            reg.CR1 |= 1U << 11;
        else
            reg.CR1 &= ~(1U << 11);
    }

    bool is_regular_discontinuous() const noexcept
    {
        return (reg.CR1 >> 11) & 1U;
    }

    bool is_regular_analog_watchdog() const noexcept
    {
        return (reg.CR1 >> 23) & 1U;
    }

    bool is_overrun_interrupt() const noexcept
    {
        return (reg.CR1 >> 26) & 1U;
    }

    void set_analog_watchdog_single_channel_in_scan(bool on) const noexcept
    {
        if (on)
            reg.CR1 |= 1U << 9;
        else
            reg.CR1 &= ~(1U << 9);
    }

    bool is_analog_watchdog_single_channel_in_scan() const noexcept
    {
        return (reg.CR1 >> 9) & 1U;
    }

    void set_analog_watchdog_channel(uint8_t channel) const
    {
        if (channel > 17)
            throw std::invalid_argument("channel out of range");
        reg.CR1 &= ~0x1FU;
        reg.CR1 |= channel;
    }

    uint8_t get_analog_watchdog_channel() const noexcept
    {
        return reg.CR1 & 0x1FU;
    }

    void set_scan_mode(bool on) const noexcept
    {
        if (on)
            reg.CR1 |= 1U << 8;
        else
            reg.CR1 &= ~(1U << 8);
    }

    bool is_scan_mode() const noexcept
    {
        return (reg.CR1 >> 8) & 1U;
    }

    void set_overrun_interrupt(bool on) const noexcept
    {
        if (on)
            reg.CR1 |= 1U << 26;
        else
            reg.CR1 &= ~(1U << 26);
    }

    void set_end_of_conversion_interrupt(bool on) const noexcept
    {
        if (on)
            reg.CR1 |= 1U << 5;
        else
            reg.CR1 &= ~(1U << 5);
    }

    void set_analog_watchdog_interrupt(bool on) const noexcept
    {
        if (on)
            reg.CR1 |= 1U << 6;
        else
            reg.CR1 &= ~(1U << 6);
    }

    void enable_interrupts() const noexcept
    {
        set_overrun_interrupt(true);
        set_end_of_conversion_interrupt(true);
        set_analog_watchdog_interrupt(true);
    }

    void on_regular_done_handler() const noexcept
    try{
        const uint32_t mask = 0x2U;
        if (reg.SR & mask)
        {
            reg.SR &= ~mask;
            reg.SR &= ~0x10U;  // clear started flag
            if (on_regular_done)
                on_regular_done();
        }
    }
    catch(...) {}

    void on_watchdog_handler() const noexcept
    try{
        const uint32_t mask = 0x1U;
        if (reg.SR & mask)
        {
            reg.SR &= ~mask;
            if (on_watchdog)
                on_watchdog();
        }
    }
    catch(...) {}

    void on_overrun_handler() const noexcept
    try{
        const uint32_t mask = 0x4U;
        if (reg.SR & mask)
        {
            reg.SR &= ~mask;
            if (on_overrun)
                on_overrun();
        }
    }
    catch (...) {}

    void global_irq_handler() const noexcept
    {
        on_regular_done_handler();
        on_watchdog_handler();
        on_overrun_handler();
    }

    static void enable_irq() noexcept
    {
        nvic::enable_irq(ADC_IRQn);
    }
};

class InjectedADC : public RegularADC
{
public:
    mutable CallbackType on_injected_done;
    constexpr InjectedADC(uint8_t order, detail::_ADCCommonReg &common_reg, detail::_ADCReg &reg) noexcept
        : RegularADC(order, common_reg, reg) {}
    
    void start_injected() const noexcept
    {
        reg.CR2 |= 1U << 22U;
    }

    /**
     * @brief Get the injected data by order
     * 
     * @param order 
     * @return uint16_t 
     * @throw std::invalid_argument if order is out of range
     */
    uint16_t get_injected_data(uint8_t order) const
    {
        switch (order)
        {
        case 0:
            return reg.JDR1;
        case 1:
            return reg.JDR2;
        case 2:
            return reg.JDR3;
        case 3:
            return reg.JDR4;
        default:
            throw std::invalid_argument("order out of range");
        }
    }

    bool is_injected_running() const noexcept
    {
        return reg.SR & 0x8U;
    }

    void set_injected_trigger_mode(ExternTriggerMode mode) const noexcept
    {
        reg.CR2 &= ~(0x3U << 20);
        reg.CR2 |= static_cast<uint32_t>(mode) << 20;
    }

    ExternTriggerMode get_injected_trigger_mode() const noexcept
    {
        return static_cast<ExternTriggerMode>((reg.CR2 >> 20) & 0x3U);
    }

    void select_injected_trigger_channel(uint8_t channel) const
    {
        if (channel > 15)
            throw std::invalid_argument("channel out of range");
        reg.CR2 &= ~(0xFU << 16);
    }

    uint8_t get_injected_trigger_channel() const noexcept
    {
        return (reg.CR2 >> 16) & 0xFU;
    }

    void set_offset(uint8_t channel, uint16_t offset) const
    {
        if (offset > 0xFFFU)
            throw std::invalid_argument("offset out of range");
        switch (channel)
        {
        case 0:
            reg.JOFR1 = offset;
            break;
        case 1:
            reg.JOFR2 = offset;
            break;
        case 2:
            reg.JOFR3 = offset;
            break;
        case 3:
            reg.JOFR4 = offset;
            break;
        default:
            throw std::invalid_argument("channel out of range");
        }
    }

    uint16_t get_offset(uint8_t channel) const
    {
        switch (channel)
        {
        case 0:
            return reg.JOFR1;
        case 1:
            return reg.JOFR2;
        case 2:
            return reg.JOFR3;
        case 3:
            return reg.JOFR4;
        default:
            throw std::invalid_argument("channel out of range");
        }
    }

    void set_injected_sequence_len(uint8_t len) const
    {
        if (len > 4)
            throw std::invalid_argument("sequence length out of range");
        reg.JSQR &= ~(0x3U << 20);
        reg.JSQR |= (len - 1) << 20;
    }

    uint8_t get_injected_sequence_len() const noexcept
    {
        return ((reg.JSQR >> 20) & 0x3U) + 1;
    }

    void set_injected_sequence(uint8_t order, uint8_t channel) const
    {
        if (order > 3)
            throw std::invalid_argument("sequence order out of range");
        if (channel > 17)
            throw std::invalid_argument("channel out of range");
        const uint8_t shift = order * 5;
        reg.JSQR &= ~(0x1FU << shift);
        reg.JSQR |= channel << shift;
        set_injected_sequence_len(std::max<uint8_t>(order + 1, get_injected_sequence_len()));
    }

    uint8_t get_injected_sequence(uint8_t order) const
    {
        if (order > 3)
            throw std::invalid_argument("sequence order out of range");
        const uint8_t shift = order * 5;
        return (reg.JSQR >> shift) & 0x1FU;
    }

    void set_injected_analog_watchdog(bool on) const noexcept
    {
        if (on)
            reg.CR1 |= 1U << 22;
        else
            reg.CR1 &= ~(1U << 22);
    }

    bool is_injected_analog_watchdog() const noexcept
    {
        return (reg.CR1 >> 22) & 1U;
    }

    void set_injected_discontinuous(bool on) const noexcept
    {
        if (on)
            reg.CR1 |= 1U << 12;
        else
            reg.CR1 &= ~(1U << 12);
    }

    bool is_injected_discontinuous() const noexcept
    {
        return (reg.CR1 >> 12) & 1U;
    }

    void set_auto_injected(bool on) const noexcept
    {
        if (on)
            reg.CR1 |= 1U << 10;
        else
            reg.CR1 &= ~(1U << 10);
    }

    bool is_auto_injected() const noexcept
    {
        return (reg.CR1 >> 10) & 1U;
    }

    void set_injected_interrupt(bool on) const noexcept
    {
        if (on)
            reg.CR1 |= 1U << 7;
        else
            reg.CR1 &= ~(1U << 7);
    }

    void enable_interrupts() const noexcept
    {
        RegularADC::enable_interrupts();
        set_injected_interrupt(true);
    }

    void on_injected_done_handler() const noexcept
    try{
        const uint32_t mask = 0x4U;
        if (reg.SR & mask)
        {
            reg.SR &= ~0x8U;  // clear started flag
            reg.SR &= ~mask;
            if (on_injected_done)
                on_injected_done();
        }
    }
    catch(...) {}

    void global_irq_handler() const noexcept
    {
        RegularADC::global_irq_handler();
        on_injected_done_handler();
    }
};

extern const InjectedADC Adc1;
extern const InjectedADC Adc2;
extern const InjectedADC Adc3;

}
}
}