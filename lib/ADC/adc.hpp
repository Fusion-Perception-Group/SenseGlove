#pragma once

#include <stdexcept>
#include <cstdint>
#include <tuple>
#include "property.hpp"
#include "nvic.hpp"
#include "userconfig.hpp"
#include "block_future.hpp"
#include "rcc.hpp"
#include "dma.hpp"

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

enum class TriggerEvent : uint8_t
{
    #ifdef VERMIL_STM32F411
    Timer1CapCom1 = 0x00,
    Timer1CapCom2 = 0x01,
    Timer1CapCom3 = 0x02,
    Timer2CapCom2 = 0x03,
    Timer2CapCom3 = 0x04,
    Timer2CapCom4 = 0x05,
    Timer2Trgo = 0x06,
    Timer3CapCom1 = 0x07,
    Timer3Trgo = 0x08,
    Timer4CapCom4 = 0x09,
    Timer5CapCom1 = 0x0A,
    Timer5CapCom2 = 0x0B,
    Timer5CapCom3 = 0x0C,
    ExtiLine11 = 0x0F,
    #endif
};

class ADCError : public std::runtime_error
{
public:
    ADCError(const char *msg = "ADC Error") : std::runtime_error(msg) {}
};

class Overrun : public ADCError
{
public:
    Overrun() : ADCError("ADC Overrun") {}
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

    RegularADC(uint8_t order, detail::_ADCCommonReg &common_reg, detail::_ADCReg &reg) noexcept
        : order(order), common_reg(common_reg), reg(reg) {}

    void init() const noexcept
    {
        clock::rcc::enable_clock(*this);
        reg.CR2 |= 1U;
    }
    void deinit() const noexcept
    {
        reg.CR2 &= ~1U;
        clock::rcc::disable_clock(*this);
    }

    void halt() const noexcept
    {
        reg.CR2 &= ~1U;
    }

    bool is_halted() const noexcept
    {
        return !(reg.CR2 & 1U);
    }

    void resume() const noexcept
    {
        reg.CR2 |= 1U;
    }

    void start_regular() const noexcept
    {
        reg.CR2 |= 1U << 30;
    }

    void stop_regular() const noexcept
    {
        reg.CR2 &= ~(1U << 30);
    }

    bool is_regular_started() const noexcept
    {
        return reg.SR & 0x10U;
    }

    void clear_regular_started() const noexcept
    {
        reg.SR &= ~0x10U;
    }

    bool is_regular_done() const noexcept
    {
        return reg.SR & 0x2U;
    }

    void clear_regular_done() const noexcept
    {
        reg.SR &= ~0x2U;
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

    void select_regular_trigger_event(uint8_t event) const
    {
        if (event > 15)
            throw std::invalid_argument("invalid event");
        reg.CR2 &= ~(0xFU << 24);
    }

    void select_regular_trigger_event(TriggerEvent event) const
    {
        select_regular_trigger_event(static_cast<uint8_t>(event));
    }

    TriggerEvent get_regular_trigger_event() const noexcept
    {
        return static_cast<TriggerEvent>((reg.CR2 >> 24) & 0xFU);
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

    /**
     * @brief Set the dma singleshot mode
     * 
     * @param on only trigger DMA once when DMA is enabled
     */
    void set_dma_singleshot(bool on) const noexcept  // only work for single mode ADC
    {
        if (!on)
            reg.CR2 |= 1U << 9;
        else
            reg.CR2 &= ~(1U << 9);
    }

    bool is_dma_singleshot() const noexcept
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

    void config_dma(
        const dma::BaseDMA::Stream &stream,
        const uint8_t channel,
        void * const dst_addr,
        const dma::UnitSize dst_unit_size,
        const bool dst_inc,
        const dma::Priority priority = dma::Priority::High,
        const bool single_shot = false,
        const bool direct = true,
        const bool enable_stream_now = true
        // const dma::FIFOThreshold fifo_threshold = dma::FIFOThreshold::Full,
        // const dma::BurstMode burst_mode = dma::BurstMode::Incr8,
        // const bool double_buffer = false,
        // void * const dbuf_addr = nullptr
    ) const
    {
        set_dma_singleshot(single_shot);
        stream.dma.init();
        stream.src_addr = (void *)(&reg.DR);
        stream.dst_addr = dst_addr;
        stream.select_channel(channel);
        stream.set_use_peri_flow_controller(false);
        stream.count = get_regular_sequence_len();
        stream.set_circular_mode(true);
        stream.set_src_unit_size(dma::UnitSize::HalfWord);
        stream.set_dst_unit_size(dst_unit_size);
        stream.set_src_inc(false);
        stream.set_dst_inc(dst_inc);
        stream.set_priority(priority);
        stream.set_direct_mode(direct);
        stream.enable();
    }

    void config_dma(
        void * const dst_addr,
        const dma::UnitSize dst_unit_size,
        const bool dst_inc,
        const dma::Priority priority = dma::Priority::High,
        const bool single_shot = false,
        const bool direct = true,
        const bool enable_stream_now = true
        // const dma::FIFOThreshold fifo_threshold = dma::FIFOThreshold::Full,
        // const dma::BurstMode burst_mode = dma::BurstMode::Incr8,
        // const bool double_buffer = false,
        // void * const dbuf_addr = nullptr
    ) const
    {
        #ifdef VERMIL_STM32F411
        config_dma(dma::Dma2.streams[0], 0,
            dst_addr, dst_unit_size, dst_inc, priority, single_shot, direct, enable_stream_now);
        #else
        throw std::not_implemented("default DMA not implemented");
        #endif
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

    template <size_t init_order>
    void config_regular_sequence() const {}
    /**
     * @brief config regular sequence
     * 
     * @tparam init_order order for the first channel
     * @param channels channels in conversion order
     */
    template <size_t init_order=0, typename... ARGS>
    void config_regular_sequence(uint8_t channel, ARGS... args) const
    {
        static_assert(sizeof...(ARGS) + 1 <= 16, "too many channels");
        static_assert(init_order < 16, "channel order is too high");
        set_regular_sequence(init_order, channel);
        config_regular_sequence<init_order+1>(args...);
    }
    /**
     * @brief config regular sequence
     * 
     * @tparam init_order order for the first channel
     * @param cfg tuple<channel, sample_cycle> in conversion order
     */
    template <size_t init_order=0, typename... ARGS>
    void config_regular_sequence(std::tuple<uint8_t, SampleCycle> cfg, ARGS... args) const
    {
        static_assert(sizeof...(ARGS) + 1 <= 16, "too many channels");
        static_assert(init_order < 16, "channel order is too high");
        set_regular_sequence(init_order, std::get<0>(cfg));
        set_sample_cycle(std::get<1>(cfg), std::get<0>(cfg));
        config_regular_sequence<init_order+1>(args...);
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

    /**
     * @brief Discontinuous mode will convert first n(n<=8) channels in sequence, then stop
     * 
     * @param len 
     */
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

    void set_regular_watchdog(bool on) const noexcept
    {
        if (on)
            reg.CR1 |= 1U << 23;
        else
            reg.CR1 &= ~(1U << 23);
    }

    bool is_regular_watchdog() const noexcept
    {
        return (reg.CR1 >> 23) & 1U;
    }

    void set_watchdog_single_mode(bool on) const noexcept
    {
        if (on)
            reg.CR1 |= 1U << 9;
        else
            reg.CR1 &= ~(1U << 9);
    }

    bool is_watchdog_single_mode() const noexcept
    {
        return (reg.CR1 >> 9) & 1U;
    }

    /**
     * @brief Set the watchdog channel used in single mode
     * 
     * @param channel
     */
    void set_watchdog_channel(uint8_t channel) const
    {
        if (channel > 17)
            throw std::invalid_argument("channel out of range");
        reg.CR1 &= ~0x1FU;
        reg.CR1 |= channel;
    }

    /**
     * @brief Get the watchdog channel used in single mode
     * 
     * @return uint8_t 
     */
    uint8_t get_watchdog_channel() const noexcept
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

    void raise_if_error() const
    {
        const uint32_t mask = 0x4U;
        if (reg.SR & mask)
        {
            reg.SR &= ~mask;
            throw Overrun();
        }
    }

    void enable_interrupt_overrun() const noexcept
    {
        reg.CR1 |= 1U << 26;
        reg.SR &= ~0x4U; // clear overrun flag
        nvic::enable_irq(ADC_IRQn);
    }
    void disable_interrupt_overrun() const noexcept { reg.CR1 &= ~(1U << 26); }

    void enable_interrupt_regular_done() const noexcept
    {
        reg.CR1 |= 1U << 5;
        reg.SR &= ~0x10U;  // clear started flag
        nvic::enable_irq(ADC_IRQn);
    }
    void disable_interrupt_regular_done() const noexcept { reg.CR1 &= ~(1U << 5); }

    void enable_interrupt_watchdog() const noexcept
    {
        reg.CR1 |= 1U << 6;
        reg.SR &= ~0x1U; // clear watchdog flag
        nvic::enable_irq(ADC_IRQn);
    }
    void disable_interrupt_watchdog() const noexcept { reg.CR1 &= ~(1U << 6); }

    virtual void enable_interrupts() const noexcept
    {
        enable_interrupt_overrun();
        enable_interrupt_regular_done();
        enable_interrupt_watchdog();
    }

    virtual void disable_interrupts() const noexcept
    {
        nvic::disable_irq(ADC_IRQn);
        disable_interrupt_overrun();
        disable_interrupt_regular_done();
        disable_interrupt_watchdog();
    }

    void on_regular_done_handler() const noexcept
    try{
        const uint32_t mask = 0x2U;
        if (reg.CR1 & (1U << 5) && reg.SR & mask)
        {
            reg.SR &= ~mask;
            if (on_regular_done)
                on_regular_done();
        }
    }
    catch(...) {}

    void on_watchdog_handler() const noexcept
    try{
        const uint32_t mask = 0x1U;
        if (reg.CR1 & (1U << 6) && reg.SR & mask)
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
        if (reg.CR1 & (1U << 26) && reg.SR & mask)
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

    static void set_irq_priority(const uint8_t priority=8)
    {
        nvic::set_priority(ADC_IRQn, priority);
    }
};

class InjectedADC : public RegularADC
{
public:
    mutable CallbackType on_injected_done;
    InjectedADC(uint8_t order, detail::_ADCCommonReg &common_reg, detail::_ADCReg &reg) noexcept
        : RegularADC(order, common_reg, reg) {}
    
    void start_injected() const noexcept
    {
        reg.CR2 |= 1U << 22U;
    }

    void stop_injected() const noexcept
    {
        reg.CR2 &= ~(1U << 22U);
    }

    /**
     * @brief Get the injected data by order
     * 
     * @param order 
     * @return int16_t because of the offset, the data could be negative
     * @throw std::invalid_argument if order is out of range
     */
    int16_t get_injected_data(uint8_t order) const
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

    bool is_injected_done() const noexcept
    {
        return reg.SR & 0x4U;
    }

    void clear_injected_done() const noexcept
    {
        reg.SR &= ~0x4U;
    }

    bool is_injected_started() const noexcept
    {
        return reg.SR & 0x8U;
    }

    void clear_injected_started() const noexcept
    {
        reg.SR &= ~0x8U;
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

    void select_injected_trigger_event(uint8_t event) const
    {
        if (event > 15)
            throw std::invalid_argument("event out of range");
        reg.CR2 &= ~(0xFU << 16);
    }

    void select_injected_trigger_event(TriggerEvent event) const
    {
        select_injected_trigger_event(static_cast<uint8_t>(event));
    }

    TriggerEvent get_injected_trigger_event() const noexcept
    {
        return static_cast<TriggerEvent>((reg.CR2 >> 16) & 0xFU);
    }

    /**
     * @brief Set the offset, which is subtracted from the raw data
     * 
     * @param offset 
     * @param order order in injected sequence
     */
    void set_offset( uint16_t offset, uint8_t order) const
    {
        if (offset > 0xFFFU)
            throw std::invalid_argument("offset out of range");
        switch (order)
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
            throw std::invalid_argument("order out of range");
        }
    }

    uint16_t get_offset(uint8_t order) const
    {
        switch (order)
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
            throw std::invalid_argument("order out of range");
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

    template <size_t init_order>
    void config_injected_sequence() const {}
    /**
     * @brief config injected sequence
     * 
     * @tparam init_order order for the first channel
     * @param channels channels in conversion order
     */
    template <size_t init_order=0, typename... ARGS>
    void config_injected_sequence(uint8_t channel, ARGS... args) const
    {
        static_assert(sizeof...(ARGS) + 1 <= 4, "too many channels");
        static_assert(init_order < 16, "channel order is too high");

        set_injected_sequence(init_order, channel);
        config_injected_sequence<init_order+1>(args...);
    }
    /**
     * @brief config injected sequence
     * 
     * @tparam init_order order for the first channel
     * @param cfg tuple<channel, sample_cycle> in conversion order
     */
    template <size_t init_order=0, typename... ARGS>
    void config_injected_sequence(std::tuple<uint8_t, SampleCycle> cfg, ARGS... args) const
    {
        static_assert(sizeof...(ARGS) + 1 <= 4, "too many channels");
        static_assert(init_order < 16, "channel order is too high");

        set_injected_sequence(init_order, std::get<0>(cfg));
        set_sample_cycle(std::get<1>(cfg), std::get<0>(cfg));
        config_injected_sequence<init_order+1>(args...);
    }
    /**
     * @brief config injected sequence
     * 
     * @tparam init_order order for the first channel
     * @param cfg tuple<channel, sample_cycle, offset> in conversion order
     */
    template <size_t init_order=0, typename... ARGS>
    void config_injected_sequence(std::tuple<uint8_t, SampleCycle, uint16_t> cfg, ARGS... args) const
    {
        static_assert(sizeof...(ARGS) + 1 <= 4, "too many channels");
        static_assert(init_order < 16, "channel order is too high");

        set_injected_sequence(init_order, std::get<0>(cfg));
        set_sample_cycle(std::get<1>(cfg), std::get<0>(cfg));
        set_offset(std::get<2>(cfg), init_order);
        config_injected_sequence<init_order+1>(args...);
    }

    void set_injected_watchdog(bool on) const noexcept
    {
        if (on)
            reg.CR1 |= 1U << 22;
        else
            reg.CR1 &= ~(1U << 22);
    }

    bool is_injected_watchdog() const noexcept
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

    void enable_interrupt_injected_done() const noexcept
    {
        reg.CR1 |= 1U << 7;
        reg.SR &= ~0x4U;  // clear started flag
        nvic::enable_irq(ADC_IRQn);
    }
    void disable_interrupt_injected_done() const noexcept { reg.CR1 &= ~(1U << 7); }

    void enable_interrupts() const noexcept override
    {
        RegularADC::enable_interrupts();
        enable_interrupt_injected_done();
    }

    void disable_interrupts() const noexcept override
    {
        RegularADC::disable_interrupts();
        disable_interrupt_injected_done();
    }

    void on_injected_done_handler() const noexcept
    try{
        const uint32_t mask = 0x4U;
        if (reg.CR1 & 1U << 7 && reg.SR & mask)
        {
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