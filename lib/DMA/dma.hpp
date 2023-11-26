#pragma once

#include <stdexcept>
#include <cstdint>
#include <functional>
#include <array>
#include "block_future.hpp"
#include "property.hpp"
#include "userconfig.hpp"
#include "nvic.hpp"
namespace vermils
{
namespace stm32
{
namespace dma
{
using CallbackType = std::function<void()>;

namespace detail
{
    struct _DMAStreamReg
    {
        volatile uint32_t CR;     /*!< DMA stream x configuration register      */
        volatile uint32_t NDTR;   /*!< DMA stream x number of data register     */
        volatile uint8_t * PAR;    /*!< DMA stream x peripheral address register */
        volatile uint8_t * M0AR;   /*!< DMA stream x memory 0 address register   */
        volatile uint8_t * M1AR;   /*!< DMA stream x memory 1 address register   */
        volatile uint32_t FCR;    /*!< DMA stream x FIFO control register       */
    };

    struct _DMAReg
    {
        uint32_t LISR;   /*!< DMA low interrupt status register,      Address offset: 0x00 */
        uint32_t HISR;   /*!< DMA high interrupt status register,     Address offset: 0x04 */
        uint32_t LIFCR;  /*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
        uint32_t HIFCR;  /*!< DMA high interrupt flag clear register, Address offset: 0x0C */
    };

    extern _DMAReg & DMA1Reg;
    extern _DMAStreamReg & DMA1Stream0Reg;
    extern _DMAStreamReg & DMA1Stream1Reg;
    extern _DMAStreamReg & DMA1Stream2Reg;
    extern _DMAStreamReg & DMA1Stream3Reg;
    extern _DMAStreamReg & DMA1Stream4Reg;
    extern _DMAStreamReg & DMA1Stream5Reg;
    extern _DMAStreamReg & DMA1Stream6Reg;
    extern _DMAStreamReg & DMA1Stream7Reg;
    extern _DMAReg & DMA2Reg;
    extern _DMAStreamReg & DMA2Stream0Reg;
    extern _DMAStreamReg & DMA2Stream1Reg;
    extern _DMAStreamReg & DMA2Stream2Reg;
    extern _DMAStreamReg & DMA2Stream3Reg;
    extern _DMAStreamReg & DMA2Stream4Reg;
    extern _DMAStreamReg & DMA2Stream5Reg;
    extern _DMAStreamReg & DMA2Stream6Reg;
    extern _DMAStreamReg & DMA2Stream7Reg;
}

enum class BurstMode
{
    Single = 0,
    Incr4 = 1,
    Incr8 = 2,
    Incr16 = 3
};

enum class Priority
{
    Low = 0,
    Medium = 1,
    High = 2,
    VeryHigh = 3
};

enum class UnitSize
{
    Byte = 0,
    HalfWord = 1,
    Word = 2
};

enum class Direction
{
    Peri2Mem = 0,
    Mem2Peripheral = 1,
    Mem2Mem = 2
};

enum class FIFOStatus
{
    LessThanQuarter = 0,
    LessThanHalf = 1,
    LessThanThreeQuarters = 2,
    LessThanFull = 3,
    Empty = 4,
    Full = 5
};

enum class FIFOThreshold
{
    Quarter = 0,
    Half = 1,
    ThreeQuarters = 2,
    Full = 3
};

class BaseDMA
{
public:
    detail::_DMAReg & reg;
    class Stream
    {
    protected:
        template <typename T>
        struct _Property : tricks::StaticProperty<T, Stream &>
        {
            constexpr _Property(Stream & stream) : tricks::StaticProperty<T, Stream &>(stream) {}
            using tricks::StaticProperty<T, Stream &>::operator=;
        };
        struct _Enabled : public _Property<bool>
        {
            constexpr _Enabled(Stream & stream) : _Property<bool>(stream) {}
            using _Property<bool>::operator=;
            bool getter() const override
            {
                return static_cast<uint16_t>(this->owner.reg.CR & 1U);
            }
            void setter(bool on) const override
            {
                if (on)
                    owner.enable();
                else
                    owner.disable();
            }
        };
        struct _Count : public _Property<uint16_t>
        {
            constexpr _Count(Stream & stream) : _Property<uint16_t>(stream) {}
            using _Property<uint16_t>::operator=;
            uint16_t getter() const override
            {
                return this->owner.reg.NDTR;
            }
            void setter(uint16_t on) const override
            {
                this->owner.reg.NDTR = on;
            }
        };
        BaseDMA & _adc;
    public:
        const uint8_t order;
        detail::_DMAStreamReg & reg;
        const nvic::IRQn_Type irqn;
        mutable CallbackType on_complete;
        mutable CallbackType on_half_complete;
        mutable CallbackType on_direct_mode_error;
        mutable CallbackType on_transfer_error;
        mutable CallbackType on_fifo_error;
        volatile uint8_t * & dst0_addr = reg.M0AR; // M0AR is the default destination address in memory
        volatile uint8_t * & dst1_addr = reg.M1AR; // M1AR is the alternate destination address in memory (used in double buffer mode)
        volatile uint8_t * & peri_addr = reg.PAR; // PAR is the default source address in peripheral
        _Enabled enabled{*this};
        _Count count{*this};

        constexpr Stream(BaseDMA & adc, uint8_t order, detail::_DMAStreamReg & reg, nvic::IRQn_Type irqn) : _adc(adc), order(order), reg(reg), irqn(irqn) {}
        virtual ~Stream() = default;
        Stream & operator=(const Stream &) = delete;

        void enable() const noexcept
        {
            reg.CR |= 1U;
        }

        void disable() const noexcept
        {
            reg.CR &= ~1U;
        }

        void select_channel(uint8_t channel) const
        {
            if (channel > 7)
                throw std::invalid_argument("channel must be between 0 and 7");
            reg.CR &= ~(0x7 << 25);
            reg.CR |= (channel << 25);
        }

        uint8_t get_channel() const noexcept
        {
            return (reg.CR >> 25) & 0x7;
        }

        void set_mburst(BurstMode mode) const noexcept
        {
            reg.CR &= ~(0x3 << 23);
            reg.CR |= (static_cast<uint8_t>(mode) << 23);
        }

        BurstMode get_mburst() const
        {
            return static_cast<BurstMode>((reg.CR >> 23) & 0x3);
        }

        void set_pburst(BurstMode mode) const noexcept
        {
            reg.CR &= ~(0x3 << 21);
            reg.CR |= (static_cast<uint8_t>(mode) << 21);
        }

        BurstMode get_pburst() const noexcept
        {
            return static_cast<BurstMode>((reg.CR >> 21) & 0x3);
        }

        void set_double_buffer(bool on) const noexcept
        {
            if (on)
                reg.CR |= (1 << 18);
            else
                reg.CR &= ~(1 << 18);
        }

        bool is_double_buffer() const noexcept
        {
            return reg.CR & (1 << 18);
        }

        void set_priority(Priority priority) const noexcept
        {
            reg.CR &= ~(0x3 << 16);
            reg.CR |= (static_cast<uint8_t>(priority) << 16);
        }

        Priority get_priority() const noexcept
        {
            return static_cast<Priority>((reg.CR >> 16) & 0x3);
        }

        /**
         * @brief Set the peri inc override, override value is 4 bytes
         * 
         * @param on 
         */
        void set_peri_inc_override(bool on) const noexcept
        {
            if (on)
                reg.CR |= (1 << 15);
            else
                reg.CR &= ~(1 << 15);
        }

        bool is_peri_inc_override() const noexcept
        {
            return reg.CR & (1 << 15);
        }

        void set_dst_unit_size(UnitSize size) const noexcept
        {
            reg.CR &= ~(0x3 << 13);
            reg.CR |= (static_cast<uint8_t>(size) << 13);
        }

        UnitSize get_dst_unit_size() const noexcept
        {
            return static_cast<UnitSize>((reg.CR >> 13) & 0x3);
        }

        void set_peri_unit_size(UnitSize size) const noexcept
        {
            reg.CR &= ~(0x3 << 11);
            reg.CR |= (static_cast<uint8_t>(size) << 11);
        }

        UnitSize get_peri_unit_size() const noexcept
        {
            return static_cast<UnitSize>((reg.CR >> 11) & 0x3);
        }

        void set_dst_inc(bool on) const noexcept
        {
            if (on)
                reg.CR |= (1 << 10);
            else
                reg.CR &= ~(1 << 10);
        }

        bool is_dst_inc() const noexcept
        {
            return reg.CR & (1 << 10);
        }

        void set_peri_inc(bool on) const noexcept
        {
            if (on)
                reg.CR |= (1 << 9);
            else
                reg.CR &= ~(1 << 9);
        }

        bool is_peri_inc() const noexcept
        {
            return reg.CR & (1 << 9);
        }

        void set_circular_mode(bool on) const noexcept
        {
            if (on)
                reg.CR |= (1 << 8);
            else
                reg.CR &= ~(1 << 8);
        }

        bool is_circular_mode() const noexcept
        {
            return reg.CR & (1 << 8);
        }

        void set_direct_mode(bool on) const noexcept
        {
            if (on)
                reg.FCR &= ~(1 << 2);
            else
                reg.FCR |= (1 << 2);
        }

        bool is_direct_mode() const noexcept
        {
            return !(reg.FCR & (1 << 2));
        }

        FIFOStatus get_fifo_status() const noexcept
        {
            return static_cast<FIFOStatus>((reg.FCR >> 3) & 0x7);
        }

        void set_fifo_thre(FIFOThreshold threshold) const noexcept
        {
            reg.FCR &= ~(0x3 << 0);
            reg.FCR |= (static_cast<uint8_t>(threshold) << 0);
        }

        FIFOThreshold get_fifo_thre() const noexcept
        {
            return static_cast<FIFOThreshold>((reg.FCR >> 0) & 0x3);
        }

        virtual void set_direction(Direction direction) const
        {
            if (direction == Direction::Mem2Mem)
                throw std::invalid_argument("direction cannot be Mem2Mem");
            reg.CR &= ~(0x3 << 6);
            reg.CR |= (static_cast<uint8_t>(direction) << 6);
        }

        Direction get_direction() const noexcept
        {
            return static_cast<Direction>((reg.CR >> 6) & 0x3);
        }

        void set_use_peri_flow_controller(bool on) const noexcept
        {
            if (on)
                reg.CR |= (1 << 5);
            else
                reg.CR &= ~(1 << 5);
        }

        bool is_use_peri_flow_controller() const noexcept
        {
            return reg.CR & (1 << 5);
        }

        volatile uint8_t * current_dst() const  // return the current destination address in double buffer mode
        {
            return reg.CR & (1 << 19) ? reg.M1AR : reg.M0AR;
        }

        void enable_on_complete_interrupt() const noexcept { reg.CR |= (1 << 4); }
        void disable_on_complete_interrupt() const noexcept { reg.CR &= ~(1 << 4); }
        void enable_on_half_interrupt() const noexcept { reg.CR |= (1 << 3); }
        void disable_on_half_interrupt() const noexcept { reg.CR &= ~(1 << 3); }
        void enable_on_transfer_error_interrupt() const noexcept { reg.CR |= (1 << 2); }
        void disable_on_transfer_error_interrupt() const noexcept { reg.CR &= ~(1 << 2); }
        void enable_on_direct_mode_error_interrupt() const noexcept { reg.CR |= (1 << 1); }
        void disable_on_direct_mode_error_interrupt() const noexcept { reg.CR &= ~(1 << 1); }
        void enable_on_fifo_error_interrupt() const noexcept { reg.FCR |= (1 << 7); }
        void disable_on_fifo_error_interrupt() const noexcept { reg.FCR &= ~(1 << 7); }

        void enable_interrupts() const noexcept { reg.CR |= 0xFU << 1U; reg.FCR |= (1 << 7); }
        void disable_interrupts() const noexcept { reg.CR &= ~(0xFU << 1U); reg.FCR &= ~(1 << 7); }

        void enable_irq() const noexcept { nvic::enable_irq(irqn); }
        void disable_irq() const noexcept { nvic::disable_irq(irqn); }

        void on_fifo_err_handler(volatile uint32_t * ifcr, volatile uint32_t * ifsr) const noexcept
        {
            const uint32_t mask = 0x1U << (6 * (order % 2) + 16 * (order % 4 > 2));
            if (*ifsr & mask)
            {
                *ifcr |= mask;
                if (on_fifo_error)
                    on_fifo_error();
            }
        }
        void on_direct_mode_err_handler(volatile uint32_t * ifcr, volatile uint32_t * ifsr) const noexcept
        {
            const uint32_t mask = 0x4U << (6 * (order % 2) + 16 * (order % 4 > 2));
            if (*ifsr & mask)
            {
                *ifcr |= mask;
                if (on_direct_mode_error)
                    on_direct_mode_error();
            }
        }
        void on_transfer_err_handler(volatile uint32_t * ifcr, volatile uint32_t * ifsr) const noexcept
        {
            const uint32_t mask = 0x8U << (6 * (order % 2) + 16 * (order % 4 > 2));
            if (*ifsr & mask)
            {
                *ifcr |= mask;
                if (on_transfer_error)
                    on_transfer_error();
            }
        }
        void on_half_complete_handler(volatile uint32_t * ifcr, volatile uint32_t * ifsr) const noexcept
        {
            const uint32_t mask = 0x10U << (6 * (order % 2) + 16 * (order % 4 > 2));
            if (*ifsr & mask)
            {
                *ifcr |= mask;
                if (on_half_complete)
                    on_half_complete();
            }
        }
        void on_complete_handler(volatile uint32_t * ifcr, volatile uint32_t * ifsr) const noexcept
        {
            const uint32_t mask = 0x20U << (6 * (order % 2) + 16 * (order % 4 > 2));
            if (*ifsr & mask)
            {
                *ifcr |= mask;
                if (on_complete)
                    on_complete();
            }
        }

        void global_irq_handler() const noexcept
        try{
            volatile uint32_t * ifcr;
            volatile uint32_t * ifsr;
            if (order < 4)
            {
                ifcr = &_adc.reg.LIFCR;
                ifsr = &_adc.reg.LISR;
            }
            else
            {
                ifcr = &_adc.reg.HIFCR;
                ifsr = &_adc.reg.HISR;
            }

            on_complete_handler(ifcr, ifsr);
            on_fifo_err_handler(ifcr, ifsr);
            on_direct_mode_err_handler(ifcr, ifsr);
            on_transfer_err_handler(ifcr, ifsr);
            on_half_complete_handler(ifcr, ifsr);
        }
        catch (...)
        {
        }
    };

    class StreamM2M : public Stream
    {
    public:

        constexpr StreamM2M(BaseDMA & adc, uint8_t order, detail::_DMAStreamReg & reg, nvic::IRQn_Type irqn) : Stream(adc, order, reg, irqn) {}

        void set_direction(Direction direction) const override
        {
            reg.CR &= ~(0x3 << 6);
            reg.CR |= (static_cast<uint8_t>(direction) << 6);
        }
    };

    BaseDMA(detail::_DMAReg & reg) : reg(reg) {}
    virtual void init() const noexcept = 0;
    virtual void deinit() const noexcept = 0;
};

class DMA : public BaseDMA
{
public:
    using BaseDMA::BaseDMA;
    std::array<Stream, 8> streams{
        Stream(*this, 0, detail::DMA1Stream0Reg, nvic::IRQn_Type::DMA1_Stream0_IRQn),
        Stream(*this, 1, detail::DMA1Stream1Reg, nvic::IRQn_Type::DMA1_Stream1_IRQn),
        Stream(*this, 2, detail::DMA1Stream2Reg, nvic::IRQn_Type::DMA1_Stream2_IRQn),
        Stream(*this, 3, detail::DMA1Stream3Reg, nvic::IRQn_Type::DMA1_Stream3_IRQn),
        Stream(*this, 4, detail::DMA1Stream4Reg, nvic::IRQn_Type::DMA1_Stream4_IRQn),
        Stream(*this, 5, detail::DMA1Stream5Reg, nvic::IRQn_Type::DMA1_Stream5_IRQn),
        Stream(*this, 6, detail::DMA1Stream6Reg, nvic::IRQn_Type::DMA1_Stream6_IRQn),
        Stream(*this, 7, detail::DMA1Stream7Reg, nvic::IRQn_Type::DMA1_Stream7_IRQn)
    };
    void init() const noexcept override;
    void deinit() const noexcept override;
};

class DMA_M2M : public BaseDMA
{
public:
    using BaseDMA::BaseDMA;
    std::array<StreamM2M, 8> streams{
        StreamM2M(*this, 0, detail::DMA2Stream0Reg, nvic::IRQn_Type::DMA2_Stream0_IRQn),
        StreamM2M(*this, 1, detail::DMA2Stream1Reg, nvic::IRQn_Type::DMA2_Stream1_IRQn),
        StreamM2M(*this, 2, detail::DMA2Stream2Reg, nvic::IRQn_Type::DMA2_Stream2_IRQn),
        StreamM2M(*this, 3, detail::DMA2Stream3Reg, nvic::IRQn_Type::DMA2_Stream3_IRQn),
        StreamM2M(*this, 4, detail::DMA2Stream4Reg, nvic::IRQn_Type::DMA2_Stream4_IRQn),
        StreamM2M(*this, 5, detail::DMA2Stream5Reg, nvic::IRQn_Type::DMA2_Stream5_IRQn),
        StreamM2M(*this, 6, detail::DMA2Stream6Reg, nvic::IRQn_Type::DMA2_Stream6_IRQn),
        StreamM2M(*this, 7, detail::DMA2Stream7Reg, nvic::IRQn_Type::DMA2_Stream7_IRQn)
    };
    void init() const noexcept override;
    void deinit() const noexcept override;
};

extern const DMA Dma1;
extern const DMA_M2M Dma2;

}
}
}
