#pragma once

#include <stdexcept>
#include <cstdint>
#include <functional>
#include <array>
#include "block_future.hpp"
#include "property.hpp"
#include "userconfig.hpp"
#include "nvic.hpp"
#include "rcc.hpp"
namespace vms
{
namespace stm32
{
namespace dma
{
using CallbackType = std::function<void(bool full_complete)>;

namespace detail
{
    struct _DMAStreamReg
    {
        volatile uint32_t CR;     /*!< DMA stream x configuration register      */
        volatile uint32_t NDTR;   /*!< DMA stream x number of data register     */
        volatile void * volatile  PAR;    /*!< DMA stream x peripheral address register */
        volatile void * volatile  M0AR;   /*!< DMA stream x memory 0 address register   */
        volatile void * volatile  M1AR;   /*!< DMA stream x memory 1 address register   */
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
    Top = 3
};

enum class UnitSize
{
    Byte = 0,
    HalfWord = 1,
    Word = 2
};

enum AddressType
{
    Flash,
    Ram,
    Peripheral,
    Unknown
};

enum class Direction
{
    Peri2Ram = 0,
    Ram2Peri = 1,
    Ram2Ram = 2
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

class DMAError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

class TransferError : public DMAError
{
public:
    TransferError(const char *msg = "DMA Transfer Error") : DMAError(msg) {}
};

class DirectModeError : public DMAError
{
public:
    DirectModeError(const char *msg = "Direct Mode Error") : DMAError(msg) {}
};

class FIFOError : public DMAError
{
public:
    FIFOError(const char *msg = "FIFO Error") : DMAError(msg) {}
};

using ErrCallBackType = std::function<void(const DMAError&)>;

/**
 * @brief Get the addr type regarding to the address and STM32 mem-map convention
 * 
 * @param addr 
 * @return AddressType 
 */
inline AddressType get_addr_type(const uintptr_t addr) noexcept
{
    if (addr < 0x20000000)
        return AddressType::Flash;
    else if (addr >= 0x20000000 && addr < 0x40000000)
        return AddressType::Ram;
    else if (addr >= 0x40000000 && addr <= 0xFFFFFFFF)
        return AddressType::Peripheral;
    else
        return AddressType::Unknown;
}

inline AddressType get_addr_type(const volatile void * addr) noexcept
{
    return get_addr_type(reinterpret_cast<uintptr_t>(addr));
}
class BaseDMA
{
public:
    const uint8_t order;
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
        struct _DestAddr : public _Property<volatile void *>
        {
            constexpr _DestAddr(Stream & stream) : _Property<volatile void *>(stream) {}
            using _Property<volatile void *>::operator=;
            volatile void * getter() const override
            {
                if (owner.get_direction() == Direction::Ram2Peri)
                    return owner.reg.PAR;
                else
                    return owner.reg.M0AR;
            }
            void setter(volatile void * value) const override
            {
                const bool dst_is_ram = is_ram_addr(value);
                void * const src_addr = owner.src_addr;
                const bool src_is_ram = is_ram_addr(src_addr);
                if (not dst_is_ram)
                {
                    owner.reg.PAR = value;
                    owner.reg.M0AR = src_addr;
                    owner.set_direction(Direction::Ram2Peri);
                }
                else 
                {
                    owner.reg.M0AR = value;
                    owner.reg.PAR = src_addr;
                    if (src_is_ram)
                        owner.set_direction(Direction::Ram2Ram);
                    else
                        owner.set_direction(Direction::Peri2Ram);
                }
            }
        };
        struct _SauceAddr : public _Property<void *>
        {
            constexpr _SauceAddr(Stream & stream) : _Property<void *>(stream) {}
            using _Property<void *>::operator=;
            void * getter() const override
            {
                if (owner.get_direction() == Direction::Ram2Peri)
                    return const_cast<void*>(owner.reg.M0AR);
                else
                    return const_cast<void*>(owner.reg.PAR);
            }
            void setter(void * value) const override
            {
                const bool src_is_ram = is_ram_addr(value);
                volatile void * const dst_addr = owner.dst_addr;
                const bool dst_is_ram = is_ram_addr(dst_addr);
                if (not src_is_ram)
                {
                    owner.reg.PAR = value;
                    owner.reg.M0AR = dst_addr;
                    owner.set_direction(Direction::Peri2Ram);
                }
                else if (dst_is_ram)
                {
                    owner.reg.PAR = value;
                    owner.reg.M0AR = dst_addr;
                    owner.set_direction(Direction::Ram2Ram);
                }
                else
                {
                    owner.reg.M0AR = value;
                    owner.reg.PAR = dst_addr;
                    owner.set_direction(Direction::Ram2Peri);
                }
            }
        };
        void _set_peri_unit_size(UnitSize size) const noexcept
        {
            reg.CR &= ~(0x3 << 11);
            reg.CR |= (static_cast<uint8_t>(size) << 11);
        }
        UnitSize _get_peri_unit_size() const noexcept
        {
            return static_cast<UnitSize>((reg.CR >> 11) & 0x3);
        }
        void _set_ram_unit_size(UnitSize size) const noexcept
        {
            reg.CR &= ~(0x3 << 13);
            reg.CR |= (static_cast<uint8_t>(size) << 13);
        }
        UnitSize _get_ram_unit_size() const noexcept
        {
            return static_cast<UnitSize>((reg.CR >> 13) & 0x3);
        }
        void _set_ram_inc(bool on) const noexcept
        {
            if (on)
                reg.CR |= (1 << 10);
            else
                reg.CR &= ~(1 << 10);
        }

        bool _is_ram_inc() const noexcept
        {
            return reg.CR & (1 << 10);
        }

        void _set_peri_inc(bool on) const noexcept
        {
            if (on)
                reg.CR |= (1 << 9);
            else
                reg.CR &= ~(1 << 9);
        }

        bool _is_peri_inc() const noexcept
        {
            return reg.CR & (1 << 9);
        }
    public:
        BaseDMA &dma;
        const uint8_t order;
        detail::_DMAStreamReg & reg;
        const nvic::IRQn_Type irqn;
        mutable CallbackType on_complete; // accept `bool` as argument, true for full complete, false for half complete
        // mutable CallbackType on_half_complete;
        mutable ErrCallBackType on_error; // accept `const DMAError&` as argument
        // mutable CallbackType on_direct_mode_error;
        // mutable CallbackType on_transfer_error;
        // mutable CallbackType on_fifo_error;
        _DestAddr dst_addr{*this}; // destination address
        _SauceAddr src_addr{*this}; // source address
        volatile void * volatile & dbuf_addr = reg.M1AR; // double buffer destination address in ram (only used in double buffer mode)

        _Count count{*this};

        Stream(BaseDMA & dma, uint8_t order, detail::_DMAStreamReg & reg, nvic::IRQn_Type irqn) : dma(dma), order(order), reg(reg), irqn(irqn) {}
        virtual ~Stream() = default;
        Stream & operator=(const Stream &) = delete;

        static bool is_ram_addr(const volatile void * addr) noexcept
        {
            return get_addr_type(addr) == AddressType::Ram;
        }

        void enable() const noexcept
        {
            if (order < 4)  // clear related event flags before enabling stream
                dma.reg.LIFCR |= (0x3D << (6 * (order % 2) + 16 * (order % 4 > 2)));
            else
                dma.reg.HIFCR |= (0x3D << (6 * (order % 2) + 16 * (order % 4 > 2)));
            reg.CR |= 1U;
        }

        void disable() const noexcept
        {
            reg.CR &= ~1U;
        }

        bool is_enabled() const noexcept
        {
            return reg.CR & 1U;
        }

        bool is_busy() const noexcept
        {
            return reg.CR & 1U;
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
         * @brief Set the peri inc unit size override, override value is 4 bytes
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
            if (get_direction() == Direction::Ram2Peri)
                _set_peri_unit_size(size);
            else
                _set_ram_unit_size(size);
        }

        UnitSize get_dst_unit_size() const noexcept
        {
            if (get_direction() == Direction::Ram2Peri)
                return _get_peri_unit_size();
            else
                return _get_ram_unit_size();
        }

        void set_src_unit_size(UnitSize size) const noexcept
        {
            if (get_direction() == Direction::Ram2Peri)
                _set_ram_unit_size(size);
            else
                _set_peri_unit_size(size);
        }

        UnitSize get_src_unit_size() const noexcept
        {
            if (get_direction() == Direction::Ram2Peri)
                return _get_ram_unit_size();
            else
                return _get_peri_unit_size();
        }

        void set_dst_inc(bool on) const noexcept
        {
            if (get_direction() == Direction::Ram2Peri)
                _set_peri_inc(on);
            else
                _set_ram_inc(on);
        }

        bool is_dst_inc() const noexcept
        {
            if (get_direction() == Direction::Ram2Peri)
                return _is_peri_inc();
            else
                return _is_ram_inc();
        }

        void set_src_inc(bool on) const noexcept
        {
            if (get_direction() == Direction::Ram2Peri)
                _set_ram_inc(on);
            else
                _set_peri_inc(on);
        }

        bool is_src_inc() const noexcept
        {
            if (get_direction() == Direction::Ram2Peri)
                return _is_ram_inc();
            else
                return _is_peri_inc();
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
            if (direction == Direction::Ram2Ram)
                throw std::invalid_argument("direction cannot be Ram2Ram");
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

        volatile void * current_dst() const  // return the current destination address in double buffer mode
        {
            return reg.CR & (1 << 19) ? reg.M1AR : reg.M0AR;
        }

        void enable_interrupt_complete() const noexcept
        {
            reg.CR |= (1 << 4);
            nvic::enable_irq(irqn);
        }
        void disable_interrupt_complete() const noexcept { reg.CR &= ~(1 << 4); }
        void enable_interrupt_half() const noexcept
        {
            reg.CR |= (1 << 3);
            nvic::enable_irq(irqn);
        }
        void disable_interrupt_half() const noexcept { reg.CR &= ~(1 << 3); }
        void enable_interrupt_transfer_error() const noexcept
        {
            reg.CR |= (1 << 2);
            nvic::enable_irq(irqn);
        }
        void disable_interrupt_transfer_error() const noexcept { reg.CR &= ~(1 << 2); }
        void enable_interrupt_direct_mode_error() const noexcept
        {
            reg.CR |= (1 << 1);
            nvic::enable_irq(irqn);
        }
        void disable_interrupt_direct_mode_error() const noexcept { reg.CR &= ~(1 << 1); }
        void enable_interrupt_fifo_error() const noexcept
        {
            reg.FCR |= (1 << 7);
            nvic::enable_irq(irqn);
        }
        void disable_interrupt_fifo_error() const noexcept { reg.FCR &= ~(1 << 7); }

        void enable_interrupts() const noexcept
        {
            reg.CR |= 0xFU << 1U;
            reg.FCR |= (1 << 7);
            nvic::enable_irq(irqn);
        }
        void disable_interrupts() const noexcept
        {
            nvic::disable_irq(irqn);
            reg.CR &= ~(0xFU << 1U);
            reg.FCR &= ~(1 << 7);
        }

        void set_irq_priority(const uint8_t priority=8) const
        {
            nvic::set_priority(irqn, priority);
        }

        void on_fifo_err_handler(volatile uint32_t * ifclr, volatile uint32_t * ifsr) const noexcept
        try{
            const uint32_t mask = 0x1U << (6 * (order % 2) + 16 * (order % 4 > 2));
            if (reg.FCR & (1 << 7) && *ifsr & mask)
            {
                *ifclr = mask;
                // if (on_fifo_error)
                //     on_fifo_error();
                if (on_error)
                    on_error(FIFOError());
            }
        }
        catch(...) {}
        void on_direct_mode_err_handler(volatile uint32_t * ifclr, volatile uint32_t * ifsr) const noexcept
        try{
            const uint32_t mask = 0x4U << (6 * (order % 2) + 16 * (order % 4 > 2));
            if (reg.CR & (1 << 1) && *ifsr & mask)
            {
                *ifclr = mask;
                // if (on_direct_mode_error)
                //     on_direct_mode_error();
                if (on_error)
                    on_error(DirectModeError());
            }
        }
        catch(...) {}
        void on_transfer_err_handler(volatile uint32_t * ifclr, volatile uint32_t * ifsr) const noexcept
        try{
            const uint32_t mask = 0x8U << (6 * (order % 2) + 16 * (order % 4 > 2));
            if (reg.CR & (1 << 2) && *ifsr & mask)
            {
                *ifclr = mask;
                // if (on_transfer_error)
                //     on_transfer_error();
                if (on_error)
                    on_error(TransferError());
            }
        }
        catch(...) {}
        void on_half_complete_handler(volatile uint32_t * ifclr, volatile uint32_t * ifsr) const noexcept
        try{
            const uint32_t mask = 0x10U << (6 * (order % 2) + 16 * (order % 4 > 2));
            if (reg.CR & (1 << 3) && *ifsr & mask)
            {
                *ifclr = mask;
                // if (on_half_complete)
                //     on_half_complete();
                if (on_complete)
                    on_complete(false);
            }
        }
        catch(...) {}
        void on_complete_handler(volatile uint32_t * ifclr, volatile uint32_t * ifsr) const noexcept
        try{
            const uint32_t mask = 0x20U << (6 * (order % 2) + 16 * (order % 4 > 2));
            if (reg.CR & (1 << 4) && *ifsr & mask)
            {
                *ifclr |= mask;
                if (on_complete)
                    on_complete(true);
            }
        }
        catch(...) {}

        void raise_if_error() const
        {
            volatile uint32_t &sr = (order < 4) ? dma.reg.LISR : dma.reg.HISR;
            volatile uint32_t &clr = (order < 4) ? dma.reg.LIFCR : dma.reg.HIFCR;
            const unsigned SHIFT = 6 * (order % 2) + 16 * (order % 4 > 2);
            const uint32_t FIFO_ERR_MASK = 0x1U << SHIFT;
            const uint32_t DIR_ERR_MASK = 0x4U << SHIFT;
            const uint32_t TRANS_ERR_MASK = 0x8U << SHIFT;
            if (sr & TRANS_ERR_MASK)
            {
                clr = TRANS_ERR_MASK;
                throw TransferError();
            }
            if (sr & FIFO_ERR_MASK)
            {
                clr = FIFO_ERR_MASK;
                throw FIFOError();
            }
            if (sr & DIR_ERR_MASK)
            {
                clr = DIR_ERR_MASK;
                throw DirectModeError();
            }
        }

        void global_irq_handler() const noexcept
        {
            volatile uint32_t * ifclr;
            volatile uint32_t * ifsr;
            if (order < 4)
            {
                ifclr = &dma.reg.LIFCR;
                ifsr = &dma.reg.LISR;
            }
            else
            {
                ifclr = &dma.reg.HIFCR;
                ifsr = &dma.reg.HISR;
            }

            on_complete_handler(ifclr, ifsr);
            on_fifo_err_handler(ifclr, ifsr);
            on_direct_mode_err_handler(ifclr, ifsr);
            on_transfer_err_handler(ifclr, ifsr);
            on_half_complete_handler(ifclr, ifsr);
        }
    };

    class StreamM2M : public Stream
    {
    public:

        StreamM2M(BaseDMA & dma, uint8_t order, detail::_DMAStreamReg & reg, nvic::IRQn_Type irqn) : Stream(dma, order, reg, irqn) {}

        void set_direction(Direction direction) const override
        {
            reg.CR &= ~(0x3 << 6);
            reg.CR |= (static_cast<uint8_t>(direction) << 6);
        }
    };

    BaseDMA(const uint8_t order, detail::_DMAReg & reg) : order(order), reg(reg) {}
    void init() const noexcept
    {
        clock::rcc::enable_clock(*this);
    }
    void deinit() const noexcept{
        clock::rcc::disable_clock(*this);
    }
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
};

extern const DMA Dma1;
extern const DMA_M2M Dma2;

}
}
}
