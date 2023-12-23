#pragma once

#include <cstdint>
#include <stdexcept>
#include <iterator>
#include <ranges>
#include <concepts>
#include <functional>
#include <utility>
#include "userconfig.hpp"
#include "property.hpp"
#include "nvic.hpp"
#include "flash_f1_detail.hpp"
#include "flash_f4_detail.hpp"
#include "flash_hx_detail.hpp"
#include "flash_generic_detail.hpp"


namespace vermils
{
namespace stm32
{
namespace flash
{
using std::size_t;
using addr_t = std::uintptr_t;
using CompleteCallbackType = std::function<void()>;
using UnitRange = std::pair<addr_t, addr_t>;

class FlashException : public std::runtime_error
{
public:
    FlashException(const char * msg="Flash Exception") : std::runtime_error(msg) {}
};

using ErrorCallbackType = std::function<void(const FlashException&)>;

class ReadProtected : public FlashException
{
public:
    ReadProtected() : FlashException("Attempt to write read protected flash address") {}
};

class WriteProtected : public FlashException
{
public:
    WriteProtected() : FlashException("Attempt to write write protected flash address") {}
};

class InvalidConfig : public FlashException
{
public:
    InvalidConfig() : FlashException("Flash registers are not configured properly") {}
};

class InvalidAddr : public FlashException
{
public:
    InvalidAddr() : FlashException("Invalid flash address") {}
};

class ParallelSizeMismatch : public FlashException
{
public:
    ParallelSizeMismatch() : FlashException("Parallel size mismatch") {}
};

class AlignmentError : public FlashException
{
public:
    AlignmentError() : FlashException("Alignment error") {}
};

class ValidationError : public FlashException
{
public:
    ValidationError(const char * msg="Flash validation error") : FlashException(msg) {}
};

bool enable_instruction_cache(); // enable instruction cache if available
bool enable_data_cache(); // enable data cache if available
inline bool enable_prefetch() // enable prefetch buffer if available
{
    return enable_instruction_cache() && enable_data_cache();
}


consteval unsigned long long operator ""_Bytes(unsigned long long size)
{
    return size;
}
consteval unsigned long long operator ""_KiB(unsigned long long size)
{
    return size * 1024;
}

class BaseFlash
{
protected:
public:
    bool validate = false;
    virtual ~BaseFlash() = default;
    virtual bool is_valid_range(addr_t addr_start, size_t size) const noexcept = 0;
    /**
     * @brief returns affected unit range in start and end address, a unit is a smallest unit that can be erased at procided address
     * 
     * @param addr 
     * @return UnitRange 
     * @throw invalid_argument if addr is not valid
     */
    virtual UnitRange get_unit_range(addr_t addr) const = 0;
    /**
     * @throw invalid_argument if addr is not valid
     * @throw FlashError (std::runtime_error)
     * @throw ValidationError (FlashError & std::runtime_error)
     */
    virtual void write_bytes(addr_t addr, const void * data, size_t bytes) = 0;
    /**
     * @throw FlashError
     * @throw invalid_argument if addr is not valid
     */
    virtual void read_bytes(addr_t addr, void * data, size_t bytes) const = 0;
    /**
     * @throw FlashError
     * @throw invalid_argument if addr is not valid
     */
    virtual void erase(addr_t addr, size_t bytes) = 0;
    /**
     * @throw FlashError
     */
    virtual void erase_all() = 0;
    virtual void raise_if_error(bool clear=true) const = 0;
    template <typename T>
    void put(addr_t addr, const T & item)
    {
        write_bytes(addr, &item, sizeof(T));
    }
    template <typename T>
    T get(addr_t addr) const requires(std::is_trivially_copyable_v<T>)
    {
        T item;
        read_bytes(addr, &item, sizeof(T));
        return item;
    }
    template <std::input_iterator Iter_t, std::sentinel_for<Iter_t> Senti_t>
    size_t write(addr_t addr, Iter_t begin, Senti_t end)
    {
        const addr_t start_addr = addr;
        size_t unit_bytes = sizeof(typename std::iterator_traits<Iter_t>::value_type);
        while (begin != end)
        {
            put(addr, *begin++);
            addr += unit_bytes;
        }
        return addr - start_addr;
    }
    
    template <std::ranges::input_range Range_t>
    size_t write(addr_t addr, const Range_t & range)
    {
        return write(addr, std::ranges::begin(range), std::ranges::end(range));
    }

    template <typename Iter_t, typename Senti_t>
    requires std::sentinel_for<Senti_t, Iter_t> &&
             std::output_iterator<Iter_t, typename std::iterator_traits<Iter_t>::value_type>
    size_t read(addr_t addr, Iter_t begin, Senti_t end) const
    {
        const addr_t start_addr = addr;
        size_t unit_bytes = sizeof(typename std::iterator_traits<Iter_t>::value_type);
        while (begin != end)
        {
            *begin = get<typename std::iterator_traits<Iter_t>::value_type>(addr);
            ++begin;
            addr += unit_bytes;
        }
        return addr - start_addr;
    }
    template <typename Range_t>
    requires std::ranges::output_range<Range_t, typename std::ranges::range_value_t<Range_t>>
    size_t read(addr_t addr, Range_t && range) const
    {
        return read(addr, std::ranges::begin(range), std::ranges::end(range));
    }
};

class EmbeddedFlash : public BaseFlash
{
protected:
    template <typename T>
    struct _Property : public tricks::StaticProperty<T, EmbeddedFlash &>
    {
        constexpr _Property(EmbeddedFlash & owner) : tricks::StaticProperty<T, EmbeddedFlash &>(owner) {}
        using tricks::StaticProperty<T, EmbeddedFlash &>::operator=;
    };
    template <typename T>
    struct _ROProperty : public tricks::StaticReadOnlyProperty<T, EmbeddedFlash &>
    {
        constexpr _ROProperty(EmbeddedFlash & owner) : tricks::StaticReadOnlyProperty<T, EmbeddedFlash &>(owner) {}
        using tricks::StaticReadOnlyProperty<T, EmbeddedFlash &>::operator=;
    };
public:
    static constexpr const nvic::IRQn_Type irqn = nvic::IRQn_Type::FLASH_IRQn;
    detail::FlashRegister &reg = detail::flash_reg;
    ErrorCallbackType on_error;
    CompleteCallbackType on_complete;
    bool is_valid_range(addr_t addr_start, size_t size) const noexcept override;
    UnitRange get_unit_range(addr_t addr) const override;
    void write_bytes(addr_t addr, const void * data, size_t bytes) override;
    void read_bytes(addr_t addr, void * data, size_t bytes) const override;
    void erase(addr_t addr, size_t bytes) override;
    void erase_all() override;

    void set_latency(const uint8_t latency) const noexcept
    {
        reg.ACR = (reg.ACR & ~0xFU) | (latency & 0xFU);
    }

    uint8_t get_latency() const noexcept
    {
        return reg.ACR & 0xFU;
    }

    void enable_interrupt_error() const noexcept;
    void disable_interrupt_error() const noexcept;
    void enable_interrupt_complete() const noexcept;
    void disable_interrupt_complete() const noexcept;
    void enable_interrupts() const noexcept
    {
        enable_interrupt_error();
        enable_interrupt_complete();
    }
    void disable_interrupts() const noexcept
    {
        nvic::disable_irq(irqn);
        disable_interrupt_error();
        disable_interrupt_complete();
    }
    void set_irq_priority(const uint8_t priority=8) const
    {
        nvic::set_priority(irqn, priority);
    }

    void clear_error() const noexcept;
    void raise_if_error(bool clear=true) const override final;
    void on_error_handler() const noexcept;
    void on_complete_handler() const noexcept;

    void global_interrupt_handler() const noexcept
    {
        on_error_handler();
        on_complete_handler();
    }
};

inline EmbeddedFlash Flash;
void test(addr_t);

}
}
}
