#pragma once

#include <cstdint>
#include <stdexcept>
#include <ranges>
#include <span>
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

enum class Error
{
    None=0,
    Unknown,
    ReadProtected,
    WriteProtected,
    InvalidConfig,  // registers are not configured properly
    InvalidAddr,    // address is not valid
    ParallelSizeMismatch,
    AlignmentError,
};

using ErrorCallbackType = std::function<void(Error)>;

bool enable_instruction_cache(); // enable instruction cache if available
bool enable_data_cache(); // enable data cache if available
inline bool enable_prefetch() // enable prefetch buffer if available
{
    return enable_instruction_cache() && enable_data_cache();
}

class FlashError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

class ValidationError : public FlashError
{
public:
    using FlashError::FlashError;
};

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
    virtual void write(addr_t addr, const void * data, size_t bytes) = 0;
    /**
     * @throw FlashError
     * @throw invalid_argument if addr is not valid
     */
    virtual void read(addr_t addr, void * data, size_t bytes) const = 0;
    /**
     * @throw FlashError
     * @throw invalid_argument if addr is not valid
     */
    virtual void erase(addr_t addr, size_t bytes) = 0;
    /**
     * @throw FlashError
     */
    virtual void erase_all() = 0;
    template <typename T>
    void write(addr_t addr, std::span<T> data)
    {
        write(addr, data.data(), data.size_bytes());
    }
    template <typename T>
    void put(addr_t addr, const T & item)
    {
        write(addr, &item, sizeof(T));
    }
    template <typename T>
    void read(addr_t addr, std::span<T> data) const
    {
        read(addr, data.data(), data.size_bytes());
    }
    template <typename T>
    T get(addr_t addr) const requires(std::is_trivially_copyable_v<T>)
    {
        T item;
        read(addr, &item, sizeof(T));
        return item;
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
    void write(addr_t addr, const void * data, size_t bytes) override;
    void read(addr_t addr, void * data, size_t bytes) const override;
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

    void enable_on_error_interrupt() const noexcept;
    void disable_on_error_interrupt() const noexcept;
    void enable_on_complete_interrupt() const noexcept;
    void disable_on_complete_interrupt() const noexcept;
    void enable_interrupts() const noexcept
    {
        enable_on_error_interrupt();
        enable_on_complete_interrupt();
    }
    void disable_interrupts() const noexcept
    {
        disable_on_error_interrupt();
        disable_on_complete_interrupt();
    }
    void enable_irq() const noexcept
    {
        nvic::enable_irq(irqn);
    }
    void disable_irq() const noexcept
    {
        nvic::disable_irq(irqn);
    }

    Error get_error() const noexcept;
    void clear_error() const noexcept;
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
