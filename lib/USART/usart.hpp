#pragma once

#include <cstdint>
#include <stdexcept>
#include <functional>
#include <type_traits>
#include <utility>
#include <iterator>
#include <concepts>
#include <ranges>
#include "property.hpp"
#include "userconfig.hpp"
#include "nvic.hpp"
#include "rcc.hpp"

namespace vermils
{
namespace stm32
{
namespace usart
{
using CallbackType = std::function<void()>;
namespace detail
{
    struct Register
    {
        #if defined(__VERMIL_STM32FX)
        volatile uint32_t SR;         /*!< USART Status register,                   Address offset: 0x00 */
        volatile uint32_t DR;         /*!< USART Data register,                     Address offset: 0x04 */
        volatile uint32_t BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
        volatile uint32_t CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
        volatile uint32_t CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
        volatile uint32_t CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
        volatile uint32_t GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
        #elif defined(__VERMIL_STM32HX)
        volatile uint32_t CR1;    /*!< USART Control register 1,                 Address offset: 0x00 */
        volatile uint32_t CR2;    /*!< USART Control register 2,                 Address offset: 0x04 */
        volatile uint32_t CR3;    /*!< USART Control register 3,                 Address offset: 0x08 */
        volatile uint32_t BRR;    /*!< USART Baud rate register,                 Address offset: 0x0C */
        volatile uint32_t GTPR;   /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
        volatile uint32_t RTOR;   /*!< USART Receiver Time Out register,         Address offset: 0x14 */
        volatile uint32_t RQR;    /*!< USART Request register,                   Address offset: 0x18 */
        volatile uint32_t ISR;    /*!< USART Interrupt and status register,      Address offset: 0x1C */
        volatile uint32_t ICR;    /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
        volatile uint32_t RDR;    /*!< USART Receive Data register,              Address offset: 0x24 */
        volatile uint32_t TDR;    /*!< USART Transmit Data register,             Address offset: 0x28 */
        volatile uint32_t PRESC;  /*!< USART clock Prescaler register,           Address offset: 0x2C */
        #endif
    };

    extern Register &reg1;
    extern Register &reg2;
    extern Register &reg3;
    extern Register &reg6;
}

enum class Parity
{
    None,
    Even,
    Odd
};

enum class StopBits : uint32_t
{
    One=0x0U, // This is the default value of number of stop bits.
    Half=0x1000U,  // To be used when receiving data in Smartcard mode.
    Two=0x2000U,  // This will be supported by normal USART, single-wire and modem modes.
    OneAndHalf=0x3000U  // To be used when transmitting and receiving data in Smartcard mode.
};

enum class WordLength
{
    Bits8=0x0000U,
    Bits9=0x1000U
};

class UsartException : public std::runtime_error
{
public:
    UsartException(const char *msg) : std::runtime_error(msg) {}
};
class OverrunError : public UsartException
{
public:
    OverrunError() : UsartException("Overrun error") {}
};
class ParityError : public UsartException
{
public:
    ParityError() : UsartException("Parity error") {}
};
class NoiseError : public UsartException
{
public:
    NoiseError() : UsartException("Noise error") {}
};
class FramingError : public UsartException
{
public:
    FramingError() : UsartException("Framing error") {}
};
class BreakError : public UsartException
{
public:
    BreakError() : UsartException("Break error") {}
};

class BaseUart
{
public:
    virtual ~BaseUart() = default;
    virtual void init() = 0;
    virtual void deinit() = 0;
    virtual void set_baudrate(float baudrate) = 0;
    virtual void set_parity(Parity parity) = 0;
    virtual Parity get_parity() const noexcept = 0;
    virtual void set_stop_bits(StopBits stop_bits) = 0;
    virtual StopBits get_stop_bits() const noexcept = 0;
    virtual void set_word_length(WordLength word_length) = 0;
    virtual WordLength get_word_length() const noexcept = 0;
    virtual void break_transmission() = 0;
    virtual size_t exchange_bytes(const void * send, size_t send_size, void * recv, size_t recv_size) = 0;
    size_t write_bytes(const void *data, size_t size)
    {
        return exchange_bytes(data, size, nullptr, 0);
    }
    size_t write(std::string_view sv)
    {
        return write_bytes(sv.data(), sv.size());
    }
    size_t read_bytes(void *data, size_t size)
    {
        return exchange_bytes(nullptr, 0, data, size);
    }
    template<typename T>
    void put(const T &data) requires(std::is_trivially_copyable_v<T>)
    {
        write_bytes(&data, sizeof(T));
    }
    template<typename T>
    T get() requires(std::is_trivially_copyable_v<T>)
    {
        T data;
        read_bytes(&data, sizeof(T));
        return data;
    }
    template<typename T>
    T& get(T& data) requires(std::is_trivially_copyable_v<T>)
    {
        read_bytes(&data, sizeof(T));
        return data;
    }
    template <std::input_iterator Iter_t, std::sentinel_for<Iter_t> Senti_t>
    size_t write(Iter_t begin, Senti_t end)
    {
        size_t count = 0;
        size_t unit_bytes = sizeof(typename std::iterator_traits<Iter_t>::value_type);
        while (begin != end)
        {
            put(*begin++);
            count += unit_bytes;
        }
        return count;
    }
    template <std::ranges::input_range Range_t>
    size_t write(const Range_t & range)
    {
        return write(std::ranges::begin(range), std::ranges::end(range));
    }

    template <typename Iter_t, typename Senti_t>
    requires std::sentinel_for<Senti_t, Iter_t> &&
             std::output_iterator<Iter_t, typename std::iterator_traits<Iter_t>::value_type>
    size_t read(Iter_t begin, Senti_t end) const
    {
        size_t count=0;
        size_t unit_bytes = sizeof(typename std::iterator_traits<Iter_t>::value_type);
        while (begin != end)
        {
            *begin = get<typename std::iterator_traits<Iter_t>::value_type>();
            ++begin;
            count += unit_bytes;
        }
        return count;
    }
    template <typename Range_t>
    requires std::ranges::output_range<Range_t, typename std::ranges::range_value_t<Range_t>>
    size_t read(Range_t && range) const
    {
        return read(std::ranges::begin(range), std::ranges::end(range));
    }

};

class HardUart : public BaseUart
{
public:
    const uint8_t order;
    const nvic::IRQn_Type irqn;
    constexpr HardUart(uint8_t order, nvic::IRQn_Type irqn) noexcept : order(order), irqn(irqn) {}
};

class HardUsart : public BaseUart
{
protected:
    template <typename T>
    struct _Property : tricks::StaticProperty<T, HardUsart &>
    {
        constexpr _Property(HardUsart &usart) noexcept : tricks::StaticProperty<T, HardUsart &>(usart) {}
        using tricks::StaticProperty<T, HardUsart &>::operator=;
    };
    struct _BaudRate : _Property<float>
    {
        constexpr _BaudRate(HardUsart &usart) noexcept : _Property<float>(usart) {}
        using _Property<float>::operator=;
        float getter() const noexcept override;
        void setter(float value) const noexcept override;
    };
public:
    enum class ExtendedMode
    {
        None,
        SmartCard,
        HalfDuplex,
        LIN,
        IrDA,
    };
    detail::Register &reg;
    const uint8_t order;
    const nvic::IRQn_Type irqn;
    bool suppress_noise_error = false;
    bool suppress_parity_error = false;
    bool suppress_overrun_error = false;
    _BaudRate baudrate{*this};
    CallbackType on_transmit_complete;
    CallbackType on_transmit_ready;
    CallbackType on_receive_ready;
    CallbackType on_overrun;
    CallbackType on_clear_to_send;
    CallbackType on_idle_line;
    CallbackType on_parity_error;
    CallbackType on_framing_error;
    CallbackType on_break_detected;
    CallbackType on_noise;
    HardUsart(detail::Register &reg, uint8_t order, nvic::IRQn_Type irqn) noexcept : reg(reg), order(order), irqn(irqn) {}
    HardUsart & operator=(const HardUsart &) = delete;
    void init() noexcept override;
    void deinit() noexcept override;
    void set_baudrate(float baudrate) noexcept override
    {
        set_baudrate(baudrate, false);
    }
    void set_parity(Parity parity) noexcept override;
    Parity get_parity() const noexcept override;
    void set_stop_bits(StopBits stop_bits) noexcept override;
    StopBits get_stop_bits() const noexcept override;
    void set_word_length(WordLength word_length) noexcept override;
    WordLength get_word_length() const noexcept override;
    void break_transmission() noexcept override;
    size_t exchange_bytes(const void * send, size_t send_size, void * recv, size_t recv_size) override;
    // size_t write(const void *data, size_t size) override;
    // size_t write(std::string_view sv) override
    // {
    //     return write(reinterpret_cast<const uint8_t*>(sv.data()), sv.size());
    // }
    // size_t read(void *data, size_t size) override;

    /**
     * @brief Trade clock deviation tolerance for higher baudrate. Allow clock to reach FCLK/8 instead of FCLK/16.
     * 
     */
    void set_oversample_8x(bool on) const noexcept;
    bool is_oversample_8x() const noexcept;
    /**
     * @brief Set the baudrate
     * 
     * @param baudrate
     * @param keep_oversampling If false, oversampling could be changed to achieve the baudrate.
     * @throw std::invalid_argument If the baudrate is not achievable.
     */
    void set_baudrate(float baudrate, bool keep_oversampling) const;
    void set_extended_mode(ExtendedMode mode) noexcept;
    ExtendedMode get_extended_mode() const noexcept;
    bool has_noise() const noexcept;
    bool has_parity_error() const noexcept;
    void raise_if_error() const;
    void set_transmitter(bool on) const noexcept;
    void set_receiver(bool on) const noexcept;
    void set_sync_clock(bool enable, bool polarity=false, bool phase=false, bool last_bit=false) const noexcept;
    void set_flow_control(bool clear_to_send_on, bool request_to_send_on) const noexcept;

    void on_transmit_complete_handler() const noexcept;
    void on_transmit_ready_handler() const noexcept;
    void on_receive_ready_handler() const noexcept;
    void on_overrun_handler() const noexcept;
    void on_clear_to_send_handler() const noexcept;
    void on_idle_line_handler() const noexcept;
    void on_parity_error_handler() const noexcept;
    void on_break_detected_handler() const noexcept;
    void on_multi_buffer_error_handler() const noexcept;

    void set_transmit_complete_interrupt(bool on=true) const noexcept;
    void set_transmit_ready_interrupt(bool on=true) const noexcept;
    void set_receiver_interrupts(bool on=true) const noexcept;
    void set_clear_to_send_interrupt(bool on=true) const noexcept;
    void set_idle_line_interrupt(bool on=true) const noexcept;
    void set_parity_error_interrupt(bool on=true) const noexcept;
    void set_break_detected_interrupt(bool on=true) const noexcept;
    void set_multi_buffer_error_interrupt(bool on=true) const noexcept;

    void enable_interrupts() const noexcept
    {
        set_transmit_complete_interrupt();
        set_transmit_ready_interrupt();
        set_receiver_interrupts();
        set_clear_to_send_interrupt();
        set_idle_line_interrupt();
        set_parity_error_interrupt();
        set_break_detected_interrupt();
        set_multi_buffer_error_interrupt();
    }

    void enable_irq() const noexcept
    {
        nvic::enable_irq(irqn);
    }

    void global_interrupt_handler() const noexcept
    {
        on_transmit_complete_handler();
        on_transmit_ready_handler();
        on_receive_ready_handler();
        on_overrun_handler();
        on_clear_to_send_handler();
        on_idle_line_handler();
        on_parity_error_handler();
        on_break_detected_handler();
        on_multi_buffer_error_handler();
    }

};

extern HardUsart Usart1;
extern HardUsart Usart2;
extern HardUsart Usart3;
extern HardUart Uart4;
extern HardUart Uart5;
extern HardUsart Usart6;

}
}
}
