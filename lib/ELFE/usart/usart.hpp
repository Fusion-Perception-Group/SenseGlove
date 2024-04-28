#pragma once

#include "_hpp_config.hpp"
#include "clock/clock.hpp"
#include "nvic/nvic.hpp"
#include "result.hpp"
#include "utils/property.hpp"
#include <concepts>
#include <cstdint>
#include <cstring>
#include <deque>
#include <functional>
#include <iterator>
#include <ranges>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace elfe {
namespace stm32 {
    namespace usart {
        using namespace err::usart;
        using CallbackType = std::function<void()>;
        using std::size_t;
        using EC = err::ErrorCode;
        namespace detail {
            struct Register {
#if defined(_ELFE_STM32FX)
                volatile uint32_t SR; /*!< USART Status register,                   Address offset: 0x00 */
                volatile uint32_t DR; /*!< USART Data register,                     Address offset: 0x04 */
                volatile uint32_t BRR; /*!< USART Baud rate register,                Address offset: 0x08 */
                volatile uint32_t CR1; /*!< USART Control register 1,                Address offset: 0x0C */
                volatile uint32_t CR2; /*!< USART Control register 2,                Address offset: 0x10 */
                volatile uint32_t CR3; /*!< USART Control register 3,                Address offset: 0x14 */
                volatile uint32_t GTPR; /*!< USART Guard time and prescaler register, Address offset: 0x18 */
#elif defined(_ELFE_STM32HX)
                volatile uint32_t CR1; /*!< USART Control register 1,                 Address offset: 0x00 */
                volatile uint32_t CR2; /*!< USART Control register 2,                 Address offset: 0x04 */
                volatile uint32_t CR3; /*!< USART Control register 3,                 Address offset: 0x08 */
                volatile uint32_t BRR; /*!< USART Baud rate register,                 Address offset: 0x0C */
                volatile uint32_t GTPR; /*!< USART Guard time and prescaler register,  Address offset: 0x10 */
                volatile uint32_t RTOR; /*!< USART Receiver Time Out register,         Address offset: 0x14 */
                volatile uint32_t RQR; /*!< USART Request register,                   Address offset: 0x18 */
                volatile uint32_t ISR; /*!< USART Interrupt and status register,      Address offset: 0x1C */
                volatile uint32_t ICR; /*!< USART Interrupt flag Clear register,      Address offset: 0x20 */
                volatile uint32_t RDR; /*!< USART Receive Data register,              Address offset: 0x24 */
                volatile uint32_t TDR; /*!< USART Transmit Data register,             Address offset: 0x28 */
                volatile uint32_t PRESC; /*!< USART clock Prescaler register,           Address offset: 0x2C */
#endif
            };

            extern Register& reg1;
            extern Register& reg2;
            extern Register& reg3;
            extern Register& reg6;
        }

        enum class Parity {
            None,
            Even,
            Odd
        };

        enum class StopBits : uint32_t {
            One = 0x0U, // This is the default value of number of stop bits.
            Half = 0x1000U, // To be used when receiving data in Smartcard mode.
            Two = 0x2000U, // This will be supported by normal USART, single-wire and modem modes.
            OneAndHalf = 0x3000U // To be used when transmitting and receiving data in Smartcard mode.
        };

        enum class WordLength {
            Bits8 = 0x0000U,
            Bits9 = 0x1000U
        };

        class BaseUart {
        public:
            uint64_t timeout_us = 0; // timeout on single byte read/write
            virtual ~BaseUart() = default;
            virtual VoidResult<> init() = 0;
            virtual void deinit() = 0;
            virtual VoidResult<> set_baudrate(float baudrate) = 0;
            virtual VoidResult<> set_parity(Parity parity) = 0;
            virtual Parity get_parity() const noexcept = 0;
            virtual VoidResult<> set_stop_bits(StopBits stop_bits) = 0;
            virtual StopBits get_stop_bits() const noexcept = 0;
            virtual VoidResult<> set_word_length(WordLength word_length) = 0;
            virtual WordLength get_word_length() const noexcept = 0;
            virtual VoidResult<> break_transmission() = 0;
            virtual Result<size_t> exchange_bytes(const void* send, size_t send_size, void* recv, size_t recv_size) = 0;
            Result<size_t> write_bytes(const void* data, size_t size)
            {
                return exchange_bytes(data, size, nullptr, 0);
            }
            Result<size_t> write(std::string_view sv)
            {
                return write_bytes(sv.data(), sv.size());
            }
            Result<size_t> write(const std::string& str)
            {
                return write_bytes(str.data(), str.size());
            }
            Result<size_t> write(const char* str)
            {
                return write_bytes(str, std::strlen(str));
            }
            Result<size_t> read_bytes(void* data, size_t size)
            {
                return exchange_bytes(nullptr, 0, data, size);
            }
            Result<std::string> read_line()
            {
                std::string str;
                char c;
                bool leading = true;
                while (true) {
                    auto r = read_bytes(&c, 1);
                    ELFE_PROP(r, Result<std::string>("", r.error));
                    if (c == '\n' or c == '\r') {
                        if (leading)
                            continue;
                        break;
                    }
                    leading = false;
                    str.push_back(c);
                }
                return str;
            }
            Result<std::vector<std::string>> read_lines(size_t lines)
            {
                std::vector<std::string> vec;
                vec.reserve(lines);
                for (size_t i = 0; i < lines; ++i) {
                    auto r = read_line();
                    ELFE_PROP(r, Result<std::vector<std::string>>(vec, r.error));
                    vec.push_back(r.value);
                }
                return vec;
            }
            template <typename T>
            VoidResult<> put(const T& data)
                requires(std::is_trivially_copyable_v<T>)
            {
                return write_bytes(&data, sizeof(T)).error;
            }
            template <typename T>
            Result<T> get()
                requires(std::is_trivially_copyable_v<T>)
            {
                T data;
                auto r = read_bytes(&data, sizeof(T));
                ELFE_PROP(r, Result<T>(data, r.error));
                return data;
            }
            template <typename T>
            Result<T&> get(T& data)
                requires(std::is_trivially_copyable_v<T>)
            {
                auto r = read_bytes(&data, sizeof(T));
                ELFE_PROP(r, Result<T&>(data, r.error));
                return data;
            }
            template <typename T>
            Result<T> replace(const T& send)
                requires(std::is_trivially_copyable_v<T>)
            {
                T new_data;
                auto r = exchange_bytes(&send, sizeof(T), &new_data, sizeof(T));
                ELFE_PROP(r, Result<T>(new_data, r.error));
                return new_data;
            }
            template <typename T>
            VoidResult<> replace(const T& send, T& recv)
                requires(std::is_trivially_copyable_v<T>)
            {
                return exchange_bytes(&send, sizeof(T), &recv, sizeof(T)).error;
            }
            template <std::input_iterator Iter_t, std::sentinel_for<Iter_t> Senti_t>
            Result<size_t> write(Iter_t begin, Senti_t end)
            {
                size_t count = 0;
                size_t unit_bytes = sizeof(typename std::iterator_traits<Iter_t>::value_type);
                while (begin != end) {
                    auto r = put(*begin++);
                    ELFE_PROP(r, Result<size_t>(count, r.error));
                    count += unit_bytes;
                }
                return count;
            }
            template <std::ranges::input_range Range_t>
            Result<size_t> write(const Range_t& range)
            {
                return write(std::ranges::begin(range), std::ranges::end(range));
            }

            template <typename Iter_t, typename Senti_t>
            Result<size_t> read(Iter_t begin, Senti_t end) const
                requires std::sentinel_for<Senti_t, Iter_t> && std::output_iterator<Iter_t, typename std::iterator_traits<Iter_t>::value_type>
            {
                size_t count = 0;
                size_t unit_bytes = sizeof(typename std::iterator_traits<Iter_t>::value_type);
                while (begin != end) {
                    auto r = get<typename std::iterator_traits<Iter_t>::value_type>();
                    ELFE_PROP(r, Result<size_t>(count, r.error));
                    *begin = r.value;
                    ++begin;
                    count += unit_bytes;
                }
                return count;
            }
            template <typename Range_t>
            Result<size_t> read(Range_t&& range) const
                requires std::ranges::output_range<Range_t, typename std::ranges::range_value_t<Range_t>>
            {
                return read(std::ranges::begin(range), std::ranges::end(range));
            }
            template <typename Iter_t, typename Senti_t>
            Result<size_t> exchange(Iter_t begin, Senti_t end) const
                requires std::sentinel_for<Senti_t, Iter_t> && std::output_iterator<Iter_t, typename std::iterator_traits<Iter_t>::value_type>
            {
                size_t count = 0;
                size_t unit_bytes = sizeof(typename std::iterator_traits<Iter_t>::value_type);
                while (begin != end) {
                    auto r = replace<typename std::iterator_traits<Iter_t>::value_type>(*begin);
                    ELFE_PROP(r, Result<size_t>(count, r.error));
                    *begin = r.value;
                    ++begin;
                    count += unit_bytes;
                }
                return count;
            }
        };

        class HardUart : public BaseUart {
        public:
            const uint8_t order;
            const nvic::IRQn_Type irqn;
            constexpr HardUart(uint8_t order, nvic::IRQn_Type irqn) noexcept
                : order(order)
                , irqn(irqn)
            {
            }
        };

        class HardUsart : public BaseUart {
        protected:
            template <typename T>
            struct _Property : tricks::StaticProperty<T, HardUsart&> {
                constexpr _Property(HardUsart& usart) noexcept
                    : tricks::StaticProperty<T, HardUsart&>(usart)
                {
                }
                using tricks::StaticProperty<T, HardUsart&>::operator=;
            };
            struct _BaudRate : _Property<float> {
                constexpr _BaudRate(HardUsart& usart) noexcept
                    : _Property<float>(usart)
                {
                }
                using _Property<float>::operator=;
                float getter() const noexcept override;
                void setter(float value) const noexcept override;
            };
            bool _buffer_enabled = false;
            size_t _tx_buffer_max_size = 0;
            size_t _rx_buffer_max_size = 0;
            mutable volatile bool _rx_buffer_overrun = false;

        public:
            enum class ExtendedMode {
                None,
                SmartCard,
                HalfDuplex,
                LIN,
                IrDA,
            };
            detail::Register& reg;
            const uint8_t order;
            const nvic::IRQn_Type irqn;
            bool suppress_noise_error = false;
            bool suppress_parity_error = false;
            bool suppress_overrun_error = false;
            _BaudRate baudrate { *this };
            std::deque<uint8_t> tx_buffer;
            std::deque<uint8_t> rx_buffer;
            CallbackType on_transmit_complete;
            CallbackType on_transmit_ready;
            CallbackType on_receive_ready; // belongs to receive_related interrupt
            CallbackType on_clear_to_send;
            CallbackType on_idle_line;
            CallbackType on_parity_error;
            CallbackType on_break_detected;
            CallbackType on_noise; // belongs to multi_buffer_error interrupt
            CallbackType on_framing_error; // belongs to multi_buffer_error interrupt
            CallbackType on_overrun; // belongs to multi_buffer_error and receive_related interrupt
            HardUsart(detail::Register& reg, uint8_t order, nvic::IRQn_Type irqn) noexcept
                : reg(reg)
                , order(order)
                , irqn(irqn)
            {
            }
            HardUsart& operator=(const HardUsart&) = delete;
            VoidResult<> init() noexcept override;
            void deinit() noexcept override;
            VoidResult<> set_baudrate(float baudrate) noexcept override
            {
                return set_baudrate(baudrate, false);
            }
            VoidResult<> set_parity(Parity parity) noexcept override;
            Parity get_parity() const noexcept override;
            VoidResult<> set_stop_bits(StopBits stop_bits) noexcept override;
            StopBits get_stop_bits() const noexcept override;
            VoidResult<> set_word_length(WordLength word_length) noexcept override;
            WordLength get_word_length() const noexcept override;
            VoidResult<> break_transmission() noexcept override;
            Result<size_t> exchange_bytes(const void* send, size_t send_size, void* recv, size_t recv_size) override
            {
                if (_buffer_enabled)
                    return buffered_exchange_bytes(send, send_size, recv, recv_size);
                else
                    return sync_exchange_bytes(send, send_size, recv, recv_size);
            }
            Result<size_t> sync_exchange_bytes(const void* send, size_t send_size, void* recv, size_t recv_size);
            Result<size_t> buffered_exchange_bytes(const void* send, size_t send_size, void* recv, size_t recv_size);

            bool is_buffer_enabled() const noexcept
            {
                return _buffer_enabled;
            }

            VoidResult<> enable_buffer(size_t tx_size = 128, size_t rx_size = 128)
            {
                ELFE_ERROR_IF(
                    not((get_word_length() == WordLength::Bits8) ^ // 8 bit mode
                        (get_parity() == Parity::None)),
                    EC::InvalidArgument,
                    std::invalid_argument("Buffered mode only support 8 bit data"));
                _buffer_enabled = true;
                _tx_buffer_max_size = tx_size;
                _rx_buffer_max_size = rx_size;
                on_receive_ready = [this]() noexcept {
                    uint8_t data = reg.DR;
                    if (_rx_buffer_max_size == rx_buffer.size()) {
                        _rx_buffer_overrun = true;
                        return;
                    }
                    rx_buffer.push_back(data);
                };
                on_transmit_ready = [this]() noexcept {
                    if (tx_buffer.empty()) {
                        disable_interrupt_transmit_ready();
                        return;
                    }
                    reg.DR = tx_buffer.front();
                    tx_buffer.pop_front();
                };
                auto r = nvic::set_priority(irqn, 1);
                ELFE_PROP(r, r);
                enable_interrupt_receive_related();
                return EC::None;
            }

            void disable_buffer() noexcept
            {
                _buffer_enabled = false;
                on_receive_ready = nullptr;
                on_transmit_ready = nullptr;
            }

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
            VoidResult<> set_baudrate(float baudrate, bool keep_oversampling) const;
            void set_extended_mode(ExtendedMode mode) noexcept;
            ExtendedMode get_extended_mode() const noexcept;
            bool has_noise() const noexcept;
            bool has_parity_error() const noexcept;
            VoidResult<> raise_if_error() const;
            void set_transmitter(bool on) const noexcept;
            void set_receiver(bool on) const noexcept;
            void set_sync_clock(bool enable, bool polarity = false, bool phase = false, bool last_bit = false) const noexcept;
            void set_flow_control(bool clear_to_send_on, bool request_to_send_on) const noexcept;

            void on_transmit_complete_handler() const noexcept;
            void on_transmit_ready_handler() const noexcept;
            void on_receive_related_handler() const noexcept;
            void on_overrun_handler() const noexcept;
            void on_clear_to_send_handler() const noexcept;
            void on_idle_line_handler() const noexcept;
            void on_parity_error_handler() const noexcept;
            void on_break_detected_handler() const noexcept;
            void on_multi_buffer_error_handler() const noexcept;

            void enable_interrupt_transmit_complete() const noexcept;
            void enable_interrupt_transmit_ready() const noexcept;
            void enable_interrupt_receive_related() const noexcept;
            void enable_interrupt_clear_to_send() const noexcept;
            void enable_interrupt_idle_line() const noexcept;
            void enable_interrupt_parity_error() const noexcept;
            void enable_interrupt_break_detected() const noexcept;
            void enable_interrupt_multi_buffer_error() const noexcept;
            void disable_interrupt_transmit_complete() const noexcept;
            void disable_interrupt_transmit_ready() const noexcept;
            void disable_interrupt_receive_related() const noexcept;
            void disable_interrupt_clear_to_send() const noexcept;
            void disable_interrupt_idle_line() const noexcept;
            void disable_interrupt_parity_error() const noexcept;
            void disable_interrupt_break_detected() const noexcept;
            void disable_interrupt_multi_buffer_error() const noexcept;

            void enable_interrupts() const noexcept
            {
                enable_interrupt_transmit_complete();
                enable_interrupt_transmit_ready();
                enable_interrupt_receive_related();
                enable_interrupt_clear_to_send();
                enable_interrupt_idle_line();
                enable_interrupt_parity_error();
                enable_interrupt_break_detected();
                enable_interrupt_multi_buffer_error();
            }
            void disable_interrupts() const noexcept
            {
                nvic::disable_irq(irqn);
                disable_interrupt_transmit_complete();
                disable_interrupt_transmit_ready();
                disable_interrupt_receive_related();
                disable_interrupt_clear_to_send();
                disable_interrupt_idle_line();
                disable_interrupt_parity_error();
                disable_interrupt_break_detected();
                disable_interrupt_multi_buffer_error();
            }

            VoidResult<> set_irq_priority(const uint8_t priority = 8) const
            {
                return nvic::set_priority(irqn, priority);
            }

            void global_interrupt_handler() const noexcept
            {
                on_transmit_complete_handler();
                on_transmit_ready_handler();
                on_receive_related_handler();
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
