#pragma once

#include "./macro.h"
#include "_hpp_config.hpp"
#include "clock/clock.hpp"
#include "errors.hpp"
#include "gpio/gpio.hpp"
#include "result.hpp"
#include "utils/property.hpp"
#include <concepts>
#include <cstdint>
#include <exception>
#include <functional>
#include <ranges>
#include <string>
#include <type_traits>

#define __I2C_NOP asm("NOP");
#define __I2C_SCL_DELAY REP(0, 4, 0, __I2C_NOP);
#define __I2C_COM_DELAY \
    if (delay_us)       \
        clock::delay(delay_us);

namespace elfe {
namespace stm32 {
    namespace i2c {
        using namespace err::i2c;
        using EC = err::ErrorCode;
        using addr_t = uint16_t;
        using std::size_t;

        namespace detail {
            struct Register {
                volatile uint32_t CR1; /*!< I2C Control register 1,     Address offset: 0x00 */
                volatile uint32_t CR2; /*!< I2C Control register 2,     Address offset: 0x04 */
                volatile uint32_t OAR1; /*!< I2C Own address register 1, Address offset: 0x08 */
                volatile uint32_t OAR2; /*!< I2C Own address register 2, Address offset: 0x0C */
#if defined(_ELFE_STM32FX)
                volatile uint32_t DR; /*!< I2C Data register,          Address offset: 0x10 */
                volatile uint32_t SR1; /*!< I2C Status register 1,      Address offset: 0x14 */
                volatile uint32_t SR2; /*!< I2C Status register 2,      Address offset: 0x18 */
                volatile uint32_t CCR; /*!< I2C Clock control register, Address offset: 0x1C */
                volatile uint32_t TRISE; /*!< I2C TRISE register,         Address offset: 0x20 */
                volatile uint32_t FLTR; /*!< I2C FLTR register,          Address offset: 0x24 */
#elif defined(_ELFE_STM32HX)
                volatile uint32_t TIMINGR; /*!< I2C Timing register,               Address offset: 0x10 */
                volatile uint32_t TIMEOUTR; /*!< I2C Timeout register,              Address offset: 0x14 */
                volatile uint32_t ISR; /*!< I2C Interrupt and status register, Address offset: 0x18 */
                volatile uint32_t ICR; /*!< I2C Interrupt clear register,      Address offset: 0x1C */
                volatile uint32_t PECR; /*!< I2C PEC register,                  Address offset: 0x20 */
                volatile uint32_t RXDR; /*!< I2C Receive data register,         Address offset: 0x24 */
                volatile uint32_t TXDR; /*!< I2C Transmit data register,        Address offset: 0x28 */
#endif
            };

            extern Register& reg1;
            extern Register& reg2;
            extern Register& reg3;
            extern Register& reg4;
        }

        enum class Speed : uint8_t {
            Standard = 0,
            Fast = 1,
            FastPlus = 2,
            HighSpeed = 3
        };

        class BaseMaster {
        protected:
            virtual bool is_arbitration_lost() const noexcept = 0;
            virtual void clear_arbitration_lost() const noexcept = 0;
            template <typename T>
            struct _Property : tricks::StaticProperty<T, BaseMaster&> {
                constexpr _Property(BaseMaster& master)
                    : tricks::StaticProperty<T, BaseMaster&>(master)
                {
                }
                using tricks::StaticProperty<T, BaseMaster&>::operator=;
            };
            struct _ArbitrationLost : _Property<bool> {
                constexpr _ArbitrationLost(BaseMaster& master)
                    : _Property<bool>(master)
                {
                }
                using _Property<bool>::operator=;
                bool getter() const noexcept override
                {
                    return owner.is_arbitration_lost();
                }
                void setter(bool value) const noexcept override
                {
                    if (!value)
                        owner.clear_arbitration_lost();
                }
            };

        public:
            _ArbitrationLost arbitration_lost { *this };
            BaseMaster() = default;
            virtual ~BaseMaster() { }
            virtual void set_speed(Speed speed) = 0;
            virtual Speed get_speed() const noexcept = 0;
            /**
             * @brief Selects the slave with the given address.
             *
             * @param address 1 or 2 byte(s) left aligned 7/10-bit address of the slave
             * @param read
             * @note 10-bit address should include 5 bits 0xf0 header
             * @return true
             * @return false
             */
            virtual Result<bool> select(addr_t address, const bool read) const = 0;

            virtual void end() const noexcept = 0;
            /**
             * @brief
             *
             * @param data
             * @return Result<bool> Whether the slave acknowledged the byte
             * @throw ArbitrationLost
             */
            virtual Result<bool> write_byte(uint8_t data) const = 0;
            /**
             * @throw ArbitrationLost
             * @return Result<uint8_t>
             */
            virtual Result<uint8_t> read_byte(bool acknowledge = true) const = 0;
            /** @brief write data to I2C bus
             *
             * @param address
             * @param data
             * @param size
             * @throw `NoSlaveAck` if address is not acknowledged
             * @throw `I2CError` if any error occurs
             */
            virtual Result<size_t> write_bytes(addr_t address, const uint8_t* data, const size_t size) const
            {
                auto rb = select(address, false);
                ELFE_PROP(rb, Result<size_t>(0, rb.error));
                ELFE_ERROR_IF(
                    !rb.value,
                    Result<size_t>(0, EC::I2CNoSlaveAck),
                    NoSlaveAck());
                size_t i = 0;

                for (; i < size; ++i) {
                    if (not write_byte(data[i]))
                        break;
                    auto r = raise_if_error();
                    ELFE_PROP(r, Result<size_t>(i, r.error));
                }
                end();
                return i;
            }
            /**
             * @throw `NoSlaveAck` if address is not acknowledged
             * @throw `ArbitrationLost` if arbitration is lost during communication
             * @return Result<size_t> bytes read
             * @throw `I2CError` if any error occurs
             */
            virtual Result<size_t> read_bytes(addr_t address, uint8_t* data, size_t maxsize) const
            {
                size_t size = 0;

                ELFE_ERROR_IF(
                    !select(address, true),
                    Result<size_t>(0, EC::I2CNoSlaveAck),
                    NoSlaveAck());

                for (; size < maxsize; ++size) {
                    auto rb = read_byte(size < maxsize - 1);
                    ELFE_PROP(rb, Result<size_t>(size, rb.error));
                    data[size] = rb.value;
                    auto r = raise_if_error();
                    ELFE_PROP(r, Result<size_t>(size, r.error));
                }
                end();

                return size;
            }
            std::vector<addr_t> scan(const addr_t start = 0x0, const addr_t end = 0xFE) const
            {
                std::vector<addr_t> addrs;
                for (addr_t addr = start; addr <= end; addr += 2) {
                    if (select(addr, false)) {
                        addrs.push_back(addr);
                    }
                    this->end();
                }
                return addrs;
            }
            template <typename T>
            VoidResult<> put(addr_t addr, const T& data) const
                requires(std::is_trivially_copyable_v<T>)
            {
                return write_bytes(addr, reinterpret_cast<const uint8_t*>(&data), sizeof(T)).error;
            }
            template <typename T>
            Result<T> get(addr_t addr) const
                requires(std::is_trivially_copyable_v<T>)
            {
                T data;
                auto r = read_bytes(addr, reinterpret_cast<uint8_t*>(&data), sizeof(T));
                ELFE_PROP(r, Result<T>(data, r.error));
                return data;
            }
            template <typename T>
            VoidResult<> get(addr_t addr, T& data) const
                requires(std::is_trivially_copyable_v<T>)
            {
                return read_bytes(addr, reinterpret_cast<uint8_t*>(&data), sizeof(T)).error;
            }
            template <std::input_iterator Iter_t, std::sentinel_for<Iter_t> Senti_t>
            size_t write(addr_t addr, Iter_t begin, Senti_t end)
            {
                const addr_t start_addr = addr;
                size_t unit_bytes = sizeof(typename std::iterator_traits<Iter_t>::value_type);
                while (begin != end) {
                    put(addr, *begin++);
                    addr += unit_bytes;
                }
                return addr - start_addr;
            }

            template <std::ranges::input_range Range_t>
            size_t write(addr_t addr, const Range_t& range)
            {
                return write(addr, std::ranges::begin(range), std::ranges::end(range));
            }

            template <typename Iter_t, typename Senti_t>
                requires std::sentinel_for<Senti_t, Iter_t> && std::output_iterator<Iter_t, typename std::iterator_traits<Iter_t>::value_type>
            size_t read(addr_t addr, Iter_t begin, Senti_t end) const
            {
                const addr_t start_addr = addr;
                size_t unit_bytes = sizeof(typename std::iterator_traits<Iter_t>::value_type);
                while (begin != end) {
                    *begin = get<typename std::iterator_traits<Iter_t>::value_type>(addr);
                    ++begin;
                    addr += unit_bytes;
                }
                return addr - start_addr;
            }
            template <typename Range_t>
                requires std::ranges::output_range<Range_t, typename std::ranges::range_value_t<Range_t>>
            size_t read(addr_t addr, Range_t&& range) const
            {
                return read(addr, std::ranges::begin(range), std::ranges::end(range));
            }
            /**
             * @throw Raises `ArbitrationLost` exception if `arbitration_lost` is true.
             * @param reset_flag whether to clear the flag after raising the exception
             */
            virtual VoidResult<> raise_if_arbitration_lost(bool reset_flag = true) const
            {
                if (arbitration_lost) {
                    arbitration_lost = !reset_flag;
                    ELFE_ERROR(EC::I2CArbitrationLost, ArbitrationLost());
                }
                return EC::None;
            }
            virtual VoidResult<> raise_if_error(const bool end = true) const
            try {
                auto r = raise_if_arbitration_lost();
                [[unlikely]] if (!r.ok() && end)
                    this->end();
                return r;
            } catch (const std::exception& e) {
                if (end)
                    this->end();
                throw e;
            }
        };

        /**
         * @brief Software I2C implementation.
         *
         * @param sda
         * @param scl
         * @param delay_us = 0
         * @param highspeed = false High Speed Mode
         * @param master_code = 1 Master code used in HS mode
         * @param use_pp_for_hs = false Whether to use Push-Pull mode for High Speed Mode
         * @param start_byte = false Whether to send start byte before address
         */
        class SoftMaster : public BaseMaster {
        protected:
            mutable bool _arbitration_lost = false;
            void _start() const;
            void _terminate() const;
            virtual bool is_arbitration_lost() const noexcept override
            {
                return _arbitration_lost;
            }
            virtual void clear_arbitration_lost() const noexcept override
            {
                _arbitration_lost = false;
            }

        public:
            const gpio::Pin sda, scl;
            uint32_t delay_us;
            // Whether to enable High Speed Mode
            bool highspeed;
            // Master code used in HS mode
            uint8_t master_code;
            // Whether to use Push-Pull mode for High Speed Mode
            bool use_pp_for_hs;
            // Whether to send start byte before address
            bool start_byte;
            SoftMaster(
                const gpio::Pin& sda, const gpio::Pin& scl, const uint32_t delay_us = 0,
                bool highspeed = false, uint8_t master_code = 1,
                bool use_pp_for_hs = false, bool start_byte = false);
            void set_speed(Speed speed) noexcept
            {
                switch (speed) {
                case Speed::Standard:
                    delay_us = 4;
                    break;
                case Speed::Fast:
                    delay_us = 1;
                    break;
                case Speed::FastPlus:
                    delay_us = 0;
                    break;
                case Speed::HighSpeed:
                    delay_us = 0;
                    highspeed = true;
                    break;
                }
            }
            Speed get_speed() const noexcept
            {
                if (highspeed)
                    return Speed::HighSpeed;
                if (delay_us == 0)
                    return Speed::FastPlus;
                if (delay_us == 1)
                    return Speed::Fast;
                return Speed::Standard;
            }
            bool detect_busy(const uint32_t timeout_us = 100) const;
            /**
             * @brief Writes a bit to slave
             *
             * @param bit
             * @return true: Written bit is the same as bit
             * @return false: Arbitration lost
             */
            VoidResult<> write_bit(const bool bit) const
            {
                sda.write(bit);
                __I2C_SCL_DELAY;
                scl.set();

                // Clock stretching and synchronization
                while (!scl.read())
                    ;

                // Arbitration lost test
                if (bit != sda.read()) {
                    _arbitration_lost = true;
                    ELFE_ERROR(EC::I2CArbitrationLost, ArbitrationLost());
                }

                __I2C_COM_DELAY;
                scl.reset();
                __I2C_COM_DELAY;
                return EC::None;
            }
            bool read_bit() const noexcept
            {
                sda.set();
                __I2C_SCL_DELAY;
                scl.set();

                // Clock stretching and synchronization
                while (!scl.read())
                    ;

                __I2C_COM_DELAY;
                bool bit = sda.read();
                scl.reset();
                __I2C_COM_DELAY;
                return bit; // return ack bit
            }
            Result<bool> select(addr_t address, const bool read) const noexcept override;
            void end() const noexcept override;
            /**
             * @brief write a byte
             *
             * @param data
             * @return Result<bool> whether the slave acknowledged the byte
             * @throw ArbitrationLost
             */
            Result<bool> write_byte(const uint8_t data) const override;
            /**
             * @throw ArbitrationLost
             * @return Result<uint8_t>
             */
            Result<uint8_t> read_byte(const bool acknowledge = true) const override;
            Result<size_t> write_bytes(addr_t address, const uint8_t* data, const size_t size) const override;
            Result<size_t> read_bytes(addr_t address, uint8_t* data, const size_t maxsize) const override;
        };

        enum class Event {
            None,
            StartBitSent,
            AddressSentOrMatched,
            Address10bitHeaderSent,
            StopReceived,
            DataByteFinished,
            ReceiverAvailable,
            TransmitterAvailable,
        };

        using OnEventCallback = std::function<void(Event)>;
        /*
       Possible Error Codes:
       - None
       - I2CBusError
       - I2CArbitrationLost
       - I2CNoSlaveAck
       - I2COverrun
       - I2CValidationError
       - I2CTimeout
       - I2CSMBusAlert
       */
        using OnErrorCallback = std::function<void(EC)>;

        class HardMaster : public BaseMaster {
            Speed _speed = Speed::Standard;

        protected:
            template <typename T>
            struct _Property : tricks::StaticProperty<T, HardMaster&> {
                constexpr _Property(HardMaster& master)
                    : tricks::StaticProperty<T, HardMaster&>(master)
                {
                }
                using tricks::StaticProperty<T, HardMaster&>::operator=;
            };
            struct _ClockSpeed : _Property<uint32_t> {
                constexpr _ClockSpeed(HardMaster& master)
                    : _Property<uint32_t>(master)
                {
                }
                using _Property<uint32_t>::operator=;
                uint32_t getter() const noexcept override;
                void setter(uint32_t value) const noexcept override;
            };
            struct _MaxRiseTime : _Property<std::chrono::nanoseconds> {
                constexpr _MaxRiseTime(HardMaster& master)
                    : _Property<std::chrono::nanoseconds>(master)
                {
                }
                using _Property<std::chrono::nanoseconds>::operator=;
                std::chrono::nanoseconds getter() const noexcept override;
                void setter(std::chrono::nanoseconds value) const noexcept override;
            };
            bool is_arbitration_lost() const noexcept override;
            void clear_arbitration_lost() const noexcept override;

        public:
            enum class ExtendedMode {
                None,
                SMBusDevice,
                SMBusHost
            };
            enum class FMDutyCycle {
                _2_1 = 0,
                _16_9 = 1
            };
            detail::Register& reg;
            const uint8_t order;
            const nvic::IRQn_Type ev_irqn;
            const nvic::IRQn_Type err_irqn;
            OnEventCallback on_event;
            OnErrorCallback on_error;
            _ClockSpeed clock_speed { *this };
            _MaxRiseTime max_rise_time { *this };
            HardMaster(
                detail::Register& reg_, const uint8_t order_,
                const nvic::IRQn_Type ev_irqn_, const nvic::IRQn_Type err_irqn_)
                : reg(reg_)
                , order(order_)
                , ev_irqn(ev_irqn_)
                , err_irqn(err_irqn_)
            {
            }

            void init();
            void deinit() noexcept;
            void set_speed(Speed speed) override;
            Speed get_speed() const noexcept override;
            Result<bool> select(addr_t address, const bool read) const override;

            void end() const noexcept override;
            /**
             * @brief write a byte
             *
             * @param data
             * @return Result<bool> whether the slave acknowledged the byte
             * @throw ArbitrationLost
             */
            Result<bool> write_byte(uint8_t data) const override;
            /**
             * @throw ArbitrationLost
             * @return Result<uint8_t>
             */
            Result<uint8_t> read_byte(bool acknowledge = true) const override;
            /**
             * @brief raise exception if error occurred
             *
             * @param end whether to end communication
             */
            VoidResult<> raise_if_error(const bool end = true) const override;

            void set_clock_strech(bool enable) const noexcept;
            bool get_clock_strech() const noexcept;
            void set_analog_filter(bool enable) const noexcept;
            bool get_analog_filter() const noexcept;
            void set_digital_filter(uint8_t cycles) const noexcept;
            uint8_t get_digital_filter() const noexcept;
            void set_fast_mode_duty_cycle(FMDutyCycle duty_cycle) const noexcept;
            FMDutyCycle get_fast_mode_duty_cycle() const noexcept;
            void set_extended_mode(ExtendedMode mode) const noexcept;
            ExtendedMode get_extended_mode() const noexcept;
            void set_packet_error_checking(bool enable) const noexcept;
            bool get_packet_error_checking() const noexcept;

            void enable_interrupt_event() const noexcept;
            void disable_interrupt_event() const noexcept;
            void enable_interrupt_event_buffer() const noexcept;
            void disable_interrupt_event_buffer() const noexcept;
            void enable_interrupt_error() const noexcept;
            void disable_interrupt_error() const noexcept;

            void enable_interrupts() const noexcept
            {
                enable_interrupt_event();
                enable_interrupt_event_buffer();
                enable_interrupt_error();
            }
            void disable_interrupts() const noexcept
            {
                nvic::disable_irq(ev_irqn);
                // nvic::disable_interrupt_irq(err_irqn); // in disable_error()
                disable_interrupt_event();
                disable_interrupt_event_buffer();
                disable_interrupt_error();
            }
            VoidResult<> set_irq_priority(const uint8_t ev_priority, const uint8_t err_priority) const
            {
                auto r = nvic::set_priority(ev_irqn, ev_priority);
                ELFE_PROP(r, r);
                return nvic::set_priority(err_irqn, err_priority);
            }
            VoidResult<> set_irq_priority(const uint8_t priority = 8) const
            {
                return set_irq_priority(priority, priority);
            }
            void on_event_handler() const noexcept;
            void on_error_handler() const noexcept;

            void global_interrupt_handler() const noexcept
            {
                on_event_handler();
                on_error_handler();
            }
        };

        extern HardMaster I2c1;
        extern HardMaster I2c2;
        extern HardMaster I2c3;
        extern HardMaster I2c4;

    }
}
}
