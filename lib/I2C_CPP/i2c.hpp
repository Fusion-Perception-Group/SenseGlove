#pragma once

#include <cstdint>
#include <exception>
#include <string>
#include <span>
#include <type_traits>
#include "property.hpp"
#include "gpio.hpp"
#include "time.hpp"
#include "macro.h"

#define __I2C_NOP asm("NOP");
#define __I2C_SCL_DELAY REP(0, 0, 7, __I2C_NOP);
#define __I2C_COM_DELAY if (delay_us) timer.delay_us(delay_us);

namespace vermils
{
namespace stm32
{
namespace i2c
{
    using addr_t = uint16_t;
    class I2CException : public std::runtime_error
    {
    public:
        I2CException(const std::string &what_arg="I2C Exception"): std::runtime_error(what_arg) {}
    };
    class ArbitrationLost : public I2CException
    {
    public:
        ArbitrationLost(
            const std::string &what_arg="ArbitrationLost during I2C communication (Signal on the bus doesn't match output value)")
            : I2CException(what_arg) {}
    };
    class I2CBusBusy : public I2CException
    {
    public:
        I2CBusBusy(
            const std::string &what_arg="I2C bus busy")
            : I2CException(what_arg) {}
    };
    class NoSlaveAck : public I2CException
    {
    public:
        NoSlaveAck(
            const std::string &what_arg="No I2C slave acknowledged the address")
            : I2CException(what_arg) {}
    };

    class BaseMaster
    {
    public:
        mutable tricks::Property<bool> arbitration_lost;
        BaseMaster(
            const tricks::Property<bool>::Getter &getter,
            const tricks::Property<bool>::Setter &setter
        ): arbitration_lost(getter, setter) {}
        virtual ~BaseMaster() {}
        /**
         * @brief Selects the slave with the given address.
         * 
         * @param address 1 or 2 byte(s) left aligned 7/10-bit address of the slave
         * @param read 
         * @note 10-bit address should include 5 bits 0xf0 header
         * @return true 
         * @return false 
         */
        virtual bool select(addr_t address, const bool read) const noexcept = 0;

        virtual void end() const = 0;
        virtual bool write_byte(uint8_t data) const noexcept = 0;
        /**
         * @throw ArbitrationLost
         * @return uint8_t 
         */
        virtual uint8_t read_byte(bool acknowledge=true) const = 0;
        /**
         * @brief Writes data to I2C bus
         * 
         * @throw `NoSlaveAck` if address is not acknowledged
         * @return size_t bytes written
         */
        virtual size_t write(addr_t address, const uint8_t *data, std::size_t size) const = 0;
        /**
         * @throw `NoSlaveAck` if address is not acknowledged
         * @throw `ArbitrationLost` if arbitration is lost during communication
         * @return std::size_t bytes read
         */
        virtual std::size_t read(addr_t address, uint8_t *data, std::size_t maxsize) const = 0;
        template <typename T>
        bool write(addr_t addr, std::span<T> data) const requires(std::is_trivially_copyable_v<T>)
        {
            return write(addr, reinterpret_cast<const uint8_t *>(data.data()), data.size_bytes());
        }
        template <typename T>
        std::size_t read(addr_t addr, std::span<T> data) const requires(std::is_trivially_copyable_v<T>)
        {
            return read(addr, reinterpret_cast<uint8_t *>(data.data()), data.size_bytes()) / sizeof(T);
        }
        template <typename T>
        void put(addr_t addr, const T &data) const requires(std::is_trivially_copyable_v<T>)
        {
            write(addr, reinterpret_cast<const uint8_t *>(&data), sizeof(T));
        }
        template <typename T>
        void get(addr_t addr, T &data) const requires(std::is_trivially_copyable_v<T>)
        {
            read(addr, reinterpret_cast<uint8_t *>(&data), sizeof(T));
        }
        /**
         * @throw Raises `ArbitrationLost` exception if `arbitration_lost` is true.
         * @param reset_flag whether to clear the flag after raising the exception
         */
        virtual void raise_if_arbitration_lost(bool reset_flag = true) const
        {
            if (arbitration_lost)
            {
                arbitration_lost = !reset_flag;
                throw ArbitrationLost();
            }
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
    class SoftMaster : public BaseMaster
    {
    protected:
        mutable bool _arbitration_lost = false;
        time::HighResTimer timer = {};
        void _start() const;
        void _terminate() const;
    public:
        const gpio::Pin &sda, &scl;
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
            const gpio::Pin &sda_, const gpio::Pin &scl_, const uint32_t delay_us_ = 0,
            bool highspeed_ = false, uint8_t master_code_ = 1,
            bool use_pp_for_hs_ = false, bool start_byte_ = false
            );
        bool detect_busy(uint32_t timeout_us=100) const;
        /**
         * @brief Writes a bit to slave
         * 
         * @param bit 
         * @return true: Written bit is the same as bit
         * @return false: Arbitration lost
         */
        void write_bit(const bool bit) const
        {
            sda.write(bit);
            __I2C_SCL_DELAY;
            scl.set();

            // Clock stretching and synchronization
            while(!scl.read());

            // Arbitration lost test
            if (bit != sda.read())
            {
                _arbitration_lost = true;
                throw ArbitrationLost();
            }

            __I2C_COM_DELAY;
            scl.reset();
            __I2C_COM_DELAY;
        }
        bool read_bit() const noexcept
        {
            sda.set();
            __I2C_SCL_DELAY;
            scl.set();

            // Clock stretching and synchronization
            while(!scl.read());

            __I2C_COM_DELAY;
            bool bit = sda.read();
            scl.reset();
            __I2C_COM_DELAY;
            return bit;  // return ack bit
        }
        bool select(addr_t address, const bool read) const noexcept override;
        void end() const override;
        bool write_byte(const uint8_t data) const noexcept override;
        uint8_t read_byte(const bool acknowledge=true) const override;
        size_t write(addr_t address, const uint8_t *data, const std::size_t size) const override;
        std::size_t read(addr_t address, uint8_t *data, const std::size_t maxsize) const override;
    };

    class HardMaster : public BaseMaster
    {
    };
}
}
}

#undef __I2C_NOP
#undef __I2C_SCL_DELAY
#undef __I2C_COM_DELAY
