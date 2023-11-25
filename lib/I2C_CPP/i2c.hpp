#pragma once

#include <cstdint>
#include <exception>
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
    typedef uint16_t I2CAddrType;
    class I2CException : std::exception
    {
    public:
        const char *what() const noexcept override
        {
            return "I2CException";
        }
    };
    class ArbitrationLost : I2CException
    {
    public:
        const char *what() const noexcept override
        {
            return "ArbitrationLost during I2C communication (Signal on the bus doesn't match output value)";
        }
    };

    class InvalidI2CAddress : I2CException
    {
    public:
        const char *what() const noexcept override
        {
            return "InvalidI2CAddress during I2C communication";
        }
    };

    class I2CBusBusy : I2CException
    {
    public:
        const char *what() const noexcept override
        {
            return "I2CBusBusy during I2C communication";
        }
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
        virtual bool select(I2CAddrType address, const bool read) const = 0;
        /**
         * @brief Ends the communication with the slave.
         * 
         */
        virtual void end() const = 0;
        virtual bool write_byte(uint8_t data) const = 0;
        virtual uint8_t read_byte(bool acknowledge=true) const = 0;
        virtual bool write(I2CAddrType address, const uint8_t *data, std::size_t size) const = 0;
        virtual std::size_t read(I2CAddrType address, uint8_t *data, std::size_t maxsize) const = 0;
        /**
         * @brief Raises `ArbitrationLost` exception if `arbitration_lost` is true.
         * 
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
        bool select(I2CAddrType address, const bool read) const override;
        void end() const override;
        bool write_byte(const uint8_t data) const noexcept override;
        uint8_t read_byte(const bool acknowledge=true) const override;
        bool write(I2CAddrType address, const uint8_t *data, const std::size_t size) const override;
        std::size_t read(I2CAddrType address, uint8_t *data, const std::size_t maxsize) const override;
    };

    //class HardI2C : public BaseI2C
    //{
    //};
}
}
}

#undef __I2C_NOP
#undef __I2C_SCL_DELAY
#undef __I2C_COM_DELAY
