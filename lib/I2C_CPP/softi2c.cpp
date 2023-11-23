#include "i2c.hpp"

#define __I2C_NOP asm("NOP");
#define __I2C_SCL_DELAY REP(0, 0, 7, __I2C_NOP);
#define __I2C_COM_DELAY if (delay_us) timer.delay_us(delay_us);

namespace vermils
{
namespace stm32
{
namespace i2c
{
    SoftMaster::SoftMaster(
        const gpio::Pin &sda_, const gpio::Pin &scl_, const uint32_t delay_us_,
        bool highspeed_, uint8_t master_code_,
        bool use_pp_for_hs_, bool start_byte_
        ):
        BaseMaster(
            [this]() -> bool { return _arbitration_lost; },
            [this](bool value) { _arbitration_lost = value; }
        ), sda(sda_), scl(scl_), delay_us(delay_us_), highspeed(highspeed_), master_code(master_code_),
        use_pp_for_hs(use_pp_for_hs_), start_byte(start_byte_)
    {
        sda.port.enable_clock();
        scl.port.enable_clock();
        sda.io = gpio::PinConfig::IO::Output;
        scl.io = gpio::PinConfig::IO::Output;
        sda.out_mode = gpio::PinConfig::OutMode::OpenDrain;
        scl.out_mode = gpio::PinConfig::OutMode::OpenDrain;
        end();  // Avoid triggering a start condition and release SDA/SCL
    }

    inline void SoftMaster::_start() const
    {
        sda.set();
        __I2C_SCL_DELAY;
        __I2C_COM_DELAY;
        scl.set();
        // Clock stretching and synchronization
        while(!scl.read());
        __I2C_SCL_DELAY;
        __I2C_COM_DELAY;
        sda.reset();
        __I2C_SCL_DELAY;
        __I2C_COM_DELAY;
        scl.reset();
        __I2C_COM_DELAY;
    }

    inline void SoftMaster::_terminate() const
    {
        sda.reset();
        __I2C_SCL_DELAY;
        __I2C_COM_DELAY;
        scl.set();
        while(!scl.read());
        __I2C_SCL_DELAY;
        __I2C_COM_DELAY;
        sda.set();
        __I2C_COM_DELAY;
    }

    class OutputModeChanger
    {
    public:
        const gpio::Pin &pin;
        const gpio::PinConfig::OutMode ori_mode;
        OutputModeChanger(const gpio::Pin &pin, const gpio::PinConfig::OutMode mode):
            pin(pin), ori_mode(pin.out_mode)
        {
            pin.out_mode = mode;
        }
        ~OutputModeChanger()
        {
            pin.out_mode = ori_mode;
        }
    };

    bool SoftMaster::select(I2CAddrType address, const bool read) const
    {
        if (start_byte)
        {
            _start();
            write_byte(1);
        }

        if (highspeed)
        {
            uint32_t ori_delay_us = delay_us;
            uint32_t & delay_ref = const_cast<uint32_t &>(delay_us);
            delay_ref = 3; // 400KHz max
            _start();
            write_byte(master_code & 0x3);
            delay_ref = ori_delay_us;
            
            if (use_pp_for_hs)
            {
                OutputModeChanger sda_changer(sda, gpio::PinConfig::OutMode::PushPull);
                OutputModeChanger scl_changer(scl, gpio::PinConfig::OutMode::PushPull);
            }
        }

        bool ack;
        _start();
        // Determine 7/10 bit address
        if ((address & 0xf800) == 0xf000)
        {
            // 10 bit address
            // 5 bits(1111 0) + first 2 bits of address + W bit(W mode required) + last 8 bits of address
            address = (address & 0xfe00) | ((address & 0x1fe) >> 1);
            ack = write_byte(address >> 8);
            if (!ack)
                return false;

            ack = write_byte((address & 0xff));
            if (!ack)
                return false;
            
            if (read)
            {
                _start();
                ack = write_byte(address >> 8 | 1);
                if (!ack)
                    return false;
            }
        }
        else
        {
            // 7 bit address
            if (!(address & 0xff))
                // In case address is left aligned to uint16_t
                address >>= 8;
            ack = write_byte(address | read);
            if (!ack)
                return false;
        }

        return true;
    }

    inline bool SoftMaster::write_byte(const uint8_t data) const noexcept
    {
        try
        {
            write_bit(data & 0x80);
            write_bit(data & 0x40);
            write_bit(data & 0x20);
            write_bit(data & 0x10);
            write_bit(data & 0x08);
            write_bit(data & 0x04);
            write_bit(data & 0x02);
            write_bit(data & 0x01);
        }
        catch (const ArbitrationLost &e)
        {
            sda.set(); // Release SDA
            return false;
        }
        sda.set(); // Release SDA
        return !read_bit(); // 0: ACK, 1: NACK
    }

    inline uint8_t SoftMaster::read_byte(const bool acknowledge) const
    {
        uint_fast8_t data = 0;
        data |= read_bit();
        data <<= 1;
        data |= read_bit();
        data <<= 1;
        data |= read_bit();
        data <<= 1;
        data |= read_bit();
        data <<= 1;
        data |= read_bit();
        data <<= 1;
        data |= read_bit();
        data <<= 1;
        data |= read_bit();
        data <<= 1;
        data |= read_bit();
        write_bit(!acknowledge);
        return data;
    }

    bool SoftMaster::detect_busy(uint32_t timeout_us) const
    {
        uint32_t start = timer.get_us();
        sda.set();
        timer.delay_ns(delay_us);
        scl.set();
        timer.delay_ns(delay_us);

        do
        {
            if (!scl.read() || !sda.read())
                return false;
        }
        while (timer.get_us() - start < timeout_us);

        return true;
    }

    /**
     * @brief write data to I2C bus
     * 
     * @param address 
     * @param data 
     * @param size 
     * @return `bool`
     * @throw `ArbitrationLost`
     */
    bool SoftMaster::write(I2CAddrType address, const uint8_t *data, const std::size_t size) const
    {
        if (!select(address, false))
            return false;

        try
        {
            for (uint_fast8_t i = 0; i < size; ++i)
            {
                if (!write_byte(data[i]))
                    return false;
                raise_if_arbitration_lost(false);
            }
        }
        catch (const ArbitrationLost &e)
        {
            return false;
        }
        end();
        return true;
    }

    /**
     * @brief read data from I2C bus
     * 
     * @param address 
     * @param data 
     * @param maxsize 
     * @return `std::size_t`
     * @throw `ArbitrationLost`
     */
    std::size_t SoftMaster::read(I2CAddrType address, uint8_t *data, const std::size_t maxsize) const
    {
        std::size_t size = 0;

        if (!select(address, true))
            return 0;
        
        try
        {
            for (; size < maxsize; ++size)
            {
                data[size] = read_byte(size < maxsize - 1);
                raise_if_arbitration_lost(false);
            }
        }
        catch (const ArbitrationLost &e)
        {
            return size;
        }
        end();
        
        return size;
    }

    void SoftMaster::end() const
    {
        _terminate();
    }
}
}
}
