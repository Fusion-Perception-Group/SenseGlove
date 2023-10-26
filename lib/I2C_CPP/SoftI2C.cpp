#include "I2C.hpp"

#define REP0(X)
#define REP1(X) X
#define REP2(X) REP1(X) X
#define REP3(X) REP2(X) X
#define REP4(X) REP3(X) X
#define REP5(X) REP4(X) X
#define REP6(X) REP5(X) X
#define REP7(X) REP6(X) X
#define REP8(X) REP7(X) X
#define REP9(X) REP8(X) X
#define REP10(X) REP9(X) X

#define REP(HUNDREDS,TENS,ONES,X) \
  REP##HUNDREDS(REP10(REP10(X))) \
  REP##TENS(REP10(X)) \
  REP##ONES(X)

#define __I2C_NOP asm("NOP");
#define __I2C_SCL_DELAY REP(0, 0, 9, __I2C_NOP);

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
        sda.set();
        scl.set();
    }

    /**
     * @brief Writes a bit to slave
     * 
     * @param bit 
     * @return true: Written bit is the same as bit
     * @return false: Arbitration lost
     */
    inline bool SoftMaster::write_bit(const bool bit) const noexcept
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
            return false;
        }

        timer.delay_us(delay_us);
        scl.reset();
        timer.delay_us(delay_us);

        return true;
    }

    inline bool SoftMaster::read_bit() const noexcept
    {
        scl.set();

        // Clock stretching and synchronization
        while(!scl.read());

        timer.delay_us(delay_us);
        bool bit = sda.read();
        scl.reset();
        timer.delay_us(delay_us);
        return bit;
    }

    inline void SoftMaster::_start() const
    {
        sda.set();
        timer.delay_us(delay_us);
        scl.set();
        // Clock stretching and synchronization
        while(!scl.read());
        timer.delay_us(delay_us);
        sda.reset();
        timer.delay_us(delay_us);
        scl.reset();
        timer.delay_us(delay_us);
    }

    inline void SoftMaster::_terminate() const
    {
        sda.reset();
        timer.delay_us(delay_us);
        scl.set();
        while(!scl.read());
        timer.delay_us(delay_us);
        sda.set();
        timer.delay_us(delay_us);
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

    bool SoftMaster::write_byte(const uint8_t data) const
    {
        for (uint_fast8_t c = 0x80; c; c >>= 1)
        {
            if (!write_bit(data & c))
            {
                arbitration_lost = true;
                return false;
            }
        }
        sda.reset();
        return !read_bit(); // 0: ACK, 1: NACK
    }

    uint8_t SoftMaster::read_byte(const bool acknowledge) const
    {
        uint_fast8_t data = 0;
        for (uint_fast8_t c = 7; c; --c)
        {
            data <<= 1;
            data |= read_bit();
        }
        write_bit(!acknowledge);
        return data;
    }

    bool SoftMaster::detect_busy(u_int32_t timeout_us) const
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
