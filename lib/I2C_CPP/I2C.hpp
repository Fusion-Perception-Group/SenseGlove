#ifndef _p_i2c_hpp_
#define _p_i2c_hpp_

#include <cstdint>

namespace vermils
{
namespace stm32
{
    class BaseI2C
    {
    public:
        virtual ~BaseI2C() = default;
        virtual bool select(const uint_least16_t address, const bool read) = 0;
        virtual bool write_byte(const uint8_t data) = 0;
        virtual uint8_t read_byte() = 0;
        virtual bool write(uint32_t address, const uint8_t *data, const uint32_t size) = 0;
        virtual uint32_t read(uint8_t *data, const uint32_t maxsize) = 0;
        virtual uint32_t read(uint32_t address, uint8_t *data, const uint32_t maxsize) = 0;
    };

    class SoftI2C : public BaseI2C
    {
    public:
        virtual bool select(const uint_least16_t address, const bool read) override;
        virtual bool write_byte(const uint8_t data) override;
        virtual uint8_t read_byte() override;
        virtual bool write(uint32_t address, const uint8_t *data, const uint32_t size) override;
        virtual uint32_t read(uint8_t *data, const uint32_t maxsize) override;
        virtual uint32_t read(uint32_t address, uint8_t *data, const uint32_t maxsize) override;
    };

    class HardI2C : public BaseI2C
    {
    public:
        virtual bool select(const uint_least16_t address, const bool read) override;
        virtual bool write_byte(const uint8_t data) override;
        virtual uint8_t read_byte() override;
        virtual bool write(uint32_t address, const uint8_t *data, const uint32_t size) override;
        virtual uint32_t read(uint8_t *data, const uint32_t maxsize) override;
        virtual uint32_t read(uint32_t address, uint8_t *data, const uint32_t maxsize) override;
    };
}
}
#endif