#include "I2C.h"
/**
 * @brief Initializes I2C GPIOs, returns true if successful
 *
 * @param i2c
 * @return bool: Whether delay system init is successful
 */
bool init_I2C(const I2C_t *i2c)
{
    enable_GPIO_CLK(i2c->GPIOx);
    init_GPIO(
        i2c->GPIOx,
        i2c->sda | i2c->scl,
        GPIO_MODE_OUTPUT_OD,
        GPIO_PULLUP,
        GPIO_SPEED_HIGH
        );
    if (i2c->delay_us)
        return DWT_delay_init();
    return true;
}

void _start_I2C(const I2C_t *i2c)
{
    qWritePin(i2c->GPIOx, i2c->sda, GPIO_PIN_SET);
    DWT_delay_us(i2c->delay_us);
    qWritePin(i2c->GPIOx, i2c->scl, GPIO_PIN_SET);
    while (qReadPin(i2c->GPIOx, i2c->scl) == GPIO_PIN_RESET);
    DWT_delay_us(i2c->delay_us);
    qWritePin(i2c->GPIOx, i2c->sda, GPIO_PIN_RESET);
    DWT_delay_us(i2c->delay_us);
    qWritePin(i2c->GPIOx, i2c->scl, GPIO_PIN_RESET);
    DWT_delay_us(i2c->delay_us);
}

void _terminate_I2C(const I2C_t *i2c)
{
    qWritePin(i2c->GPIOx, i2c->sda, GPIO_PIN_RESET);
    DWT_delay_us(i2c->delay_us);
    qWritePin(i2c->GPIOx, i2c->scl, GPIO_PIN_SET);
    while (qReadPin(i2c->GPIOx, i2c->scl) == GPIO_PIN_RESET);
    DWT_delay_us(i2c->delay_us);
    qWritePin(i2c->GPIOx, i2c->sda, GPIO_PIN_SET);
    DWT_delay_us(i2c->delay_us);
}

/**
 * @brief  Writes a byte to slave
 *
 * @param  i2c: I2C_t struct
 * @param  byte: byte to write
 * @retval bool: Whether is acknowledged
 */
bool wbyte_I2C(I2C_t *i2c, const uint8_t byte)
{
    for (uint_fast8_t c = 0x80; c; c >>= 1)
    {
        if (!wbit_I2C(i2c, (bool)(byte & c)))
        {
            i2c->arb_lost = true; // set arbitration lost flag
            return false;
        }
    }
    SET_GPIO_PIN(i2c->GPIOx, i2c->sda); // release SDA
    return !rbit_I2C(i2c); // 0: ACK, 1: NACK
}

/**
 * @brief Reads a byte from slave
 *
 * @param i2c
 * @param acknowledge
 * @return uint8_t
 */
uint8_t rbyte_I2C(const I2C_t *i2c, const bool acknowledge)
{
    uint8_t byte = 0;
    for (uint_fast8_t c = 7; c; c--)
    {
        byte |= (uint8_t)rbit_I2C(i2c) << c;
    }
    wbit_I2C(i2c, !acknowledge);
    return byte;
}

/**
 * @brief Reads a bit from slave and acknowledges it
 *
 * @param i2c
 * @return uint8_t
 */
uint8_t rbyte_I2C_ack(const I2C_t *i2c)
{
    uint8_t byte = 0;
    for (uint32_t c = 7; c; c--)
    {
        byte |= (uint8_t)rbit_I2C(i2c) << c;
    }
    wbit_I2C(i2c, 0);
    return byte;
}

/**
 * @brief Reads a bit from slave and does not acknowledge it
 *
 * @param i2c
 * @return uint8_t
 */
uint8_t rbyte_I2C_nack(const I2C_t *i2c)
{
    uint8_t byte = 0;
    for (uint32_t c = 7; c; c--)
    {
        byte |= (uint8_t)rbit_I2C(i2c) << c;
    }
    wbit_I2C(i2c, 1);
    return byte;
}

/**
 * @brief Select slaves by 7-bit address
 *
 * @param i2c
 * @param address: 7-bit address, left aligned(appended with 0)
 * @param read: 0 for write, 1 for read. Use macros:
 *       @arg I2C_WRITE
 *       @arg I2C_READ
 * @return true: NACK bit
 * @return false: ACK bit
 */
bool select_I2C(I2C_t *i2c, const uint8_t address, const bool read)
{
    _start_I2C(i2c);
    bool ack = wbyte_I2C(i2c, address | read);
    return ack;
}

/**
 * @brief Select slaves by 10-bit address
 *
 * @param i2c
 * @param address: 10-bit address, left aligned(appended with 0s)
 * @param read: 0 for write, 1 for read
 * @return bool: Whether is acknowledged
 */
bool select_10b_I2C(I2C_t *i2c, uint32_t address, const bool read)
{
    _start_I2C(i2c);
    address = (address >> 5) | read | 0xF000;
    bool ack = wbyte_I2C(i2c, address >> 8);
    if (ack)
    {
        ack = wbyte_I2C(i2c, address & 0xFF);
    }
    return ack;
}

/**
 * @brief Writes bytes to slave
 *
 * @param i2c
 * @param address
 * @param data
 * @param size
 * @return true: Successful
 * @return false: Target not found or arbitration lost
 */
bool write_I2C(I2C_t *i2c, const uint8_t address, const uint8_t data[], const uint_fast32_t size)
{
    if (!select_I2C(i2c, address, I2C_WRITE))
    {
        return false;
    }
    for (uint_fast32_t i = 0; i < size; i++)
    {
        if (!wbyte_I2C(i2c, data[i]) && (i != size - 1 || i2c->arb_lost))
            return false;
    }
    _terminate_I2C(i2c);
    return true;
}

/**
 * @brief Reads bytes from slave
 *
 * @param i2c
 * @param address
 * @param data
 * @param maxsize: Maximum number of bytes to read, sends NACK at the end
 * @return uint32_t: Number of bytes read
 */
uint_fast32_t read_I2C(I2C_t *i2c, const uint8_t address, uint8_t *data, const uint_fast32_t maxsize)
{
    if (!select_I2C(i2c, address, I2C_READ))
    {
        return 0;
    }
    uint_fast32_t size = 0;
    for (; size < maxsize; size++)
    {
        data[size] = rbyte_I2C(i2c, size < maxsize - 1);
    }
    _terminate_I2C(i2c);
    return size;
}

/**
 * @brief Benchmark I2C speed
 * 
 * @param i2c 
 * @param size_byte 
 * @return double: kbit/s
 */
double benchmark_I2C(I2C_t *i2c, const unsigned size_byte)
{
    DWT_delay_init();
    _start_I2C(i2c);
    uint32_t start = DWT->CYCCNT, end;
    for (unsigned i = 0; i < size_byte; ++i)
    {
        wbyte_I2C(i2c, (uint8_t)0xe5);
    }
    end = DWT->CYCCNT;
    double sec = (double)(end - start) / HAL_RCC_GetHCLKFreq();
    _terminate_I2C(i2c);
    return (double)size_byte * 8 / 1000 / sec;
}