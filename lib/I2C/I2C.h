#pragma once
#ifdef __cplusplus
extern "C"
{
#endif

#define I2C_WRITE 0
#define I2C_READ 1
#define I2C_ACK 0
#define I2C_NACK 1
#define I2C_ArbitrationLost 2

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "DWT_Delay.h"
#include "GPIO.h"
#include "misc.h"

//#define I2C_AFAP
#define __I2C_NOP asm("NOP");

#ifdef I2C_AFAP
#define I2C_Delay(x)
#define I2C_WaitState_OR_NOWAIT(pin, state)
#define I2C_WaitState_OR_NOP(pin, state) REP(0, 0, 2, __I2C_NOP)
#else
#define I2C_Delay(x) if (x) DWT_delay_us(x)
#define I2C_WaitState_OR_NOWAIT(pin, state) while(qReadPin(GPIOx, pin) != state)
#define I2C_WaitState_OR_NOP(pin, state) while(qReadPin(GPIOx, pin) != state)
#endif

typedef struct {
    GPIO_TypeDef * const GPIOx;
    const uint32_t sda;
    const uint32_t scl;
    const uint32_t delay_us; // delay for I2C, 100kbps: 10us, 400kbps: 2.5us
    bool arb_lost; // Arbitration lost flag
} I2C_t;

/**
 * @brief Writes a bit to slave
 * 
 * @param i2c 
 * @param bit 
 * @return true: Written bit is the same as bit
 * @return false: Arbitration lost
 */
static inline bool wbit_I2C(const I2C_t* i2c, bool bit)
{
    register GPIO_TypeDef * const GPIOx = i2c->GPIOx;
    register volatile uint32_t * const BSRR = &GPIOx->BSRR;
    register const uint32_t sda = i2c->sda, scl = i2c->scl, delay_us = i2c->delay_us;
    qWritePin(GPIOx, sda, (GPIO_PinState)bit);
    REP(0, 0, 9, __I2C_NOP);  // wait a few cycles for GPIOx->ODR to be updated

    //HAL_GPIO_WritePin(GPIOx, scl, GPIO_PIN_SET);
    //qWritePin(GPIOx, scl, GPIO_PIN_SET);
    SET_BSRR_PIN(BSRR, scl);
    // For clock stretching and synchronization
    I2C_WaitState_OR_NOWAIT(scl, GPIO_PIN_SET);

    if (qReadPin(GPIOx, sda) != (GPIO_PinState)bit)
        return false; // Arbitration lost

    I2C_Delay(delay_us);
    RESET_BSRR_PIN(BSRR, scl);
    I2C_Delay(delay_us);
    return true;
}

static inline bool rbit_I2C(const I2C_t* i2c)
{
    register GPIO_TypeDef * const GPIOx = i2c->GPIOx;
    register volatile uint32_t * const BSRR = &GPIOx->BSRR;
    register const uint32_t sda = i2c->sda, scl = i2c->scl, delay_us = i2c->delay_us;
    SET_BSRR_PIN(BSRR, scl);
    // For clock stretching and synchronization
    I2C_WaitState_OR_NOP(scl, GPIO_PIN_SET);

    I2C_Delay(delay_us);
    bool bit = (bool)qReadPin(GPIOx, sda);
    RESET_BSRR_PIN(BSRR, scl);
    I2C_Delay(delay_us);
    return bit;
}

bool init_I2C(const I2C_t *i2c);

void _start_I2C(const I2C_t *i2c);

void _terminate_I2C(const I2C_t *i2c);

bool wbyte_I2C(I2C_t *i2c, const uint8_t byte);

uint8_t rbyte_I2C(const I2C_t *i2c, const bool acknowledge);

uint8_t rbyte_I2C_ack(const I2C_t *i2c);

uint8_t rbyte_I2C_nack(const I2C_t *i2c);

bool select_I2C(I2C_t *i2c, const uint8_t address, const bool read);

bool select_10b_I2C(I2C_t *i2c, uint16_t address, const bool read);

bool write_I2C(I2C_t *i2c, const uint8_t address, const uint8_t data[], const uint_fast32_t size);

uint_fast32_t read_I2C(I2C_t *i2c, const uint8_t address, uint8_t *data, const uint_fast32_t maxsize);

double benchmark_I2C(I2C_t *i2c, const unsigned size_byte);

#ifdef __cplusplus
}
#endif
