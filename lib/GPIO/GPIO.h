#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define GPIO_PIN(x) (1<<(x))
#define GPIO_PORT(x) ((GPIO_TypeDef *)(GPIOA_BASE + (GPIOB_BASE-GPIOA_BASE)*(x)))
#define ENABLE_GPIO_CLK(x) __HAL_RCC_GPIO##x##_CLK_ENABLE()
#define DISABLE_GPIO_CLK(x) __HAL_RCC_GPIO##x##_CLK_DISABLE()
#define SET_GPIO_PIN(GPIOx, pin) {GPIOx->BSRR = pin;}
#define RESET_GPIO_PIN(GPIOx, pin) {GPIOx->BSRR = pin << 16U;}
#define SET_BSRR_PIN(BSRRp, x) {*BSRRp = x;}
#define RESET_BSRR_PIN(BSRRp, x) {*BSRRp = x << 16U;}

static inline void qWritePin(GPIO_TypeDef * const GPIOx, const uint32_t pin, const GPIO_PinState state)
{
    //HAL_GPIO_WritePin(GPIOx, pin, state);
    if (state != GPIO_PIN_RESET)
        GPIOx->BSRR = pin;
    else
        GPIOx->BSRR = pin << 16U;
}

static inline void qTogglePin(GPIO_TypeDef * const GPIOx, const uint32_t pin)
{
    //HAL_GPIO_TogglePin(GPIOx, pin);
    GPIOx->ODR ^= pin;
}

static inline GPIO_PinState qReadPin(const GPIO_TypeDef * const GPIOx, const uint32_t pin)
{
    //return HAL_GPIO_ReadPin(GPIOx, pin);
    return (GPIO_PinState)((GPIO_PinState)(GPIOx->IDR & pin) != GPIO_PIN_RESET);
}

static inline void WritePinBSRR(uint32_t * const BSRRp, const uint32_t pin, const GPIO_PinState state)
{
    if (state != GPIO_PIN_RESET)
        *BSRRp = pin;
    else
        *BSRRp = pin << 16U;
}

void init_GPIO(GPIO_TypeDef *GPIOx, uint32_t pin, uint32_t mode, uint32_t pull, uint32_t speed);

void deinit_GPIO(GPIO_TypeDef *GPIOx, uint32_t pin);

void enable_GPIO_CLK(GPIO_TypeDef *GPIOx);

void disable_GPIO_CLK(GPIO_TypeDef *GPIOx);

#ifdef __cplusplus
}
#endif
