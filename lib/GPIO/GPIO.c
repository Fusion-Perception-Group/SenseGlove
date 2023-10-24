#include "GPIO.h"

void init_GPIO(GPIO_TypeDef *GPIOx, uint32_t pin, uint32_t mode, uint32_t pull, uint32_t speed)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = pull;
    GPIO_InitStruct.Speed = speed;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void deinit_GPIO(GPIO_TypeDef *GPIOx, uint32_t pin)
{
    HAL_GPIO_DeInit(GPIOx, pin);
}

void enable_GPIO_CLK(GPIO_TypeDef *GPIOx)
{
    if (GPIOx == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    #ifdef GPIOB
    else if (GPIOx == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    #ifdef GPIOC
    else if (GPIOx == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    #ifdef GPIOD
    else if (GPIOx == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    #ifdef GPIOE
    else if (GPIOx == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
    #ifdef GPIOF
    else if (GPIOx == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
    #ifdef GPIOG
    else if (GPIOx == GPIOG)
    {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    }
    #endif
    #endif
    #endif
    #endif
    #endif
    #endif
}

void disable_GPIO_CLK(GPIO_TypeDef *GPIOx)
{
    if (GPIOx == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_DISABLE();
    }
    #ifdef GPIOB
    else if (GPIOx == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_DISABLE();
    }
    #ifdef GPIOC
    else if (GPIOx == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_DISABLE();
    }
    #ifdef GPIOD
    else if (GPIOx == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_DISABLE();
    }
    #ifdef GPIOE
    else if (GPIOx == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_DISABLE();
    }
    #ifdef GPIOF
    else if (GPIOx == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_DISABLE();
    }
    #ifdef GPIOG
    else if (GPIOx == GPIOG)
    {
        __HAL_RCC_GPIOG_CLK_DISABLE();
    }
    #endif
    #endif
    #endif
    #endif
    #endif
    #endif
}
