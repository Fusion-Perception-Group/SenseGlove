/**
 * @file GPIO.cpp
 * @brief This file contains the implementation of the GPIOPin class, which provides an interface for configuring and controlling GPIO pins on STM32 microcontrollers.
 * 
 * The GPIOPin class provides methods for enabling/disabling the clock for a GPIO port, setting the pin configuration (input/output/alternate function/analog), 
 * configuring the pull-up/pull-down resistors, configuring the alternate function, and configuring the external interrupt/event for the GPIO pin.
 * 
 * The implementation of the GPIOPin class is based on the STM32 HAL library.
 * 
 * @author Vermil
 * @date 2023-10-24
 */
#include "GPIO.hpp"

namespace vermils
{
namespace stm32
{

/**
 * @brief Enables the clock for the GPIO port associated with this GPIOPin object.
 * 
 * This function enables the clock for the GPIO port associated with this GPIOPin object.
 * 
 * @note This function will be called by the GPIOPin::set() function or when initialised with GPIOPinConfig object.
 * @note This function is inline.
 */
inline void GPIOPin::enable_clock() const
{
    if (port == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();
    }
    #ifdef GPIOB
    else if (port == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
    }
    #ifdef GPIOC
    else if (port == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_ENABLE();
    }
    #ifdef GPIOD
    else if (port == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_ENABLE();
    }
    #ifdef GPIOE
    else if (port == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_ENABLE();
    }
    #ifdef GPIOF
    else if (port == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_ENABLE();
    }
    #ifdef GPIOG
    else if (port == GPIOG)
    {
        __HAL_RCC_GPIOG_CLK_ENABLE();
    }
    #ifdef GPIOH
    else if (port == GPIOH)
    {
        __HAL_RCC_GPIOH_CLK_ENABLE();
    }
    #endif
    #endif
    #endif
    #endif
    #endif
    #endif
    #endif
}

inline void GPIOPin::disable_clock() const
{
    if (port == GPIOA)
    {
        __HAL_RCC_GPIOA_CLK_DISABLE();
    }
    #ifdef GPIOB
    else if (port == GPIOB)
    {
        __HAL_RCC_GPIOB_CLK_DISABLE();
    }
    #ifdef GPIOC
    else if (port == GPIOC)
    {
        __HAL_RCC_GPIOC_CLK_DISABLE();
    }
    #ifdef GPIOD
    else if (port == GPIOD)
    {
        __HAL_RCC_GPIOD_CLK_DISABLE();
    }
    #ifdef GPIOE
    else if (port == GPIOE)
    {
        __HAL_RCC_GPIOE_CLK_DISABLE();
    }
    #ifdef GPIOF
    else if (port == GPIOF)
    {
        __HAL_RCC_GPIOF_CLK_DISABLE();
    }
    #ifdef GPIOG
    else if (port == GPIOG)
    {
        __HAL_RCC_GPIOG_CLK_DISABLE();
    }
    #ifdef GPIOH
    else if (port == GPIOH)
    {
        __HAL_RCC_GPIOH_CLK_DISABLE();
    }
    #endif
    #endif
    #endif
    #endif
    #endif
    #endif
    #endif
}

/**
 * @brief Sets the configuration of a GPIO pin.
 * 
 * @param config The configuration to set.
 * @return const GPIOPinConfig& The configuration that was set.
 */
const GPIOPinConfig & GPIOPin::set(const GPIOPinConfig & config) const
{
    uint32_t temp;
    enable_clock();

    // Config output/AF mode
    if (config.io == GPIOPinConfig::Output ||
        config.io == GPIOPinConfig::AF)
    {
        // Config speed
        temp = port->OSPEEDR;
        temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pin * 2));
        temp |= (static_cast<uint32_t>(config.speed) << (pin * 2));
        port->OSPEEDR = temp;

        temp = port->OTYPER;
        temp &= ~(GPIO_OTYPER_OT_0 << pin);
        temp |= (static_cast<uint32_t>(config.out_mode) << pin);
        port->OTYPER = temp;
    }

    // Config pull mode
    if (config.io != GPIOPinConfig::Analog)
    {
        temp = port->PUPDR;
        temp &= ~(GPIO_PUPDR_PUPDR0 << (pin * 2));
        temp |= (static_cast<uint32_t>(config.pull) << (pin * 2));
        port->PUPDR = temp;
    }

    // Config Alternate function
    if (config.io == GPIOPinConfig::AF)
    {
        temp = port->AFR[pin >> 3];
        temp &= ~(static_cast<uint32_t>(0xFU) << ((pin & 0x7) << 2));
        temp |= (static_cast<uint32_t>(config.alternate) << ((pin & 0x7) << 2));
        port->AFR[pin >> 3] = temp;
    }

    // Configure IO Direction mode (Input, Output, Alternate or Analog)
    temp = port->MODER;
    temp &= ~(GPIO_MODER_MODER0 << (pin * 2));
    temp |= (static_cast<uint32_t>(config.io) << (pin * 2));
    port->MODER = temp;

    // Configure the External Interrupt or event for GPIO
    if (config.exti_mode != GPIOPinConfig::NoEXTI)
    {
        // Enable SYSCFG Clock
        __HAL_RCC_SYSCFG_CLK_ENABLE();

        uint32_t mask = 1 << pin;
        temp = SYSCFG->EXTICR[pin >> 2];
        temp &= ~(0x0FU << (4 * (pin & 0x03)));
        temp |= (uint32_t(GPIO_GET_INDEX(port)) << (4 * (pin & 0x03)));
        SYSCFG->EXTICR[pin >> 2] = temp;

        // Clear EXTI line configuration
        temp = EXTI->IMR;
        temp &= ~uint32_t(mask);
        if (config.exti_mode == GPIOPinConfig::Interrupt)
        {
            temp |= uint32_t(mask);
        }
        EXTI->IMR = temp;

        temp = EXTI->EMR;
        temp &= ~uint32_t(mask);
        if (config.exti_mode == GPIOPinConfig::Event)
        {
            temp |= uint32_t(mask);
        }
        EXTI->EMR = temp;

        // Clear Rising Falling edge configuration
        temp = EXTI->RTSR;
        temp &= ~uint32_t(mask);
        if (config.trigger == GPIOPinConfig::Rising ||
            config.trigger == GPIOPinConfig::RisingFalling)
        {
            temp |= uint32_t(mask);
        }
        EXTI->RTSR = temp;

        temp = EXTI->FTSR;
        temp &= ~uint32_t(mask);
        if (config.trigger == GPIOPinConfig::Falling ||
            config.trigger == GPIOPinConfig::RisingFalling)
        {
            temp |= uint32_t(mask);
        }
        EXTI->FTSR = temp;
    }

    return config;
}

const GPIOPinConfig GPIOPin::set() const
{
    return set(GPIOPinConfig());
}

void GPIOPin::reset() const
{
    uint32_t temp;

    // EXTI configuration
    temp = SYSCFG->EXTICR[pin >> 2];
    temp &= ~(0x0FU << (4 * (pin & 0x03)));
    if (temp == (uint32_t(GPIO_GET_INDEX(port)) << (4 * (pin & 0x03))))
    {
        // Clear EXTI line configuration
        EXTI->IMR &= ~uint32_t(pin);
        EXTI->EMR &= ~uint32_t(pin);

        // Clear Rising Falling edge configuration
        EXTI->RTSR &= ~uint32_t(pin);
        EXTI->FTSR &= ~uint32_t(pin);

        // Clear EXTI for current io
        temp = 0x0FU << (4 * (pin & 0x03));
        SYSCFG->EXTICR[pin >> 2] &= ~temp;
    }

    // GPIO Mode configuration
    port->MODER &= ~(GPIO_MODER_MODER0 << (pin * 2));
    port->AFR[pin >> 3] &= ~(uint32_t(0xFU) << ((pin & 0x7) << 2));
    port->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (pin * 2));
    port->OTYPER &= ~(GPIO_OTYPER_OT_0 << pin);
    port->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pin * 2));
}
    
}
}