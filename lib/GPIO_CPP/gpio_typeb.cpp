/**
 * @file GPIO.cpp
 * @brief This file contains the implementation of the Pin class, which provides an interface for configuring and controlling GPIO pins on STM32 microcontrollers.
 * 
 * The Pin class provides methods for enabling/disabling the clock for a GPIO port, setting the pin configuration (input/output/alternate function/analog), 
 * configuring the pull-up/pull-down resistors, configuring the alternate function, and configuring the external interrupt/event for the GPIO pin.
 * 
 * The implementation of the Pin class is based on the STM32 HAL library.
 * 
 * @author Vermil
 * @date 2023-10-24
 */
#include "_config.hpp"
#include "gpio.hpp"

#if (defined(__VERMIL_STM32F4) || defined(__VERMIL_STM32H7)) && !__VERMIL_STM32_USE_GENERIC && !VERMIL_STM32_USE_CMSIS

#if defined(__VERMIL_STM32F4)
#define EXTI_IMR (EXTI->IMR)
#define EXTI_EMR (EXTI->EMR)
#define EXTI_RTSR (EXTI->RTSR)
#define EXTI_FTSR (EXTI->FTSR)

#elif defined(__VERMIL_STM32H7)

#if defined(DUAL_CORE) && defined(CORE_CM4)
  EXTI_CurrentCPU = EXTI_D2; /* EXTI for CM4 CPU */
#else
  EXTI_CurrentCPU = EXTI_D1; /* EXTI for CM7 CPU */
#endif
#define EXTI_IMR (EXTI_CurrentCPU->IMR1)
#define EXTI_EMR (EXTI_CurrentCPU->EMR1)
#define EXTI_RTSR (EXTI_CurrentCPU->RTSR1)
#define EXTI_FTSR (EXTI_CurrentCPU->FTSR1)

#endif

namespace vermils
{
namespace stm32
{
namespace gpio
{

using hidden::_Port;

namespace ports
{
    #ifdef GPIOA_BASE
    _Port & PortA = *(hidden::_Port*)GPIOA_BASE; // Extern linkage
    #endif
    #ifdef GPIOB_BASE
    _Port & PortB = *(hidden::_Port*)GPIOB_BASE; // Extern linkage
    #endif
    #ifdef GPIOC_BASE
    _Port & PortC = *(hidden::_Port*)GPIOC_BASE; // Extern linkage
    #endif
    #ifdef GPIOD_BASE
    _Port & PortD = *(hidden::_Port*)GPIOD_BASE; // Extern linkage
    #endif
    #ifdef GPIOE_BASE
    _Port & PortE = *(hidden::_Port*)GPIOE_BASE; // Extern linkage
    #endif
    #ifdef GPIOF_BASE
    _Port & PortF = *(hidden::_Port*)GPIOF_BASE; // Extern linkage
    #endif
    #ifdef GPIOG_BASE
    _Port & PortG = *(hidden::_Port*)GPIOG_BASE; // Extern linkage
    #endif
    #ifdef GPIOH_BASE
    _Port & PortH = *(hidden::_Port*)GPIOH_BASE; // Extern linkage
    #endif
    #ifdef GPIOI_BASE
    _Port & PortI = *(hidden::_Port*)GPIOI_BASE; // Extern linkage
    #endif
}

int _get_port_index(const _Port & port)
{
    if constexpr(false)
    {}
    #ifdef GPIOA_BASE
    else if (port == PortA)
        return 0;
    #endif
    #ifdef GPIOB_BASE
    else if (port == PortB)
        return 1;
    #endif
    #ifdef GPIOC_BASE
    else if (port == PortC)
        return 2;
    #endif
    #ifdef GPIOD_BASE
    else if (port == PortD)
        return 3;
    #endif
    #ifdef GPIOE_BASE
    else if (port == PortE)
        return 4;
    #endif
    #ifdef GPIOF_BASE
    else if (port == PortF)
        return 5;
    #endif
    #ifdef GPIOG_BASE
    else if (port == PortG)
        return 6;
    #endif
    #ifdef GPIOH_BASE
    else if (port == PortH)
        return 7;
    #endif
    #ifdef GPIOI_BASE
    else if (port == PortI)
        return 8;
    #endif

    return -1;
}

_Port * _get_port_by_index(const int index)
{
    if constexpr(false)
    {}
    #ifdef GPIOA_BASE
    else if (index == 0)
        return &PortA;
    #endif
    #ifdef GPIOB_BASE
    else if (index == 1)
        return &PortB;
    #endif
    #ifdef GPIOC_BASE
    else if (index == 2)
        return &PortC;
    #endif
    #ifdef GPIOD_BASE
    else if (index == 3)
        return &PortD;
    #endif
    #ifdef GPIOE_BASE
    else if (index == 4)
        return &PortE;
    #endif
    #ifdef GPIOF_BASE
    else if (index == 5)
        return &PortF;
    #endif
    #ifdef GPIOG_BASE
    else if (index == 6)
        return &PortG;
    #endif
    #ifdef GPIOH_BASE
    else if (index == 7)
        return &PortH;
    #endif
    #ifdef GPIOI_BASE
    else if (index == 8)
        return &PortI;
    #endif

    return nullptr;
}

bool is_valid_port(const _Port & port)
{
    return _get_port_index(port) != -1;
}

bool is_valid_port(const int index)
{
    return _get_port_by_index(index) != nullptr;
}

/**
 * @brief Get the port index
 * 
 * @param port 
 * @return int index of the port.
 * @throw std::invalid_argument if the port is not supported by the MCU.
 */
int get_port_index(const _Port & port)
{
    int index = _get_port_index(port);
    if (index != -1)
        return index;

    throw std::invalid_argument("Port not supported by MCU.");
}

/**
 * @brief Get the port by index
 * 
 * @param index 
 * @return _Port* pointer to the port.
 * @throw std::invalid_argument if the index is not supported by the MCU.
 */
_Port & get_port_by_index(const int index)
{
    _Port * port = _get_port_by_index(index);
    if (port != nullptr)
        return *port;

    throw std::invalid_argument("Port not supported by MCU.");
}

/*Pin class methods*/

PinConfig::IO Pin::_IO::getter() const
{
    return static_cast<PinConfig::IO>(
        (owner.port.MODER >> (owner._pin * 2)) & GPIO_MODER_MODER0);
}
void Pin::_IO::setter(const PinConfig::IO value)
{
    uint32_t temp = owner.port.MODER;
    temp &= ~(GPIO_MODER_MODER0 << (owner._pin * 2));
    temp |= (static_cast<uint32_t>(value) << (owner._pin * 2));
    owner.port.MODER = temp;
}

PinConfig::OutMode Pin::_OutMode::getter() const
{
    return static_cast<PinConfig::OutMode>(
        (owner.port.OTYPER >> owner._pin) & GPIO_OTYPER_OT_0);
}
void Pin::_OutMode::setter(const PinConfig::OutMode value)
{
    uint32_t temp = owner.port.OTYPER;
    temp &= ~(GPIO_OTYPER_OT_0 << owner._pin);
    temp |= (static_cast<uint32_t>(value) << owner._pin);
    owner.port.OTYPER = temp;
}

PinConfig::EXTIMode Pin::_EXTIMode::getter() const
{
    return static_cast<PinConfig::EXTIMode>(
        bool(EXTI_IMR & owner._mask) | (bool(EXTI_EMR & owner._mask) << 1));
}
void Pin::_EXTIMode::setter(const PinConfig::EXTIMode value)
{
    uint32_t temp;
    switch (value)
    {
        case PinConfig::NoEXTI:
            temp = EXTI_IMR;
            temp &= ~owner._mask;
            EXTI_IMR = temp;
            temp = EXTI_EMR;
            temp &= ~owner._mask;
            EXTI_EMR = temp;
            break;
        case PinConfig::Interrupt:
            temp = EXTI_IMR;
            temp |= owner._mask;
            EXTI_IMR = temp;
            temp = EXTI_EMR;
            temp &= ~owner._mask;
            EXTI_EMR = temp;
            break;
        case PinConfig::Event:
            temp = EXTI_IMR;
            temp &= ~owner._mask;
            EXTI_IMR = temp;
            temp = EXTI_EMR;
            temp |= owner._mask;
            EXTI_EMR = temp;
            break;
    }
}

PinConfig::Trigger Pin::_Trigger::getter() const
{
    return static_cast<PinConfig::Trigger>(
        bool(EXTI_RTSR & owner._mask) | (bool(EXTI_FTSR & owner._mask) << 1));
}
void Pin::_Trigger::setter(const PinConfig::Trigger value)
{
    uint32_t temp;
    switch (value)
    {
        case PinConfig::NoTrigger:
            temp = EXTI_RTSR;
            temp &= ~owner._mask;
            EXTI_RTSR = temp;
            temp = EXTI_FTSR;
            temp &= ~owner._mask;
            EXTI_FTSR = temp;
            break;
        case PinConfig::Rising:
            temp = EXTI_RTSR;
            temp |= owner._mask;
            EXTI_RTSR = temp;
            temp = EXTI_FTSR;
            temp &= ~owner._mask;
            EXTI_FTSR = temp;
            break;
        case PinConfig::Falling:
            temp = EXTI_RTSR;
            temp &= ~owner._mask;
            EXTI_RTSR = temp;
            temp = EXTI_FTSR;
            temp |= owner._mask;
            EXTI_FTSR = temp;
            break;
        case PinConfig::RisingFalling:
            temp = EXTI_RTSR;
            temp |= owner._mask;
            EXTI_RTSR = temp;
            temp = EXTI_FTSR;
            temp |= owner._mask;
            EXTI_FTSR = temp;
            break;
    }
}

PinConfig::Pull Pin::_Pull::getter() const
{
    return static_cast<PinConfig::Pull>(
        (owner.port.PUPDR >> (owner._pin * 2)) & GPIO_PUPDR_PUPDR0);
}
void Pin::_Pull::setter(const PinConfig::Pull value)
{
    uint32_t temp = owner.port.PUPDR;
    temp &= ~(GPIO_PUPDR_PUPDR0 << (owner._pin * 2));
    temp |= (static_cast<uint32_t>(value) << (owner._pin * 2));
    owner.port.PUPDR = temp;
}

PinConfig::Speed Pin::_Speed::getter() const
{
    return static_cast<PinConfig::Speed>(
        (owner.port.OSPEEDR >> (owner._pin * 2)) & GPIO_OSPEEDER_OSPEEDR0);
}
void Pin::_Speed::setter(const PinConfig::Speed value)
{
    uint32_t temp = owner.port.OSPEEDR;
    temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (owner._pin * 2));
    temp |= (static_cast<uint32_t>(value) << (owner._pin * 2));
    owner.port.OSPEEDR = temp;
}

uint8_t Pin::_Alternate::getter() const
{
    return static_cast<uint8_t>(
        (owner.port.AFR[owner._pin >> 3] >> ((owner._pin & 0x7) << 2)) & 0xFU);
}
void Pin::_Alternate::setter(const uint8_t value)
{
    uint32_t temp = owner.port.AFR[owner._pin >> 3];
    temp &= ~(static_cast<uint32_t>(0xFU) << ((owner._pin & 0x7) << 2));
    temp |= (static_cast<uint32_t>(value) << ((owner._pin & 0x7) << 2));
    owner.port.AFR[owner._pin >> 3] = temp;
}

Pin::Pin(_Port & port, const uint8_t pin): _pin(pin), _mask(1 << pin), port(port)
{
    port.enable_clock();
}

Pin::Pin(hidden::_Port & port, const uint8_t pin, const PinConfig & config):
        Pin(port, pin)
    {
        load(config);
    }

/**
 * @brief Enables the clock for the GPIO port
 * 
 * 
 * @note This function will be called by the Pin when initialised
 */
void _Port::enable_clock() const
{
    #ifdef __HAL_RCC_AFIO_CLK_ENABLE
    __HAL_RCC_AFIO_CLK_ENABLE();
    #endif
    [[maybe_unused]] volatile uint32_t temp;
    uint32_t mask = 1U << get_port_index(*this);
    RCC->AHB1ENR |= mask;
    // Delay after an RCC peripheral clock enabling
    temp = RCC->AHB1ENR & mask;
}

/**
 * @brief Disables the clock for the GPIO port
 * 
 */
void _Port::disable_clock() const
{
    uint32_t mask = 1 << get_port_index(*this);
    RCC->AHB1ENR &= ~mask;
}

inline void _Port::set(const Pin & pin)
{
    BSRR = pin._mask;
}

inline void _Port::reset(const Pin & pin)
{
    BSRR = pin._mask << 16U;
}

/**
 * @brief Sets the configuration of a GPIO pin.
 * 
 * @param config The configuration to set.
 * @return const PinConfig& The configuration that was set.
 */
const PinConfig & Pin::load(const PinConfig & config) const
{
    uint32_t temp;

    // Config output/AF mode
    if (config.io == PinConfig::Output ||
        config.io == PinConfig::AF)
    {
        // Config speed
        temp = port.OSPEEDR;
        temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (_pin * 2U));
        temp |= (static_cast<uint32_t>(config.speed) << (_pin * 2U));
        port.OSPEEDR = temp;

        temp = port.OTYPER;
        temp &= ~(GPIO_OTYPER_OT_0 << _pin);
        temp |= (static_cast<uint32_t>(config.out_mode) << _pin);
        port.OTYPER = temp;
    }

    // Config pull mode
    if (config.io != PinConfig::Analog)
    {
        temp = port.PUPDR;
        temp &= ~(GPIO_PUPDR_PUPDR0 << (_pin * 2U));
        temp |= (static_cast<uint32_t>(config.pull) << (_pin * 2U));
        port.PUPDR = temp;
    }

    // Config Alternate function
    if (config.io == PinConfig::AF)
    {
        temp = port.AFR[_pin >> 3U];
        temp &= ~(static_cast<uint32_t>(0xFU) << ((_pin & 0x7U) << 2U));
        temp |= (static_cast<uint32_t>(config.alternate) << ((_pin & 0x7U) << 2U));
        port.AFR[_pin >> 3U] = temp;
    }

    // Configure IO Direction mode (Input, Output, Alternate or Analog)
    temp = port.MODER;
    temp &= ~(GPIO_MODER_MODER0 << (_pin * 2U));
    temp |= (static_cast<uint32_t>(config.io) << (_pin * 2U));
    port.MODER = temp;

    // Configure the External Interrupt or event for GPIO
    if (config.exti_mode != PinConfig::NoEXTI)
    {
        // Enable SYSCFG Clock
        __HAL_RCC_SYSCFG_CLK_ENABLE();

        temp = SYSCFG->EXTICR[_pin >> 2U];
        temp &= ~(0x0FU << (4U * (_pin & 0x03U)));
        temp |= (uint32_t(get_port_index(port)) << (4U * (_pin & 0x03U)));
        SYSCFG->EXTICR[_pin >> 2U] = temp;

        // Clear EXTI line configuration
        temp = EXTI_IMR;
        temp &= ~_mask;
        if (config.exti_mode == PinConfig::Interrupt)
        {
            temp |= _mask;
        }
        EXTI_IMR = temp;

        temp = EXTI_EMR;
        temp &= ~_mask;
        if (config.exti_mode == PinConfig::Event)
        {
            temp |= _mask;
        }
        EXTI_EMR = temp;

        // Clear Rising Falling edge configuration
        temp = EXTI_RTSR;
        temp &= ~_mask;
        if (config.trigger == PinConfig::Rising ||
            config.trigger == PinConfig::RisingFalling)
        {
            temp |= _mask;
        }
        EXTI_RTSR = temp;

        temp = EXTI_FTSR;
        temp &= ~_mask;
        if (config.trigger == PinConfig::Falling ||
            config.trigger == PinConfig::RisingFalling)
        {
            temp |= _mask;
        }
        EXTI_FTSR = temp;
    }

    return config;
}

void Pin::unload() const
{
    uint32_t temp;

    // EXTI configuration
    temp = SYSCFG->EXTICR[_pin >> 2U];
    temp &= ~(0x0FU << (4U * (_pin & 0x03U)));
    if (temp == (uint32_t(get_port_index(port)) << (4U * (_pin & 0x03U))))
    {
        // Clear EXTI line configuration
        EXTI_IMR &= ~uint32_t(_pin);
        EXTI_EMR &= ~uint32_t(_pin);

        // Clear Rising Falling edge configuration
        EXTI_RTSR &= ~uint32_t(_pin);
        EXTI_FTSR &= ~uint32_t(_pin);

        // Clear EXTI for current io
        temp = 0x0FU << (4U * (_pin & 0x03U));
        SYSCFG->EXTICR[_pin >> 2U] &= ~temp;
    }

    // GPIO Mode configuration
    port.MODER &= ~(GPIO_MODER_MODER0 << (_pin * 2U));
    port.AFR[_pin >> 3U] &= ~(uint32_t(0xFU) << ((_pin & 0x7U) << 2U));
    port.PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (_pin * 2U));
    port.OTYPER &= ~(GPIO_OTYPER_OT_0 << _pin);
    port.OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (_pin * 2U));
}

} // namespace gpio
} // namespace stm32
} // namespace vermils

#endif // defined(__VERMIL_STM32F4) || defined(__VERMIL_STM32H7)
