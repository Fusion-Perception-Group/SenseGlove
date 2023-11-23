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

#if defined(__VERMIL_STM32F1) && !__VERMIL_STM32_USE_GENERIC && !VERMIL_STM32_USE_CMSIS

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

Pin::Pin(_Port & port, const uint8_t pin): _pin(pin), _mask(1 << pin), port(port),
    io({
        [this, &port]() -> PinConfig::IO
        {
            return static_cast<PinConfig::IO>(
                (port.MODER >> (_pin * 2)) & GPIO_MODER_MODER0
            );
        },
        [this, &port](const PinConfig::IO value)
        {
            uint32_t temp = port.MODER;
            temp &= ~(GPIO_MODER_MODER0 << (_pin * 2));
            temp |= (static_cast<uint32_t>(value) << (_pin * 2));
            port.MODER = temp;
        }
    }),
    out_mode(
        {
        [this, &port]() -> PinConfig::OutMode
        {
            return static_cast<PinConfig::OutMode>(
                (port.OTYPER >> _pin) & GPIO_OTYPER_OT_0
            );
        },
        [this, &port](const PinConfig::OutMode value)
        {
            uint32_t temp = port.OTYPER;
            temp &= ~(GPIO_OTYPER_OT_0 << _pin);
            temp |= (static_cast<uint32_t>(value) << _pin);
            port.OTYPER = temp;
        }
    }),
    exti_mode({
        [this]() -> PinConfig::EXTIMode
        {
            return static_cast<PinConfig::EXTIMode>(
                bool(EXTI_IMR & _mask) | (bool(EXTI_EMR & _mask) << 1)
            );
        },
        [this](const PinConfig::EXTIMode value)
        {
            uint32_t temp;
            switch (value)
            {
                case PinConfig::NoEXTI:
                    temp = EXTI_IMR;
                    temp &= ~_mask;
                    EXTI_IMR = temp;
                    temp = EXTI_EMR;
                    temp &= ~_mask;
                    EXTI_EMR = temp;
                    break;
                case PinConfig::Interrupt:
                    temp = EXTI_IMR;
                    temp |= _mask;
                    EXTI_IMR = temp;
                    temp = EXTI_EMR;
                    temp &= ~_mask;
                    EXTI_EMR = temp;
                    break;
                case PinConfig::Event:
                    temp = EXTI_IMR;
                    temp &= ~_mask;
                    EXTI_IMR = temp;
                    temp = EXTI_EMR;
                    temp |= _mask;
                    EXTI_EMR = temp;
                    break;
            }
        }
    }),
    trigger({
        [this]() -> PinConfig::Trigger
        {
            return static_cast<PinConfig::Trigger>(
                bool(EXTI_RTSR & _mask) | (bool(EXTI_FTSR & _mask) << 1)
            );
        },
        [this](const PinConfig::Trigger value)
        {
            uint32_t temp;
            switch (value)
            {
                case PinConfig::NoTrigger:
                    temp = EXTI_RTSR;
                    temp &= ~_mask;
                    EXTI_RTSR = temp;
                    temp = EXTI_FTSR;
                    temp &= ~_mask;
                    EXTI_FTSR = temp;
                    break;
                case PinConfig::Rising:
                    temp = EXTI_RTSR;
                    temp |= _mask;
                    EXTI_RTSR = temp;
                    temp = EXTI_FTSR;
                    temp &= ~_mask;
                    EXTI_FTSR = temp;
                    break;
                case PinConfig::Falling:
                    temp = EXTI_RTSR;
                    temp &= ~_mask;
                    EXTI_RTSR = temp;
                    temp = EXTI_FTSR;
                    temp |= _mask;
                    EXTI_FTSR = temp;
                    break;
                case PinConfig::RisingFalling:
                    temp = EXTI_RTSR;
                    temp |= _mask;
                    EXTI_RTSR = temp;
                    temp = EXTI_FTSR;
                    temp |= _mask;
                    EXTI_FTSR = temp;
                    break;
            }
        }
    }),
    pull({
        [this, &port]() -> PinConfig::Pull
        {
            return static_cast<PinConfig::Pull>(
                (port.PUPDR >> (_pin * 2)) & GPIO_PUPDR_PUPDR0
            );
        },
        [this, &port](const PinConfig::Pull value)
        {
            uint32_t temp = port.PUPDR;
            temp &= ~(GPIO_PUPDR_PUPDR0 << (_pin * 2));
            temp |= (static_cast<uint32_t>(value) << (_pin * 2));
            port.PUPDR = temp;
        }
    }),
    speed({
        [this, &port]() -> PinConfig::Speed
        {
            return static_cast<PinConfig::Speed>(
                (port.OSPEEDR >> (_pin * 2)) & GPIO_OSPEEDER_OSPEEDR0
            );
        },
        [this, &port](const PinConfig::Speed value)
        {
            uint32_t temp = port.OSPEEDR;
            temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (_pin * 2));
            temp |= (static_cast<uint32_t>(value) << (_pin * 2));
            port.OSPEEDR = temp;
        }
    }),
    alternate({
        [this, &port]() -> uint8_t
        {
            return static_cast<uint8_t>(
                (port.AFR[_pin >> 3] >> ((_pin & 0x7) << 2)) & 0xFU
            );
        },
        [this, &port](const uint8_t value)
        {
            uint32_t temp = port.AFR[_pin >> 3];
            temp &= ~(static_cast<uint32_t>(0xFU) << ((_pin & 0x7) << 2));
            temp |= (static_cast<uint32_t>(value) << ((_pin & 0x7) << 2));
            port.AFR[_pin >> 3] = temp;
        }
    })
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
    uint32_t mask = 1 << get_port_index(*this);
    RCC->AHB1ENR |= mask;
    // Delay after an RCC peripheral clock enabling
    temp = RCC->AHB1ENR &= mask;
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
        temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (_pin * 2));
        temp |= (static_cast<uint32_t>(config.speed) << (_pin * 2));
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
        temp &= ~(GPIO_PUPDR_PUPDR0 << (_pin * 2));
        temp |= (static_cast<uint32_t>(config.pull) << (_pin * 2));
        port.PUPDR = temp;
    }

    // Config Alternate function
    if (config.io == PinConfig::AF)
    {
        temp = port.AFR[_pin >> 3];
        temp &= ~(static_cast<uint32_t>(0xFU) << ((_pin & 0x7) << 2));
        temp |= (static_cast<uint32_t>(config.alternate) << ((_pin & 0x7) << 2));
        port.AFR[_pin >> 3] = temp;
    }

    // Configure IO Direction mode (Input, Output, Alternate or Analog)
    temp = port.MODER;
    temp &= ~(GPIO_MODER_MODER0 << (_pin * 2));
    temp |= (static_cast<uint32_t>(config.io) << (_pin * 2));
    port.MODER = temp;

    // Configure the External Interrupt or event for GPIO
    if (config.exti_mode != PinConfig::NoEXTI)
    {
        // Enable SYSCFG Clock
        __HAL_RCC_SYSCFG_CLK_ENABLE();

        temp = SYSCFG->EXTICR[_pin >> 2];
        temp &= ~(0x0FU << (4 * (_pin & 0x03)));
        temp |= (uint32_t(get_port_index(port)) << (4 * (_pin & 0x03)));
        SYSCFG->EXTICR[_pin >> 2] = temp;

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

const PinConfig Pin::load() const
{
    return load(PinConfig());
}

void Pin::unload() const
{
    uint32_t temp;

    // EXTI configuration
    temp = SYSCFG->EXTICR[_pin >> 2];
    temp &= ~(0x0FU << (4 * (_pin & 0x03)));
    if (temp == (uint32_t(get_port_index(port)) << (4 * (_pin & 0x03))))
    {
        // Clear EXTI line configuration
        EXTI_IMR &= ~uint32_t(_pin);
        EXTI_EMR &= ~uint32_t(_pin);

        // Clear Rising Falling edge configuration
        EXTI_RTSR &= ~uint32_t(_pin);
        EXTI_FTSR &= ~uint32_t(_pin);

        // Clear EXTI for current io
        temp = 0x0FU << (4 * (_pin & 0x03));
        SYSCFG->EXTICR[_pin >> 2] &= ~temp;
    }

    // GPIO Mode configuration
    port.MODER &= ~(GPIO_MODER_MODER0 << (_pin * 2));
    port.AFR[_pin >> 3] &= ~(uint32_t(0xFU) << ((_pin & 0x7) << 2));
    port.PUPDR &= ~(GPIO_PUPDR_PUPDR0 << (_pin * 2));
    port.OTYPER &= ~(GPIO_OTYPER_OT_0 << _pin);
    port.OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (_pin * 2));
}

} // namespace gpio
} // namespace stm32
} // namespace vermils

#endif // !VERMIL_STM32_USE_CMSIS && __VERMIL_STM32_USE_GENERIC