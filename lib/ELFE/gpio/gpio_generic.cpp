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
#include "./_cpp_config.hpp"
#include "gpio/gpio.hpp"

#define PIN_FWD(TYPE, CLSN, NAME)                  \
    TYPE Pin::CLSN::getter() const                 \
    {                                              \
        return owner._config.NAME;                 \
    }                                              \
    void Pin::CLSN::setter(const TYPE value) const \
    {                                              \
        owner._config.NAME = value;                \
        owner.load(owner._config);                 \
    }

#if !ELFE_STM32_USE_CMSIS && _ELFE_STM32_USE_GENERIC
namespace elfe {
namespace stm32 {
    namespace gpio {

        using hidden::_Port;
        namespace ports {
#ifdef GPIOA
            _Port PortA = _Port(GPIOA->BSRR, GPIOA->IDR, GPIOA->ODR); // Extern linkage
#endif
#ifdef GPIOB
            _Port PortB = _Port(GPIOB->BSRR, GPIOB->IDR, GPIOB->ODR); // Extern linkage
#endif
#ifdef GPIOC
            _Port PortC = _Port(GPIOC->BSRR, GPIOC->IDR, GPIOC->ODR); // Extern linkage
#endif
#ifdef GPIOD
            _Port PortD = _Port(GPIOD->BSRR, GPIOD->IDR, GPIOD->ODR); // Extern linkage
#endif
#ifdef GPIOE
            _Port PortE = _Port(GPIOE->BSRR, GPIOE->IDR, GPIOE->ODR); // Extern linkage
#endif
#ifdef GPIOF
            _Port PortF = _Port(GPIOF->BSRR, GPIOF->IDR, GPIOF->ODR); // Extern linkage
#endif
#ifdef GPIOG
            _Port PortG = _Port(GPIOG->BSRR, GPIOG->IDR, GPIOG->ODR); // Extern linkage
#endif
#ifdef GPIOH
            _Port PortH = _Port(GPIOH->BSRR, GPIOH->IDR, GPIOH->ODR); // Extern linkage
#endif
#ifdef GPIOI
            _Port PortI = _Port(GPIOI->BSRR, GPIOI->IDR, GPIOI->ODR); // Extern linkage
#endif
        }

        int _get_port_index(const _Port& port)
        {
            if constexpr (false) {
            }
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

        _Port* _get_port_by_index(const int index)
        {
            if constexpr (false) {
            }
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

        bool is_valid_port(const _Port& port)
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
        int get_port_index(const _Port& port)
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
        _Port& get_port_by_index(const int index)
        {
            _Port* port = _get_port_by_index(index);
            if (port != nullptr)
                return *port;

            throw std::invalid_argument("Port not supported by MCU.");
        }

        /*Pin class methods*/

        PIN_FWD(PinConfig::IO, _IO, io)
        PIN_FWD(PinConfig::OutMode, _OutMode, out_mode)
        PIN_FWD(PinConfig::EXTIMode, _EXTIMode, exti_mode)
        PIN_FWD(PinConfig::Trigger, _Trigger, trigger)
        PIN_FWD(PinConfig::Pull, _Pull, pull)
        PIN_FWD(PinConfig::Speed, _Speed, speed)
        PIN_FWD(uint8_t, _Alternate, alternate)

        Pin::Pin(hidden::_Port& port, const uint8_t pin, const PinConfig& config)
            : _pin(pin)
            , _mask(1 << pin)
            , _config(config)
            , port(port)

        {
            port.enable_clock();
            load(config);
        }

        Pin::Pin(_Port& port, const uint8_t pin)
            : Pin(port, pin, PinConfig())
        {
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
            if constexpr (false) {
            }
#if defined(GPIOA_BASE)
            else if (this == &PortA)
                __HAL_RCC_GPIOA_CLK_ENABLE();
#endif
#if defined(GPIOB_BASE)
            else if (this == &PortB)
                __HAL_RCC_GPIOB_CLK_ENABLE();
#endif
#if defined(GPIOC_BASE)
            else if (this == &PortC)
                __HAL_RCC_GPIOC_CLK_ENABLE();
#endif
#if defined(GPIOD_BASE)
            else if (this == &PortD)
                __HAL_RCC_GPIOD_CLK_ENABLE();
#endif
#if defined(GPIOE_BASE)
            else if (this == &PortE)
                __HAL_RCC_GPIOE_CLK_ENABLE();
#endif
#if defined(GPIOF_BASE)
            else if (this == &PortF)
                __HAL_RCC_GPIOF_CLK_ENABLE();
#endif
#if defined(GPIOG_BASE)
            else if (this == &PortG)
                __HAL_RCC_GPIOG_CLK_ENABLE();
#endif
#if defined(GPIOH_BASE)
            else if (this == &PortH)
                __HAL_RCC_GPIOH_CLK_ENABLE();
#endif
#if defined(GPIOI_BASE)
            else if (this == &PortI)
                __HAL_RCC_GPIOI_CLK_ENABLE();
#endif
        }

        /**
         * @brief Disables the clock for the GPIO port
         *
         */
        void _Port::disable_clock() const
        {
            if constexpr (false) {
            }
#if defined(GPIOA_BASE)
            else if (this == &PortA)
                __HAL_RCC_GPIOA_CLK_DISABLE();
#endif
#if defined(GPIOB_BASE)
            else if (this == &PortB)
                __HAL_RCC_GPIOB_CLK_DISABLE();
#endif
#if defined(GPIOC_BASE)
            else if (this == &PortC)
                __HAL_RCC_GPIOC_CLK_DISABLE();
#endif
#if defined(GPIOD_BASE)
            else if (this == &PortD)
                __HAL_RCC_GPIOD_CLK_DISABLE();
#endif
#if defined(GPIOE_BASE)
            else if (this == &PortE)
                __HAL_RCC_GPIOE_CLK_DISABLE();
#endif
#if defined(GPIOF_BASE)
            else if (this == &PortF)
                __HAL_RCC_GPIOF_CLK_DISABLE();
#endif
#if defined(GPIOG_BASE)
            else if (this == &PortG)
                __HAL_RCC_GPIOG_CLK_DISABLE();
#endif
#if defined(GPIOH_BASE)
            else if (this == &PortH)
                __HAL_RCC_GPIOH_CLK_DISABLE();
#endif
#if defined(GPIOI_BASE)
            else if (this == &PortI)
                __HAL_RCC_GPIOI_CLK_DISABLE();
#endif
        }

        inline void _Port::set(const Pin& pin)
        {
            BSRR = pin._mask;
        }

        inline void _Port::reset(const Pin& pin)
        {
            BSRR = pin._mask << 16U;
        }

        /**
         * @brief Sets the configuration of a GPIO pin.
         *
         * @param config The configuration to set.
         * @return const PinConfig& The configuration that was set.
         */
        const PinConfig& Pin::load(const PinConfig& config) const
        {
            uint32_t mode = GPIO_MODE_INPUT, pull = GPIO_NOPULL, speed = GPIO_SPEED_FREQ_LOW;

            if (config.io == PinConfig::Input) {
                if (config.exti_mode == PinConfig::NoEXTI)
                    mode = GPIO_MODE_INPUT;
                else if (config.exti_mode == PinConfig::Interrupt) {
                    if (config.trigger == PinConfig::Rising)
                        mode = GPIO_MODE_IT_RISING;
                    else if (config.trigger == PinConfig::Falling)
                        mode = GPIO_MODE_IT_FALLING;
                    else if (config.trigger == PinConfig::RisingFalling)
                        mode = GPIO_MODE_IT_RISING_FALLING;
                } else if (config.exti_mode == PinConfig::Event) {
                    if (config.trigger == PinConfig::Rising)
                        mode = GPIO_MODE_EVT_RISING;
                    else if (config.trigger == PinConfig::Falling)
                        mode = GPIO_MODE_EVT_FALLING;
                    else if (config.trigger == PinConfig::RisingFalling)
                        mode = GPIO_MODE_EVT_RISING_FALLING;
                }
            } else if (config.io == PinConfig::Output) {
                if (config.out_mode == PinConfig::PushPull)
                    mode = GPIO_MODE_OUTPUT_PP;
                else
                    mode = GPIO_MODE_OUTPUT_OD;
            } else if (config.io == PinConfig::AF) {
                if (config.out_mode == PinConfig::PushPull)
                    mode = GPIO_MODE_AF_PP;
                else
                    mode = GPIO_MODE_AF_OD;
            } else if (config.io == PinConfig::Analog) {
                mode = GPIO_MODE_ANALOG;
            }

            switch (config.pull) {
            case PinConfig::NoPull:
                pull = GPIO_NOPULL;
                break;
            case PinConfig::PullUp:
                pull = GPIO_PULLUP;
                break;
            case PinConfig::PullDown:
                pull = GPIO_PULLDOWN;
                break;
            }

            switch (config.speed) {
            case PinConfig::Low:
                speed = GPIO_SPEED_FREQ_LOW;
                break;
            case PinConfig::Medium:
                speed = GPIO_SPEED_FREQ_MEDIUM;
                break;
            case PinConfig::High:
                speed = GPIO_SPEED_FREQ_HIGH;
                break;
            case PinConfig::VeryHigh:
#ifdef GPIO_SPEED_FREQ_VERY_HIGH
                speed = GPIO_SPEED_FREQ_VERY_HIGH;
#else
                speed = GPIO_SPEED_FREQ_HIGH;
#endif
                break;
            }

            GPIO_InitTypeDef init = {
                .Pin = _mask,
                .Mode = mode,
                .Pull = pull,
                .Speed = speed,
                .Alternate = config.alternate
            };

            HAL_GPIO_Init((GPIO_TypeDef*)&port, &init);

            return config;
        }

        const PinConfig Pin::load() const
        {
            return load(PinConfig());
        }

        void Pin::unload() const
        {
            HAL_GPIO_DeInit((GPIO_TypeDef*)&port, _mask);
        }

    } // namespace gpio
} // namespace stm32
} // namespace elfe

#endif // !ELFE_STM32_USE_CMSIS && _ELFE_STM32_USE_GENERIC