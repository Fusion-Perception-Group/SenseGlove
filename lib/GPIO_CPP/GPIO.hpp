#ifndef _p_gpio_hpp_
#define _p_gpio_hpp_

#include <string>
#include "MCU_Conf.hpp"
#include "Property.hpp"

namespace vermils
{
namespace stm32
{

static const uint8_t GPIO_PINS_N = 16;

struct GPIOPinConfig
{
    enum IO
    {
        Input=MODE_INPUT,
        Output=MODE_OUTPUT,
        AF=MODE_AF,
        Analog=MODE_ANALOG,
    };
    enum OutMode
    {
        PushPull=OUTPUT_PP,
        OpenDrain=OUTPUT_OD,
    };
    enum EXTIMode
    {
        NoEXTI=EXTI_MODE_NONE,
        Interrupt=EXTI_MODE_INTERRUPT ,
        Event=EXTI_MODE_EVENT,
    };
    enum Trigger
    {
        NoTrigger=EXTI_TRIGGER_NONE,
        Rising=EXTI_TRIGGER_RISING,
        Falling=EXTI_TRIGGER_FALLING,
        RisingFalling=EXTI_TRIGGER_RISING_FALLING,
    };
    enum Pull
    {
        NoPull=GPIO_NOPULL,
        PullUp=GPIO_PULLUP,
        PullDown=GPIO_PULLDOWN,
    };
    enum Speed
    {
        Low=GPIO_SPEED_FREQ_LOW,
        Medium=GPIO_SPEED_FREQ_MEDIUM,
        High=GPIO_SPEED_FREQ_HIGH,
        #ifdef GPIO_SPEED_FREQ_VERY_HIGH
        VeryHigh=GPIO_SPEED_FREQ_VERY_HIGH,
        #endif
    };

    GPIOPinConfig() = default;
    //Opted for input
    GPIOPinConfig(
        const IO io, const Pull pull,
        const EXTIMode exti_mode = NoEXTI, const Trigger trigger = NoTrigger):
        io(io), pull(pull), exti_mode(exti_mode), trigger(trigger) {}
    //Opted for output
    GPIOPinConfig(
        const IO io, const Speed speed, const OutMode out_mode, const uint8_t alternate = 0):
        io(io), speed(speed), out_mode(out_mode), alternate(alternate) {}
    // Universal
    GPIOPinConfig(
        const IO io, const Pull pull, const Speed speed, const OutMode out_mode,
        const uint8_t alternate, const EXTIMode exti_mode, const Trigger trigger):
        io(io), pull(pull), speed(speed), out_mode(out_mode),
        alternate(alternate), exti_mode(exti_mode), trigger(trigger) {}

    IO io = Input;
    Pull pull = NoPull;
    Speed speed = Low;
    OutMode out_mode = PushPull;
    uint8_t alternate = 0;
    EXTIMode exti_mode = NoEXTI;
    Trigger trigger = NoTrigger;
};

class GPIOPin
{    
public:
    GPIO_TypeDef* const port;
    const uint8_t pin;

    GPIOPin(GPIO_TypeDef* const port, const uint8_t pin): port(port), pin(pin) {}
    GPIOPin(GPIO_TypeDef* const port, const uint8_t pin, const GPIOPinConfig & config): port(port), pin(pin)
    {
        set(config);
    }

    mutable tricks::Property<GPIOPinConfig::IO> io = {
        [this]() -> GPIOPinConfig::IO
        {
            return static_cast<GPIOPinConfig::IO>(
                (port->MODER >> (pin * 2)) & GPIO_MODER_MODER0
            );
        },
        [this](const GPIOPinConfig::IO value)
        {
            uint32_t temp = port->MODER;
            temp &= ~(GPIO_MODER_MODER0 << (pin * 2));
            temp |= (static_cast<uint32_t>(value) << (pin * 2));
            port->MODER = temp;
        }
    };

    mutable tricks::Property<GPIOPinConfig::OutMode> out_mode = {
        [this]() -> GPIOPinConfig::OutMode
        {
            return static_cast<GPIOPinConfig::OutMode>(
                (port->OTYPER >> pin) & GPIO_OTYPER_OT_0
            );
        },
        [this](const GPIOPinConfig::OutMode value)
        {
            uint32_t temp = port->OTYPER;
            temp &= ~(GPIO_OTYPER_OT_0 << pin);
            temp |= (static_cast<uint32_t>(value) << pin);
            port->OTYPER = temp;
        }
    };

    mutable tricks::Property<GPIOPinConfig::EXTIMode> exti_mode = {
        [this]() -> GPIOPinConfig::EXTIMode
        {
            const uint32_t mask = 1 << pin;
            return static_cast<GPIOPinConfig::EXTIMode>(
                bool(EXTI->IMR & mask) | (bool(EXTI->EMR & mask) << 1)
            );
        },
        [this](const GPIOPinConfig::EXTIMode value)
        {
            uint32_t temp;
            const uint32_t mask = 1 << pin;
            switch (value)
            {
                case GPIOPinConfig::NoEXTI:
                    temp = EXTI->IMR;
                    temp &= ~mask;
                    EXTI->IMR = temp;
                    temp = EXTI->EMR;
                    temp &= ~mask;
                    EXTI->EMR = temp;
                    break;
                case GPIOPinConfig::Interrupt:
                    temp = EXTI->IMR;
                    temp |= mask;
                    EXTI->IMR = temp;
                    temp = EXTI->EMR;
                    temp &= ~mask;
                    EXTI->EMR = temp;
                    break;
                case GPIOPinConfig::Event:
                    temp = EXTI->IMR;
                    temp &= ~mask;
                    EXTI->IMR = temp;
                    temp = EXTI->EMR;
                    temp |= mask;
                    EXTI->EMR = temp;
                    break;
            }
        }
    };

    mutable tricks::Property<GPIOPinConfig::Trigger> trigger = {
        [this]() -> GPIOPinConfig::Trigger
        {
            const uint32_t mask = 1 << pin;
            return static_cast<GPIOPinConfig::Trigger>(
                bool(EXTI->RTSR & mask) | (bool(EXTI->FTSR & mask) << 1)
            );
        },
        [this](const GPIOPinConfig::Trigger value)
        {
            uint32_t temp;
            const uint32_t mask = 1 << pin;
            switch (value)
            {
                case GPIOPinConfig::NoTrigger:
                    temp = EXTI->RTSR;
                    temp &= ~mask;
                    EXTI->RTSR = temp;
                    temp = EXTI->FTSR;
                    temp &= ~mask;
                    EXTI->FTSR = temp;
                    break;
                case GPIOPinConfig::Rising:
                    temp = EXTI->RTSR;
                    temp |= mask;
                    EXTI->RTSR = temp;
                    temp = EXTI->FTSR;
                    temp &= ~mask;
                    EXTI->FTSR = temp;
                    break;
                case GPIOPinConfig::Falling:
                    temp = EXTI->RTSR;
                    temp &= ~mask;
                    EXTI->RTSR = temp;
                    temp = EXTI->FTSR;
                    temp |= mask;
                    EXTI->FTSR = temp;
                    break;
                case GPIOPinConfig::RisingFalling:
                    temp = EXTI->RTSR;
                    temp |= mask;
                    EXTI->RTSR = temp;
                    temp = EXTI->FTSR;
                    temp |= mask;
                    EXTI->FTSR = temp;
                    break;
            }
        }
    };

    mutable tricks::Property<GPIOPinConfig::Pull> pull = {
        [this]() -> GPIOPinConfig::Pull
        {
            return static_cast<GPIOPinConfig::Pull>(
                (port->PUPDR >> (pin * 2)) & GPIO_PUPDR_PUPDR0
            );
        },
        [this](const GPIOPinConfig::Pull value)
        {
            uint32_t temp = port->PUPDR;
            temp &= ~(GPIO_PUPDR_PUPDR0 << (pin * 2));
            temp |= (static_cast<uint32_t>(value) << (pin * 2));
            port->PUPDR = temp;
        }
    };

    mutable tricks::Property<GPIOPinConfig::Speed> speed = {
        [this]() -> GPIOPinConfig::Speed
        {
            return static_cast<GPIOPinConfig::Speed>(
                (port->OSPEEDR >> (pin * 2)) & GPIO_OSPEEDER_OSPEEDR0
            );
        },
        [this](const GPIOPinConfig::Speed value)
        {
            uint32_t temp = port->OSPEEDR;
            temp &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pin * 2));
            temp |= (static_cast<uint32_t>(value) << (pin * 2));
            port->OSPEEDR = temp;
        }
    };

    mutable tricks::Property<uint8_t> alternate = {
        [this]() -> uint8_t
        {
            return static_cast<uint8_t>(
                (port->AFR[pin >> 3] >> ((pin & 0x7) << 2)) & 0xFU
            );
        },
        [this](const uint8_t value)
        {
            uint32_t temp = port->AFR[pin >> 3];
            temp &= ~(static_cast<uint32_t>(0xFU) << ((pin & 0x7) << 2));
            temp |= (static_cast<uint32_t>(value) << ((pin & 0x7) << 2));
            port->AFR[pin >> 3] = temp;
        }
    };

    void enable_clock() const;
    void disable_clock() const;
    const GPIOPinConfig set() const;
    const GPIOPinConfig & set(const GPIOPinConfig & config) const;
    void reset() const;

    bool read() const
    {
        return bool(port->IDR & (1 << pin));
    }

    void write(const bool value) const
    {
        uint32_t temp = 1 << pin;
        if (value)
            temp <<= GPIO_PINS_N;
        port->BSRR = temp;
    }

    bool toggle() const
    {
        uint32_t temp = 1 << pin;
        bool value = bool(port->ODR & temp);
        if (value)
            temp <<= GPIO_PINS_N;
        port->BSRR = temp;
        return value;
    }

    explicit operator bool () const
    {
        return read();
    }

    bool operator = (const bool value) const
    {
        write(value);
        return value;
    }
};

}
}

#endif