#pragma once

#include <string>
#include <stdexcept>
#include <functional>
#include "userconfig.hpp"
#include "property.hpp"
#include "nvic.hpp"

namespace vermils
{
namespace stm32
{
namespace gpio
{
using CallbackType = std::function<void()>;

 class GPIOException : public std::runtime_error
 {
public:
    GPIOException(const std::string & msg="GPIO Exception."):
        std::runtime_error(msg) {}
 };

// Define some constants
inline constexpr uint8_t GPIO_PINS_N = 16;

class Pin;
namespace hidden
{
    #if _VERMIL_STM32_USE_GENERIC
    extern volatile uint32_t & BSRR;
    #endif
    /**
     * @brief GPIO Port Class
     * @warning DO NOT MANNUALLY INSTANTIATE THIS CLASS
     */
    class _Port final
    {
    public:
        #if _VERMIL_STM32_USE_GENERIC
            volatile uint32_t & BSRR;
            volatile uint32_t & IDR;
            volatile uint32_t & ODR;
            _Port(volatile uint32_t & BSRR, volatile uint32_t & IDR, volatile uint32_t & ODR):
            BSRR(BSRR), IDR(BSRR), ODR(BSRR) {}
        #else
            #if defined(_VERMIL_STM32F1)
            volatile uint32_t CRL;      /*!< GPIO port configuration register low,  Address offset: 0x00      */
            volatile uint32_t CRH;      /*!< GPIO port configuration register high, Address offset: 0x04      */
            volatile uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x08      */
            volatile uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x0C      */
            volatile uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x10      */
            volatile uint32_t BRR;      /*!< GPIO port bit reset register,          Address offset: 0x14      */
            volatile uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x18      */
            #elif defined(_VERMIL_STM32F4) || defined(_VERMIL_STM32H7)
            volatile uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
            volatile uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
            volatile uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
            volatile uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
            volatile uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
            volatile uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
            volatile uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
            volatile uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
            volatile uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
            #endif
            _Port() = delete;
        #endif
        _Port & operator = (const _Port & port) = delete;

        void enable_clock() const;

        void disable_clock() const;

        inline void set(const uint32_t mask)
        {
            BSRR = mask;
        }

        inline void set(const Pin & pin);

        inline void reset(const uint32_t mask)
        {
            BSRR = mask << GPIO_PINS_N;
        }

        inline void reset(const Pin & pin);

        bool operator == (const _Port & port) const
        {
            return &port == this;
        }

        friend class Pin;
    };

    extern CallbackType interrupt_callbacks[GPIO_PINS_N];
}
using Port = hidden::_Port;

bool is_valid_port(const hidden::_Port & port);
bool is_valid_port(const int port_index);
int get_port_index(const hidden::_Port & port);
hidden::_Port & get_port_by_index(const int index);

namespace ports
{
    #if _VERMIL_STM32_USE_GENERIC
        #define __Port_Type hidden::_Port
    #else
        #define __Port_Type hidden::_Port &
    #endif
    extern __Port_Type PortA;
    extern __Port_Type PortB;
    extern __Port_Type PortC;
    extern __Port_Type PortD;
    extern __Port_Type PortE;
    extern __Port_Type PortF;
    extern __Port_Type PortG;
    extern __Port_Type PortH;
    extern __Port_Type PortI;
    #undef __Port_Type
}
using namespace ports;

/**
 * @brief Pin Configuration
 * 
 * @param io The pin mode.
 * @param pull The pull mode.
 * @param speed The speed of the pin.
 * @param out_mode The output mode.
 * @param alternate The alternate function.
 * @param exti_mode The EXTI mode.
 * @param trigger The trigger mode.
 * 
 * @note opt for input: io, pull, exti_mode, trigger
 * @note opt for output: io, speed, out_mode, alternate
 */
struct PinConfig
{
    enum IO
    {
        Input=0x00,
        Output=0x01,
        // Alternate function
        AF=0x02,
        Analog=0x03,
    };
    enum OutMode
    {
        PushPull=0x00,
        OpenDrain=0x01,
    };
    enum EXTIMode
    {
        NoEXTI=0x00,
        Interrupt=0x01,
        Event=0x02,
    };
    enum Trigger
    {
        NoTrigger=0x00,
        Rising=0x01,
        Falling=0x02,
        RisingFalling=Rising | Falling,
    };
    enum Pull
    {
        NoPull=0x00,
        PullUp=0x01,
        PullDown=0x02,
    };
    enum Speed
    {
        // Typically 2MHz
        Low=0x00,
        // Typically 12.5MHz - 50MHz
        Medium=0x01,
        // Typically 25MHz - 100MHz
        High=0x02,
        // Typically 50MHz - 200MHz
        VeryHigh=0x03,
    };

    PinConfig() = default;
    //Opted for input
    PinConfig(
        const IO io, const Pull pull,
        const EXTIMode exti_mode = NoEXTI, const Trigger trigger = NoTrigger):
        io(io), pull(pull), exti_mode(exti_mode), trigger(trigger) {}
    //Opted for output
    PinConfig(
        const IO io, const Speed speed, const OutMode out_mode, const uint8_t alternate = 0):
        io(io), speed(speed), out_mode(out_mode), alternate(alternate) {}
    // Universal
    PinConfig(
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

/**
 * @brief GPIO Pin
 * 
 * @param port The port of the pin.
 * @param pin The pin number. Starts from 0.
 */
class Pin
{
    uint8_t _pin;
    uint32_t _mask;
    #if _VERMIL_STM32_USE_GENERIC
    mutable PinConfig _config;
    #endif
    template <typename T>
    struct _Property : tricks::StaticProperty<T, Pin &>
    {
        using tricks::StaticProperty<T, Pin &>::StaticProperty;
        using tricks::StaticProperty<T, Pin &>::operator =;
    };
    struct _Pin : _Property<uint8_t>
    {
        using _Property<uint8_t>::_Property;
        using _Property<uint8_t>::operator =;
        uint8_t getter() const override { return owner._pin; }
        void setter(const uint8_t value) const override
        {
            owner._pin = value;
            owner._mask = 1U << value;
        }
    };
    struct _Mask : _Property<uint32_t>
    {
        using _Property<uint32_t>::_Property;
        using _Property<uint32_t>::operator =;
        uint32_t getter() const override { return owner._mask; }
    };
    struct _OnInterrupt : _Property<CallbackType&>
    {
        using _Property<CallbackType&>::_Property;
        using _Property<CallbackType&>::operator =;
        CallbackType &getter() const override
        {
            if (owner._pin >= GPIO_PINS_N)
            {
                throw GPIOException("Invalid pin number.");
            }
            return hidden::interrupt_callbacks[owner._pin];
        }
        void setter(const CallbackType &value) const override
        {
            if (owner._pin >= GPIO_PINS_N)
            {
                return;
            }
            hidden::interrupt_callbacks[owner._pin] = value;
        }
    };
    struct _IO : _Property<PinConfig::IO>
    {
        using _Property<PinConfig::IO>::_Property;
        using _Property<PinConfig::IO>::operator =;
        PinConfig::IO getter() const override;
        void setter(const PinConfig::IO value) const override;
    };
    struct _OutMode : _Property<PinConfig::OutMode>
    {
        using _Property<PinConfig::OutMode>::_Property;
        using _Property<PinConfig::OutMode>::operator =;
        PinConfig::OutMode getter() const override;
        void setter(const PinConfig::OutMode value) const override;
    };
    struct _EXTIMode : _Property<PinConfig::EXTIMode>
    {
        using _Property<PinConfig::EXTIMode>::_Property;
        using _Property<PinConfig::EXTIMode>::operator =;
        PinConfig::EXTIMode getter() const override;
        void setter(const PinConfig::EXTIMode value) const override;
    };
    struct _Trigger : _Property<PinConfig::Trigger>
    {
        using _Property<PinConfig::Trigger>::_Property;
        using _Property<PinConfig::Trigger>::operator =;
        PinConfig::Trigger getter() const override;
        void setter(const PinConfig::Trigger value) const override;
    };
    struct _Pull : _Property<PinConfig::Pull>
    {
        using _Property<PinConfig::Pull>::_Property;
        using _Property<PinConfig::Pull>::operator =;
        PinConfig::Pull getter() const override;
        void setter(const PinConfig::Pull value) const override;
    };
    struct _Speed : _Property<PinConfig::Speed>
    {
        using _Property<PinConfig::Speed>::_Property;
        using _Property<PinConfig::Speed>::operator =;
        PinConfig::Speed getter() const override;
        void setter(const PinConfig::Speed value) const override;
    };
    struct _Alternate : _Property<uint8_t>
    {
        using _Property<uint8_t>::_Property;
        using _Property<uint8_t>::operator =;
        uint8_t getter() const override;
        void setter(const uint8_t value) const override;
    };
public:
    hidden::_Port & port;
    // Pin number
    _Pin pin{ *this };
    // Pin mask
    const _Mask mask{ *this };
    _OnInterrupt on_interrupt{ *this };

    Pin(hidden::_Port & port, const uint8_t pin);
    Pin(hidden::_Port & port, const uint8_t pin, const PinConfig & config);

    mutable _IO io{ *this };
    mutable _OutMode out_mode{ *this };
    mutable _EXTIMode exti_mode{ *this };
    mutable _Trigger trigger{ *this };
    mutable _Pull pull{ *this };
    mutable _Speed speed{ *this };
    mutable _Alternate alternate{ *this };

    /**
     * @brief Loads the configuration and enable clock
     * 
     * @return const PinConfig 
     */
    const PinConfig load() const;
    const PinConfig & load(const PinConfig & config) const;

    /**
     * @brief Unloads the configuration, don't affect clock
     * 
     */
    void unload() const;

    /**
     * @brief Sets the pin to high.
     * 
     */
    inline void set() const noexcept
    {
        port.BSRR = _mask;
    }

    /**
     * @brief Sets the pin to low.
     * 
     */
    inline void reset() const noexcept
    {
        port.BSRR = _mask << GPIO_PINS_N;
    }

    /**
     * @brief Toggles the pin.
     * 
     * @return true: Pin is high.
     * @return false: Pin is low.
     */
    inline bool read() const noexcept
    {
        return static_cast<bool>(port.IDR & _mask);
    }

    /**
     * @brief Writes a value to the pin.
     * 
     * @param value The value to write.
     */
    inline void write(const bool value) const noexcept
    {
        uint32_t temp = _mask;
        if (!value)
            temp <<= GPIO_PINS_N;
        port.BSRR = temp;
    }

    /**
     * @brief Toggles the pin.
     * 
     * @return true: Pin was high.
     * @return false: Pin was low.
     */
    inline bool toggle() const noexcept
    {
        uint32_t temp = _mask;
        bool value = bool(port.ODR & temp);
        if (value)
            temp <<= GPIO_PINS_N;
        port.BSRR = temp;
        return value;
    }

    explicit inline operator bool () const noexcept
    {
        return read();
    }

    inline bool operator = (const bool value) const noexcept
    {
        write(value);
        return value;
    }

    /**
     * @brief Get the irqn object
     * 
     * @return nvic::IRQn_Type 
     * @throw GPIOException if the pin is not valid.
     */
    nvic::IRQn_Type get_irqn() const
    {
        switch(_pin)
        {
            case 0: return nvic::EXTI0_IRQn;
            case 1: return nvic::EXTI1_IRQn;
            case 2: return nvic::EXTI2_IRQn;
            case 3: return nvic::EXTI3_IRQn;
            case 4: return nvic::EXTI4_IRQn;
            case 5: case 6: case 7: case 8: case 9:
                return nvic::EXTI9_5_IRQn;
            case 10: case 11: case 12: case 13: case 14: case 15:
                return nvic::EXTI15_10_IRQn;
            default: throw GPIOException();
        }
    }

    /**
     * @brief Enable the interrupt.
     * 
     * @throw GPIOException if the pin is not valid.
     */
    void enable_interrupt(const uint8_t priority=8) const
    {
        auto irqn = get_irqn();
        nvic::enable_irq(irqn);
    }

    void disable_interrupt() const
    {
        nvic::disable_irq(get_irqn());
    }

    /**
     * @brief Set the irq priority object
     * 
     * @param priority 
     * @throw invalid_argument if the priority is not valid.
     */
    void set_irq_priority(const uint8_t priority) const
    {
        nvic::set_priority(get_irqn(), priority);
    }

    /**
     * @brief trigger exti by software
     * 
     */
    void trigger_interrupt() const noexcept;


    bool operator == (const bool value) const noexcept
    {
        return read() == value;
    }

    bool operator == (const Pin & pin) const noexcept
    {
        return port == pin.port && _pin == pin._pin;
    }

    uint32_t operator | (const uint32_t mask) const noexcept
    {
        return _mask | mask;
    }

    uint32_t operator | (const Pin & pin) const noexcept
    {
        return _mask | pin._mask;
    }

    uint32_t operator ~ () const noexcept
    {
        return ~_mask;
    }

    friend class hidden::_Port;
};

    /**
     * @brief Button class, for easy button input handling.
     * 
     * @param port The port of the button.
     * @param pin The pin number of the button.
     * @param active_low Whether the button is low when pressed.
     * @param trigger The trigger mode of the button interrupt.
     */
    class Button
    {
        bool _active_low;
    public:
        enum class Trigger
        {
            None,
            Release,
            Press,
            Both
        };
        Pin pin;
        Trigger trigger;
        CallbackType & on_interrupt = pin.on_interrupt;
        Button(Port & port, const uint8_t pin_n,
            bool active_low=false, Trigger trigger=Trigger::None)
            : _active_low(active_low), pin(port, pin_n)
        {
            PinConfig::Trigger trigger_mode = PinConfig::Trigger::NoTrigger;
            if (trigger != Trigger::None)
            {
                if (trigger == Trigger::Both)
                    trigger_mode = PinConfig::Trigger::RisingFalling;
                else if (trigger == Trigger::Press and active_low)
                    trigger_mode = PinConfig::Trigger::Falling;
                else if (trigger == Trigger::Press and !active_low)
                    trigger_mode = PinConfig::Trigger::Rising;
                else if (trigger == Trigger::Release and active_low)
                    trigger_mode = PinConfig::Trigger::Rising;
                else if (trigger == Trigger::Release and !active_low)
                    trigger_mode = PinConfig::Trigger::Falling;
            }
            PinConfig config(
                PinConfig::Input,
                PinConfig::PullUp,
                PinConfig::Low,
                PinConfig::OpenDrain,
                0,
                PinConfig::Interrupt,
                trigger_mode
            );
            pin.load(config);
        }

        bool pressed() const noexcept
        {
            return _active_low ^ pin.read();
        }

        void enable_interrupt(const uint8_t priority=8) const
        {
            pin.enable_interrupt(priority);
        }

        void disable_interrupt() const
        {
            pin.disable_interrupt();
        }
    };

    /**
     * @brief Switch for output.
     * 
     */
    class Switch
    {
        bool _active_low;
    public:
        Pin pin;
        Switch(Port & port, const uint8_t pin_n, bool active_low=false)
            : _active_low(active_low), pin(port, pin_n)
        {
            PinConfig config(
                PinConfig::Output,
                PinConfig::Pull::NoPull,
                PinConfig::Low,
                PinConfig::PushPull,
                0,
                PinConfig::NoEXTI,
                PinConfig::NoTrigger
            );
            pin.load(config);
            off();
        }

        void on() const noexcept
        {
            pin.write(!_active_low);
        }

        void off() const noexcept
        {
            pin.write(_active_low);
        }

        void toggle() const noexcept
        {
            pin.toggle();
        }

        void set(const bool on) const noexcept
        {
            pin.write(on ^ _active_low);
        }

        bool is_on() const noexcept
        {
            return _active_low ^ pin.read();
        }
    };

}
}
}
