#pragma once

#include <string>
#include <exception>
#include "userconfig.hpp"
#include "Property.hpp"

namespace vermils
{
namespace stm32
{
namespace gpio
{
 class GPIOException : public std::exception
 {
public:
    const char *what() const noexcept override
    {
        return "GPIOException";
    }
 };

// Define some constants
inline constexpr uint8_t GPIO_PINS_N = 16;

class Pin;
namespace hidden
{
    /**
     * @brief GPIO Port Class
     * @warning DO NOT MANNUALLY INSTANTIATE THIS CLASS
     */
    class _Port final
    {
    public:
        #if defined(__VERMIL_STM32F1)
        volatile uint32_t CRL;      /*!< GPIO port configuration register low,  Address offset: 0x00      */
        volatile uint32_t CRH;      /*!< GPIO port configuration register high, Address offset: 0x04      */
        volatile uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x08      */
        volatile uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x0C      */
        volatile uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x10      */
        volatile uint32_t BRR;      /*!< GPIO port bit reset register,          Address offset: 0x14      */
        volatile uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x18      */
        #elif defined(__VERMIL_STM32F4) || defined(__VERMIL_STM32H7)
        volatile uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
        volatile uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
        volatile uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
        volatile uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
        volatile uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
        volatile uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
        volatile uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
        volatile uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
        volatile uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
        #elif __VERMIL_STM32_USE_GENERIC
        Property<uint32_t> BSRR;
        #endif
        _Port() = delete;
        _Port & operator = (const _Port & port) = delete;

        void enable_clock() const;

        void disable_clock() const;

        void set(const uint32_t mask)
        {
            BSRR = mask;
        }

        void set(const Pin & pin);

        void reset(const uint32_t mask)
        {
            BSRR = mask << GPIO_PINS_N;
        }

        void reset(const Pin & pin);

        bool operator == (const _Port & port) const
        {
            return &port == this;
        }

        friend class Pin;
    };
}

bool is_valid_port(const hidden::_Port & port);
bool is_valid_port(const int port_index);
int get_port_index(const hidden::_Port & port);
hidden::_Port & get_port_by_index(const int index);

namespace ports
{
    extern hidden::_Port & PortA;
    extern hidden::_Port & PortB;
    extern hidden::_Port & PortC;
    extern hidden::_Port & PortD;
    extern hidden::_Port & PortE;
    extern hidden::_Port & PortF;
    extern hidden::_Port & PortG;
    extern hidden::_Port & PortH;
    extern hidden::_Port & PortI;
}
using namespace ports;

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
    #if __VERMIL_STM32_USE_GENERIC
    mutable PinConfig _config;
    #endif
public:
    hidden::_Port & port;
    // Pin number
    tricks::Property<uint8_t> pin = {
        [this]() { return _pin; },
        [this](const auto value)
        {
            _pin = value;
            _mask = 1 << value;
        }
    };
    // Pin mask
    const tricks::Property<uint32_t> mask = {
        [this]() -> uint32_t { return _mask; }
    };

    Pin(hidden::_Port & port, const uint8_t pin);
    Pin(hidden::_Port & port, const uint8_t pin, const PinConfig & config);

    mutable tricks::Property<PinConfig::IO> io;
    mutable tricks::Property<PinConfig::OutMode> out_mode;
    mutable tricks::Property<PinConfig::EXTIMode> exti_mode;
    mutable tricks::Property<PinConfig::Trigger> trigger;
    mutable tricks::Property<PinConfig::Pull> pull;
    mutable tricks::Property<PinConfig::Speed> speed;
    mutable tricks::Property<uint8_t> alternate;

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
    void set() const
    {
        port.BSRR = _mask;
    }

    /**
     * @brief Sets the pin to low.
     * 
     */
    void reset() const
    {
        port.BSRR = _mask << GPIO_PINS_N;
    }

    /**
     * @brief Toggles the pin.
     * 
     * @return true: Pin is high.
     * @return false: Pin is low.
     */
    bool read() const
    {
        return bool(port.IDR & _mask);
    }

    /**
     * @brief Writes a value to the pin.
     * 
     * @param value The value to write.
     */
    void write(const bool value) const
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
    bool toggle() const
    {
        uint32_t temp = _mask;
        bool value = bool(port.ODR & temp);
        if (value)
            temp <<= GPIO_PINS_N;
        port.BSRR = temp;
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

    bool operator == (const bool value) const
    {
        return read() == value;
    }

    bool operator == (const Pin & pin) const
    {
        return port == pin.port && _pin == pin._pin;
    }

    uint32_t operator | (const uint32_t mask) const
    {
        return _mask | mask;
    }

    uint32_t operator | (const Pin & pin) const
    {
        return _mask | pin._mask;
    }

    uint32_t operator ~ () const
    {
        return ~_mask;
    }

    friend class hidden::_Port;
};

}
}
}
