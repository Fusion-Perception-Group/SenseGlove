#include <string>
#include <tuple>
#include <chrono>
#include "mcu.hpp"
#include "units.hpp"
#include "ssd1306.hpp"
#include "texrender.hpp"
#include "ffmt.hpp"
#include "wit.hpp"

#define LED_PIN 13
#define LED_GPIO_PORT PortC

using std::string;

int main()
{
    using namespace vermils;
    using namespace stm32;
    using gpio::Pin;
    using gpio::PinConfig;
    using namespace gpio::ports;

    mcu::init();
    Pin snap(PortB, 9, PinConfig(PinConfig::Input, PinConfig::PullUp)),
        cap_sensor(PortA, 0, PinConfig(PinConfig::Input, PinConfig::PullDown));
    PinConfig out_cfg = PinConfig(PinConfig::Output, PinConfig::VeryHigh, PinConfig::PushPull);
    Pin motor_pin0(PortB, 0, out_cfg), motor_pin1(PortB, 1, out_cfg);
    Pin led(LED_GPIO_PORT, LED_PIN, out_cfg);

    // if (cap_sensor.read())
    // {
    //     led.reset();
    //     motor_pin0.set();
    //     clock::delay(5ms);
    //     motor_pin0.reset();
    // }
    // else
    // {
    //     led.set();
    // }
    motor_pin1.reset();
    led.set();

    while (true)
    {
        if (cap_sensor.read())
        {
            led.reset();
        }
        else
        {
            led.set();
        }
        // if (!snap.read())
        // {
        //     if (cap_sensor.read())
        //     {
        //         led.reset();
        //     }
        //     else
        //     {
        //         led.set();
        //     }
        //     motor_pin0.set();
        //     clock::delay(5ms);
        //     motor_pin0.reset();
        //     clock::delay(5ms);
        // }
        // else
        // {
        //     motor_pin0.reset();
        // }
    }
}
