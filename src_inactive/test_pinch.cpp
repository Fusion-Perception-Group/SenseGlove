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
    gpio::Button pinch(PortB, 9, true), cap_sensor(PortA, 0);
    gpio::Switch motor_pin0(PortB, 0), motor_pin1(PortB, 1);
    gpio::Switch led(LED_GPIO_PORT, LED_PIN, true);

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
    motor_pin1.off();
    motor_pin0.off();
    led.off();

    while (true)
    {
        if (cap_sensor.pressed())
        {
            led.on();
        }
        else
        {
            led.off();
        }
        if (pinch.pressed())
        {
            motor_pin0.on();
            motor_pin1.off();
            clock::delay(5ms);
            // motor_pin0.off();
            // while (pinch.pressed())
            // {
            // }
        }
        else
        {
        //     motor_pin1.on();
            motor_pin0.off();
            clock::delay(5ms);
            // motor_pin1.off();
            // while (!pinch.pressed())
            // {
            // }
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
