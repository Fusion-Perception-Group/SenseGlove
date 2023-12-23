#include <string>
#include <chrono>
#include "mcu.hpp"
#include "time.hpp"
#include "nvic.hpp"
#include "gpio.hpp"

#define LED_PIN 13
#define LED_GPIO_PORT PortC

using std::string;

int main()
{
    using std::chrono::operator ""ms;
    using std::chrono::operator ""ns;
    using namespace vermils;
    using namespace stm32;
    using time::HighResTimer;
    using gpio::Pin;
    using gpio::PinConfig;
    using namespace gpio::ports;

    mcu::init();

    PinConfig config(
        PinConfig::Output,
        PinConfig::VeryHigh,
        PinConfig::PushPull
        );
    Pin led(LED_GPIO_PORT, LED_PIN, config);

    nvic::set_priority_group(nvic::Pre2_Sub2);

    Pin button(PortA, 0, PinConfig(PinConfig::Input,
        PinConfig::PullUp, PinConfig::Interrupt, PinConfig::Falling));

    button.on_interrupt = [&led, &button]()
    {
        while (!button.read())
        {
            led.set();
        }
    };

    button.enable_interrupt();

    while (true)
    {
        led.toggle();
        clock::delay(1000ms);
    }
}
