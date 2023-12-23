#include <string>
#include <chrono>
#include "mcu.hpp"
#include "time.hpp"
#include "nvic.hpp"
#include "gpio.hpp"
#include "clock.hpp"

#define LED_PIN 13
#define LED_GPIO_PORT PortC

using std::string;

int main()
{
    using std::chrono::operator ""s;
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

    auto &tim = clock::Tim1;
    tim.init();
    tim.set_repetition(100);
    tim.set_frequency(100);
    tim.on_reload = [&led, &tim]()
    {
        if (led.read())
        {
            led.reset();
            tim.set_repetition(200);
        }
        else
        {
            led.set();
            tim.set_repetition(100);
        }
    };
    tim.enable_interrupt_reload();
    tim.start();

    //render.render("Hello, world!\n", 0, 0);

    while (true)
    {
    }
}
