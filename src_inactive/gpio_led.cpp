#include <chrono>
#include "mcu.hpp"
#include "time.hpp"
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

    while (true)
    {
        led.set();
        clock::delay(100ms);
        led.reset();
        clock::delay(200ms);
    }
}