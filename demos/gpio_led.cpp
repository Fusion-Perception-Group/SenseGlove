#include "gpio/gpio.hpp"
#include "mcu/mcu.hpp"

#define LED_PIN 13
#define LED_GPIO_PORT gpio::ports::PortC

int main()
{
    using namespace elfe::stm32;
    using gpio::Pin;
    using gpio::Switch;

    mcu::init();

    Switch led(LED_GPIO_PORT, LED_PIN);

    while (true) {
        led.on();
        clock::delay(100ms);
        led.off();
        clock::delay(200ms);
    }
}