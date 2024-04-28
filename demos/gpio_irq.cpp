#include "core.hpp"

#define LED_PIN 13
#define LED_GPIO_PORT gpio::ports::PortC
#define BUTTON_PIN 0
#define BUTTON_GPIO_PORT gpio::ports::PortA

int main()
{
    using namespace elfe;
    using namespace stm32;
    using gpio::Button;
    using gpio::Pin;
    using gpio::Switch;

    mcu::init();

    Switch led(LED_GPIO_PORT, LED_PIN);

    Button button(BUTTON_GPIO_PORT, BUTTON_PIN);

    button.on_interrupt = [&led, &button]() {
        while (!button.pressed()) {
            led.on();
        }
        led.off();
    };

    button.enable_interrupt();

    while (true) {
        led.toggle();
        clock::delay(1000ms);
    }
}
