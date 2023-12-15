#include <string>
#include "units.hpp"
#include "clock.hpp"
#include "mcu.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "ssd1306.hpp"
#include "texrender.hpp"


#define LED_PIN 13
#define LED_GPIO_PORT PortC
using std::string;

int main()
{
    bool ret = true;

    using namespace vermils;
    using namespace stm32;
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

    Pin scl(PortB, 6), sda(PortB, 7);
    i2c::SoftMaster i2c(sda, scl);
    clock::delay(100ms);
    ssd1306::I2CDisplay display(i2c);
    //ret &= display.set_entire_display_on(false);
    //ret &= display.set_entire_display_on(true);
    ret &= display.fill(0xAF);

    ssd1306::TexRender render(display);

    render.render(0, 0, "Hello, world!\n");
    render << "Goodbye, world!\n";
    render << "abcdefghijklmnopqrstuvwxyz";
    render << 'a';
    render << 'a';
    render << 123;
    render << string("This is a string");
    render.format("I know the answer {}", 42);
    render.render_char(4, 120, 'b');
    render.render_char(7, 128, 'z');

    while (ret)
    {
        led.toggle();
        //HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
        // LED_GPIO_PORT->ODR ^= LED_PIN;
        //HAL_Delay(1000);
        clock::delay(1000ms);
    }
}
