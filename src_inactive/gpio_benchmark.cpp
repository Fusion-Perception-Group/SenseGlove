#include <string>
#include "mcu.hpp"
#include "time.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "ssd1306.hpp"
#include "texrender.hpp"

#define FMT_HEADER_ONLY
#include "ffmt.hpp"

#define LED_PIN 13
#define LED_GPIO_PORT PortC

using std::string;

int main()
{
    bool ret = true;

    using namespace vms;
    using namespace stm32;
    using time::HighResTimer;
    using gpio::Pin;
    using gpio::PinConfig;
    using namespace gpio::ports;

    mcu::init();

    HighResTimer timer;

    PinConfig config(
        PinConfig::Output,
        PinConfig::VeryHigh,
        PinConfig::PushPull
        );
    Pin led(LED_GPIO_PORT, LED_PIN, config);

    Pin scl(PortB, 6), sda(PortB, 7);

    scl.io = PinConfig::Output;
    scl.out_mode = PinConfig::OpenDrain;

    uint32_t start = timer.get_cycles(), test_size=1000;

    for (unsigned i = 0; i < test_size; ++i)
    {
        scl.set();
        while(!scl.read());
        scl.reset();
        while(scl.read());
    }
    uint32_t end = timer.get_cycles();
    double sec = double(end - start) / stm32::SystemCoreClock;
    double kbps = test_size / 1000.0 / sec;

    i2c::SoftMaster i2c(sda, scl, 1);
    timer.delay_ms(100);
    i2c.delay_us = 1;
    ssd1306::I2CDisplay display(i2c);
    //ret &= display.init();
    //ret &= display.set_entire_display_on(false);
    //ret &= display.set_entire_display_on(true);
    //ret &= display.fill(0xAF);
    display.clear();

    ssd1306::TexRender render(display);

    render.render(0, 12, "GPIO Benchmark!\n");
    render << ffmt::format("Speed: {} kflip/s", kbps);

    while (ret)
    {
        led.toggle();
        //HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
        // LED_GPIO_PORT->ODR ^= LED_PIN;
        //HAL_Delay(1000);
        timer.delay_ms(1000);
    }
}