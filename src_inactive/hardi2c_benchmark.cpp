#include <string>
#include "CLK_CFG.h"
#include "time.hpp"
#include "mcu.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "ssd1306.hpp"
#include "texrender.hpp"
#include"units.hpp"

#define FMT_HEADER_ONLY
#include "ffmt.hpp"

#define LED_PIN 13
#define LED_GPIO_PORT PortC

#define BUTTON_PIN GPIO_PIN_0
#define BUTTON_GPIO_PORT GPIOB

void BUTTON_Init();

using std::string;

int main()
{
    bool ret = true;

    using namespace vermils;
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
    PinConfig i2ccfg(PinConfig::AF, PinConfig::VeryHigh, PinConfig::OpenDrain);
    i2ccfg.alternate = 4;
    scl.load(i2ccfg);
    sda.load(i2ccfg);
    //i2c::SoftMaster i2c(sda, scl);
    auto &i2c = i2c::I2c1;
    i2c.init();
    //i2c.set_speed(i2c::Speed::FastPlus);
    i2c.clock_speed = 1_MHz;
    uint32_t start = timer.get_cycles(), test_size=1000;
    
    i2c.select(0x78, false);
    for (unsigned i = 0; i < test_size; ++i)
    {
        try
        {
        i2c.write_byte(0xe5);

        }
        catch (...)
        {
        }
    }
    i2c.end();
    uint32_t end = timer.get_cycles();
    double sec = double(end - start) / stm32::SystemCoreClock;
    double kbps = test_size * 8.0 / 1000 / sec;

    timer.delay_ms(100);
    ssd1306::I2CDisplay display(i2c);
    //ret &= display.init();
    //ret &= display.set_entire_display_on(false);
    //ret &= display.set_entire_display_on(true);
    //ret &= display.fill(0xAF);
    display.clear();

    ssd1306::TexRender render(display);

    render.render(0, 12, "I2C Benchmark!\n");
    render << ffmt::format("Speed: {} kbit/s", kbps);
    render << ffmt::format("Clock: {} Khz", i2c.clock_speed() / 1000);

    while (ret)
    {
        led.toggle();
        //HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
        // LED_GPIO_PORT->ODR ^= LED_PIN;
        //HAL_Delay(1000);
        timer.delay_ms(1000);
    }
}
